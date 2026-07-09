# -*- coding: utf-8 -*-
"""Self-contained SO-101 leader arm driver for STS3215 servos.

This module keeps the interface expected by ``ground_so101_sender.py``:

    SO101LeaderConfig(port=..., id=..., use_degrees=True)
    SO101Leader.connect()
    SO101Leader.get_action()
    SO101Leader.disconnect()

The STS3215 bus uses the Feetech/SCS packet format:
    FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Iterable

import serial


# STS3215 / Feetech SCS protocol constants.
_HEADER = b"\xFF\xFF"
_INST_READ = 0x02
_PRESENT_POSITION_ADDR = 0x38
_PRESENT_POSITION_SIZE = 2
_DEFAULT_BAUDRATE = 1_000_000

# Six joints used by the SO-101 leader.
MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class ServoCommunicationError(RuntimeError):
    """Raised when an STS3215 transaction does not produce a valid response."""


def _checksum(data: Iterable[int]) -> int:
    """Return the Feetech/SCS inverted 8-bit checksum."""
    return (~sum(data)) & 0xFF


def _build_read_packet(servo_id: int, address: int, size: int) -> bytes:
    """Build one STS3215 READ_DATA request packet."""
    if not 0 <= servo_id <= 253:
        raise ValueError(f"Invalid servo ID: {servo_id}")
    if not 0 <= address <= 255:
        raise ValueError(f"Invalid register address: {address}")
    if not 1 <= size <= 255:
        raise ValueError(f"Invalid read size: {size}")

    # LENGTH equals parameter count + 2 (instruction and checksum).
    body = [
        servo_id,
        0x04,
        _INST_READ,
        address,
        size,
    ]
    return _HEADER + bytes(body) + bytes([_checksum(body)])


def _extract_status_packet(
    buffer: bytearray,
    servo_id: int,
    expected_params: int,
    transmitted_packet: bytes,
) -> tuple[int, bytes] | None:
    """Find and remove one valid status packet from ``buffer``.

    Some USB half-duplex adapters echo the transmitted request. The exact
    request packet is explicitly ignored so that its address bytes are not
    mistaken for a servo position.
    """
    expected_length_field = expected_params + 2
    total_size = expected_params + 6
    index = 0

    while index <= len(buffer) - total_size:
        if buffer[index:index + 2] != _HEADER:
            index += 1
            continue

        # Drop an exact adapter echo of our request.
        if buffer[index:index + len(transmitted_packet)] == transmitted_packet:
            del buffer[index:index + len(transmitted_packet)]
            continue

        if buffer[index + 2] != servo_id:
            index += 1
            continue

        if buffer[index + 3] != expected_length_field:
            index += 1
            continue

        frame = bytes(buffer[index:index + total_size])
        if _checksum(frame[2:-1]) != frame[-1]:
            index += 1
            continue

        error_byte = frame[4]
        params = frame[5:-1]
        del buffer[:index + total_size]
        return error_byte, params

    # Keep only a short tail so the buffer cannot grow indefinitely while
    # preserving a possible partial packet header for the next read.
    if len(buffer) > 64:
        del buffer[:-16]
    return None


def _read_servo_raw(
    ser: serial.Serial,
    servo_id: int,
    *,
    retries: int = 3,
    response_timeout: float = 0.04,
) -> int:
    """Read the STS3215 present position register and return its raw value.

    Args:
        ser: Open serial port connected to the half-duplex servo bus.
        servo_id: Target servo ID.
        retries: Number of complete request attempts.
        response_timeout: Maximum wait per attempt in seconds.

    Raises:
        ServoCommunicationError: No valid response or servo reports an error.
    """
    request = _build_read_packet(
        servo_id,
        _PRESENT_POSITION_ADDR,
        _PRESENT_POSITION_SIZE,
    )
    last_received = b""

    for attempt in range(1, retries + 1):
        # Transactions are synchronous and only one servo is addressed at a
        # time, so stale bytes from a previous failed transaction are unsafe.
        ser.reset_input_buffer()
        ser.write(request)
        ser.flush()

        received = bytearray()
        deadline = time.monotonic() + response_timeout

        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            if waiting:
                received.extend(ser.read(waiting))
                parsed = _extract_status_packet(
                    received,
                    servo_id,
                    _PRESENT_POSITION_SIZE,
                    request,
                )
                if parsed is not None:
                    error_byte, params = parsed
                    if error_byte != 0:
                        raise ServoCommunicationError(
                            f"Servo ID {servo_id} returned error "
                            f"0x{error_byte:02X}"
                        )
                    return params[0] | (params[1] << 8)
            else:
                time.sleep(0.0005)

        last_received = bytes(received)
        if attempt < retries:
            time.sleep(0.003)

    raw_text = last_received.hex(" ").upper() if last_received else "<empty>"
    raise ServoCommunicationError(
        f"No valid response from servo ID {servo_id} after {retries} attempts; "
        f"last RX: {raw_text}. Check servo power, half-duplex data wiring, "
        f"common ground, ID, and 1,000,000 baud configuration."
    )


def load_calibration(file_path: str | Path) -> dict:
    """Load and minimally validate a LeRobot-style calibration JSON file."""
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {path}")

    with path.open("r", encoding="utf-8") as file:
        calibration = json.load(file)

    missing = [name for name in MOTOR_NAMES if name not in calibration]
    if missing:
        raise ValueError(
            "Calibration file is missing joints: " + ", ".join(missing)
        )

    required_fields = {"id", "range_min", "range_max", "drive_mode"}
    for name in MOTOR_NAMES:
        absent = required_fields.difference(calibration[name])
        if absent:
            raise ValueError(
                f"Calibration entry '{name}' is missing fields: "
                + ", ".join(sorted(absent))
            )

    return calibration


def raw_to_degree(calib: dict, joint: str, raw: int) -> float:
    """Convert a calibrated raw encoder value into joint degrees."""
    cfg = calib[joint]
    range_min = float(cfg["range_min"])
    range_max = float(cfg["range_max"])
    half_range = (range_max - range_min) / 2.0

    if abs(half_range) < 1e-9:
        raise ValueError(f"Invalid zero-width calibration range for {joint}")

    midpoint = (range_min + range_max) / 2.0
    degrees = ((float(raw) - midpoint) / half_range) * 180.0
    return -degrees if int(cfg["drive_mode"]) == 1 else degrees


class SO101LeaderConfig:
    """Configuration used by :class:`SO101Leader`."""

    def __init__(
        self,
        port: str,
        id: str = "blue",
        use_degrees: bool = True,
    ):
        if not port:
            raise ValueError("Leader serial port cannot be empty")
        if not id:
            raise ValueError("Calibration ID cannot be empty")

        self.port = port
        self.id = id
        self.use_degrees = use_degrees
        self.calibration_path = str(
            Path(__file__).resolve().parent / "config" / f"{id}.json"
        )


class SO101Leader:
    """Read the six passive SO-101 leader-arm servos over one serial bus."""

    def __init__(self, config: SO101LeaderConfig):
        self.config = config
        self.calibration: dict | None = None
        self._ser: serial.Serial | None = None
        self._ids: dict[str, int] = {}

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self) -> None:
        if self.is_connected:
            return

        calibration = load_calibration(self.config.calibration_path)
        ids = {name: int(calibration[name]["id"]) for name in MOTOR_NAMES}

        duplicate_ids = {
            servo_id for servo_id in ids.values()
            if list(ids.values()).count(servo_id) > 1
        }
        if duplicate_ids:
            values = ", ".join(str(value) for value in sorted(duplicate_ids))
            raise ValueError(f"Duplicate servo IDs in calibration: {values}")

        try:
            ser = serial.Serial(
                port=self.config.port,
                baudrate=_DEFAULT_BAUDRATE,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0.05,
                write_timeout=0.05,
            )
            # Give Windows USB serial drivers a moment to settle after opening.
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            # Do not leave a partially created port object behind.
            try:
                ser.close()  # type: ignore[possibly-undefined]
            except Exception:
                pass
            raise

        self.calibration = calibration
        self._ids = ids
        self._ser = ser

    def get_action(self) -> dict[str, float | int]:
        if not self.is_connected or self._ser is None:
            raise RuntimeError("SO101 leader is not connected")
        if self.calibration is None:
            raise RuntimeError("SO101 leader calibration is not loaded")

        action: dict[str, float | int] = {}
        for name in MOTOR_NAMES:
            servo_id = self._ids[name]
            try:
                raw = _read_servo_raw(self._ser, servo_id)
            except ServoCommunicationError as exc:
                raise RuntimeError(
                    f"Failed to read {name} (servo ID {servo_id}): {exc}"
                ) from exc

            action[f"{name}.pos"] = (
                raw_to_degree(self.calibration, name, raw)
                if self.config.use_degrees
                else raw
            )

        return action

    def disconnect(self) -> None:
        ser = self._ser
        self._ser = None
        if ser is not None and ser.is_open:
            ser.close()

    def __enter__(self) -> "SO101Leader":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()
