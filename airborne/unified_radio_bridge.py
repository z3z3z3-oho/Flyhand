#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unified aircraft-side radio bridge for SO-101.

Supported ground protocol:
- CRC32 framed text:
    MODE,HOLD
    MODE,TELEOP
    MODE,GESTURE
    MODE,AUTO
    H,<sequence>
    J,<sequence>,<six joint values in leader units>
    GESTURE_RESET
    start / hover / next / stop / return
- Plain JSON gesture stream:
    {"mode":"servo","vx":...,"vy":...,"vz":...,"grip":true}

ROS outputs:
- /control/mode                 std_msgs/String
- /arm/teleop_joint_target     sensor_msgs/JointState
- /gesture/arm_delta           geometry_msgs/Twist
- /gesture/gripper_close       std_msgs/Bool
- /gesture/reset               std_msgs/Bool
- /harvest/command             std_msgs/String

Important unit convention:
- Ground J frames contain SO-101 leader joint values in degrees.
- shoulder_pan..wrist_roll are converted to radians for JointState.
- gripper stays in the leader calibration unit because so101_arm_interface
  maps it using teleop_gripper_input_close/open.
"""

from __future__ import annotations

import json
import math
import threading
import time
import zlib
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise RuntimeError(
        "pyserial 未安装，请执行: sudo apt install python3-serial"
    ) from exc


BUILD_ID = "2026-07-09-unified-radio-bridge-v1"

MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

VALID_MODES = {"HOLD", "TELEOP", "GESTURE", "AUTO"}
VALID_TASK_COMMANDS = {"start", "hover", "next", "stop", "return"}


@dataclass
class BridgeStats:
    lines: int = 0
    valid_frames: int = 0
    invalid_frames: int = 0
    teleop_frames: int = 0
    gesture_frames: int = 0
    last_sequence: Optional[int] = None
    last_valid_time: float = 0.0


def crc32_hex(body: str) -> str:
    return f"{zlib.crc32(body.encode('ascii')) & 0xFFFFFFFF:08X}"


def decode_crc_frame(line: str) -> tuple[Optional[str], Optional[str]]:
    """Return (body, error). JSON lines are handled elsewhere."""
    if "*" not in line:
        return None, "missing checksum separator"

    body, supplied = line.rsplit("*", 1)
    supplied = supplied.strip().upper()

    if len(supplied) != 8:
        return None, "checksum length"

    try:
        int(supplied, 16)
    except ValueError:
        return None, "checksum format"

    expected = crc32_hex(body)
    if supplied != expected:
        return None, f"checksum mismatch expected={expected} got={supplied}"

    return body, None


class UnifiedRadioBridge(Node):
    def __init__(self) -> None:
        super().__init__("unified_radio_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("read_timeout_sec", 0.05)
        self.declare_parameter("max_line_bytes", 512)
        self.declare_parameter("feedback_enabled", True)
        self.declare_parameter("debug_raw_lines", False)
        self.declare_parameter("teleop_log_interval", 30)
        self.declare_parameter("publish_initial_hold", True)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.read_timeout = max(
            0.01,
            float(self.get_parameter("read_timeout_sec").value),
        )
        self.max_line_bytes = max(
            128,
            int(self.get_parameter("max_line_bytes").value),
        )
        self.feedback_enabled = bool(
            self.get_parameter("feedback_enabled").value
        )
        self.debug_raw_lines = bool(
            self.get_parameter("debug_raw_lines").value
        )
        self.teleop_log_interval = max(
            1,
            int(self.get_parameter("teleop_log_interval").value),
        )

        self.mode_pub = self.create_publisher(
            String,
            "/control/mode",
            10,
        )
        self.teleop_pub = self.create_publisher(
            JointState,
            "/arm/teleop_joint_target",
            10,
        )
        self.gesture_delta_pub = self.create_publisher(
            Twist,
            "/gesture/arm_delta",
            10,
        )
        self.gesture_gripper_pub = self.create_publisher(
            Bool,
            "/gesture/gripper_close",
            10,
        )
        self.gesture_reset_pub = self.create_publisher(
            Bool,
            "/gesture/reset",
            10,
        )
        self.command_pub = self.create_publisher(
            String,
            "/harvest/command",
            10,
        )

        self._mode = "HOLD"
        self._running = True
        self._write_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._stats = BridgeStats()
        self._last_grip: Optional[bool] = None
        self._last_invalid_log = 0.0

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.read_timeout,
            write_timeout=0.5,
        )

        # Discard bytes left by a previous process. Do not reset output after
        # this point because it could drop feedback already queued by the UART.
        time.sleep(0.2)
        self._serial.reset_input_buffer()

        self._reader = threading.Thread(
            target=self._read_loop,
            name="unified-radio-rx",
            daemon=True,
        )
        self._reader.start()

        self.create_timer(2.0, self._publish_diagnostics)

        if bool(self.get_parameter("publish_initial_hold").value):
            self._publish_mode("HOLD")

        self.get_logger().info(
            f"{BUILD_ID} ready | port={self.port} | "
            f"baud={self.baudrate} | max_line={self.max_line_bytes}"
        )
        print(f"[RADIO BRIDGE] {BUILD_ID}")

    # ------------------------------------------------------------------
    # Lifecycle and serial feedback
    # ------------------------------------------------------------------

    def destroy_node(self) -> bool:
        self._running = False
        try:
            if self._serial.is_open:
                self._serial.close()
        except Exception:
            pass

        if self._reader.is_alive():
            self._reader.join(timeout=1.0)

        return super().destroy_node()

    def _feedback(self, text: str) -> None:
        if not self.feedback_enabled:
            return

        payload = (text.rstrip("\r\n") + "\n").encode(
            "ascii",
            errors="replace",
        )
        try:
            with self._write_lock:
                self._serial.write(payload)
                self._serial.flush()
        except Exception as exc:
            self.get_logger().warn(
                f"feedback write failed: {exc}",
                throttle_duration_sec=2.0,
            )

    def _mark_valid(self) -> None:
        with self._stats_lock:
            self._stats.valid_frames += 1
            self._stats.last_valid_time = time.monotonic()

    def _mark_invalid(self, line: str, reason: str) -> None:
        with self._stats_lock:
            self._stats.invalid_frames += 1

        # Keep the feedback short so it cannot congest the transparent radio.
        safe_prefix = (
            line[:32]
            .replace(",", "_")
            .replace("*", "_")
            .replace(" ", "")
        )
        self._feedback(f"ERR,INVALID,{safe_prefix}")

        now = time.monotonic()
        if now - self._last_invalid_log >= 0.5:
            self._last_invalid_log = now
            self.get_logger().warn(
                f"Invalid radio line: reason={reason}, line={line!r}"
            )

    # ------------------------------------------------------------------
    # Reader and parser
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        buffer = bytearray()

        while self._running and rclpy.ok():
            try:
                chunk = self._serial.read(
                    max(1, min(256, self._serial.in_waiting or 1))
                )
                if not chunk:
                    continue

                buffer.extend(chunk)

                # A corrupted or missing newline must not grow memory forever.
                if len(buffer) > self.max_line_bytes * 4:
                    buffer.clear()
                    self._feedback("ERR,BUFFER_RESET")
                    continue

                while b"\n" in buffer:
                    raw, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    raw = raw.rstrip(b"\r")

                    if not raw:
                        continue
                    if len(raw) > self.max_line_bytes:
                        self._mark_invalid(
                            raw[:64].decode("ascii", errors="replace"),
                            "line too long",
                        )
                        continue

                    line = raw.decode(
                        "ascii",
                        errors="replace",
                    ).strip()
                    if not line:
                        continue

                    with self._stats_lock:
                        self._stats.lines += 1

                    if self.debug_raw_lines:
                        self.get_logger().info(f"RX {line!r}")

                    self._handle_line(line)

            except serial.SerialException as exc:
                self.get_logger().error(f"Serial read failed: {exc}")
                time.sleep(0.2)
            except Exception as exc:
                self.get_logger().error(
                    f"Unexpected radio error: {exc}",
                    throttle_duration_sec=1.0,
                )

    def _handle_line(self, line: str) -> None:
        if line.startswith("{"):
            self._handle_json(line)
            return

        body, error = decode_crc_frame(line)
        if body is None:
            self._mark_invalid(line, error or "invalid CRC frame")
            return

        self._mark_valid()
        self._handle_body(body)

    def _handle_body(self, body: str) -> None:
        if body.startswith("MODE,"):
            mode = body.split(",", 1)[1].strip().upper()
            if mode not in VALID_MODES:
                self._mark_invalid(body, f"unsupported mode {mode}")
                return
            self._publish_mode(mode)
            self._feedback(f"OK,MODE,{mode}")
            return

        if body.startswith("J,"):
            self._handle_joint_frame(body)
            return

        if body.startswith("H,"):
            # H is an explicit safe hold heartbeat. It is valid even if the
            # preceding MODE,HOLD acknowledgement was lost.
            self._publish_mode("HOLD")
            self._feedback("OK,HOLD")
            return

        if body == "GESTURE_RESET":
            self.gesture_reset_pub.publish(Bool(data=True))
            self._feedback("OK,GESTURE_RESET")
            return

        task = body.strip().lower()
        if task in VALID_TASK_COMMANDS:
            self.command_pub.publish(String(data=task))
            self._feedback(f"OK,CMD,{task.upper()}")
            return

        self._mark_invalid(body, "unsupported framed command")

    # ------------------------------------------------------------------
    # TELEOP
    # ------------------------------------------------------------------

    def _handle_joint_frame(self, body: str) -> None:
        parts = body.split(",")
        if len(parts) != 8:
            self._mark_invalid(
                body,
                f"J field count {len(parts)}, expected 8",
            )
            return

        try:
            sequence = int(parts[1])
            values = [float(item) for item in parts[2:]]
        except ValueError:
            self._mark_invalid(body, "J number format")
            return

        if not 0 <= sequence <= 65535:
            self._mark_invalid(body, "sequence range")
            return
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            self._mark_invalid(body, "invalid joint values")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MOTOR_NAMES)

        # so101_arm_interface converts the five arm joints from radians back to
        # degrees. Its gripper path intentionally expects the calibrated leader
        # input value directly.
        msg.position = [
            math.radians(values[0]),
            math.radians(values[1]),
            math.radians(values[2]),
            math.radians(values[3]),
            math.radians(values[4]),
            values[5],
        ]

        self.teleop_pub.publish(msg)

        with self._stats_lock:
            self._stats.teleop_frames += 1
            self._stats.last_sequence = sequence
            count = self._stats.teleop_frames
            self._stats.last_valid_time = time.monotonic()

        # Do not acknowledge every J frame; that would flood the return link.
        if count == 1 or count % self.teleop_log_interval == 0:
            self.get_logger().info(
                f"TELEOP J accepted: seq={sequence}, count={count}, "
                f"pan={values[0]:.1f}, lift={values[1]:.1f}, "
                f"elbow={values[2]:.1f}, gripper={values[5]:.1f}"
            )
            self._feedback(f"OK,J,{sequence}")

    # ------------------------------------------------------------------
    # GESTURE
    # ------------------------------------------------------------------

    def _handle_json(self, line: str) -> None:
        try:
            data = json.loads(line)
        except json.JSONDecodeError as exc:
            self._mark_invalid(line, f"JSON: {exc}")
            return

        if not isinstance(data, dict):
            self._mark_invalid(line, "JSON must be object")
            return

        mode = str(data.get("mode", "")).strip().lower()
        if mode not in {"servo", "gesture", ""}:
            self._mark_invalid(line, f"unsupported JSON mode {mode}")
            return

        try:
            vx = float(data.get("vx", 0.0))
            vy = float(data.get("vy", 0.0))
            vz = float(data.get("vz", 0.0))
        except (TypeError, ValueError):
            self._mark_invalid(line, "gesture number format")
            return

        if not all(math.isfinite(value) for value in (vx, vy, vz)):
            self._mark_invalid(line, "non-finite gesture value")
            return

        # Clamp malformed senders rather than forwarding dangerous values.
        vx = max(-1.0, min(1.0, vx))
        vy = max(-1.0, min(1.0, vy))
        vz = max(-1.0, min(1.0, vz))

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        self.gesture_delta_pub.publish(twist)

        grip_value = data.get(
            "grip",
            data.get(
                "gripper_close",
                data.get("gripper", self._last_grip),
            ),
        )
        if grip_value is not None:
            grip = bool(grip_value)
            # Re-publish every packet to keep downstream state deterministic.
            self.gesture_gripper_pub.publish(Bool(data=grip))
            self._last_grip = grip

        self._mark_valid()
        with self._stats_lock:
            self._stats.gesture_frames += 1
            self._stats.last_valid_time = time.monotonic()

    # ------------------------------------------------------------------
    # ROS state and diagnostics
    # ------------------------------------------------------------------

    def _publish_mode(self, mode: str) -> None:
        self._mode = mode
        self.mode_pub.publish(String(data=mode))
        self.get_logger().warn(f"Control mode -> {mode}")

    def _publish_diagnostics(self) -> None:
        with self._stats_lock:
            stats = BridgeStats(**vars(self._stats))

        age = (
            time.monotonic() - stats.last_valid_time
            if stats.last_valid_time > 0
            else float("inf")
        )
        age_text = f"{age:.2f}s" if math.isfinite(age) else "never"

        self.get_logger().info(
            f"radio stats | mode={self._mode} | lines={stats.lines} | "
            f"valid={stats.valid_frames} | invalid={stats.invalid_frames} | "
            f"J={stats.teleop_frames} | gesture={stats.gesture_frames} | "
            f"last_seq={stats.last_sequence} | last_valid={age_text}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[UnifiedRadioBridge] = None
    try:
        node = UnifiedRadioBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
