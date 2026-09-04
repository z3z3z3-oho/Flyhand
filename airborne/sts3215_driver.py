#!/usr/bin/env python3
"""Minimal STS3215 driver with LeRobot-compatible SO-101 calibration."""

import json
import os
import threading
import time

import serial


SERVO_MAX_RAW = 4095.0
MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def _default_calibration_path():
    base = os.path.dirname(os.path.abspath(__file__))
    candidates = (
        os.path.join(base, "config", "drone_follower.json"),
        os.path.join(base, "drone_follower.json"),
    )
    return next((path for path in candidates if os.path.isfile(path)), candidates[0])


def load_calibration(path):
    if not path:
        path = _default_calibration_path()
    path = os.path.abspath(os.path.expanduser(path))
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    missing = [name for name in MOTOR_NAMES if name not in data]
    if missing:
        raise ValueError(f"Calibration is missing motors: {missing}")

    ids = set()
    calibration = {}
    for name in MOTOR_NAMES:
        entry = data[name]
        motor_id = int(entry["id"])
        minimum = int(entry["range_min"])
        maximum = int(entry["range_max"])
        drive_mode = int(entry.get("drive_mode", 0))
        if motor_id in ids:
            raise ValueError(f"Duplicate motor id in calibration: {motor_id}")
        if not 0 <= minimum < maximum <= int(SERVO_MAX_RAW):
            raise ValueError(f"Invalid calibrated range for {name}: {minimum}..{maximum}")
        if drive_mode not in (0, 1):
            raise ValueError(f"Invalid drive_mode for {name}: {drive_mode}")
        ids.add(motor_id)
        calibration[name] = {
            "id": motor_id,
            "drive_mode": drive_mode,
            "homing_offset": int(entry.get("homing_offset", 0)),
            "range_min": minimum,
            "range_max": maximum,
        }
    return calibration, path


def raw_to_value(calibration, name, raw):
    entry = calibration[name]
    minimum = float(entry["range_min"])
    maximum = float(entry["range_max"])
    raw = max(minimum, min(maximum, float(raw)))
    if entry["drive_mode"] == 1:
        raw = minimum + maximum - raw
    if name == "gripper":
        return (raw - minimum) * 100.0 / (maximum - minimum)
    midpoint = (minimum + maximum) / 2.0
    return (raw - midpoint) * 360.0 / SERVO_MAX_RAW


def value_to_raw(calibration, name, value):
    entry = calibration[name]
    minimum = float(entry["range_min"])
    maximum = float(entry["range_max"])
    if name == "gripper":
        percent = max(0.0, min(100.0, float(value)))
        raw = minimum + percent * (maximum - minimum) / 100.0
    else:
        midpoint = (minimum + maximum) / 2.0
        raw = midpoint + float(value) * SERVO_MAX_RAW / 360.0
        raw = max(minimum, min(maximum, raw))
    if entry["drive_mode"] == 1:
        raw = minimum + maximum - raw
    return raw


class STS3215Driver:
    def __init__(self, port="/dev/ttyACM0", baudrate=1_000_000):
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        self.lock = threading.Lock()
        print(f"Serial connected: {port} ({baudrate} baud)")

    @staticmethod
    def checksum(data):
        return (~sum(data)) & 0xFF

    def read_pos_raw(self, motor_id):
        if not self.ser:
            return None
        with self.lock:
            try:
                payload = [motor_id, 0x04, 0x02, 0x38, 0x02]
                self.ser.write(bytes([0xFF, 0xFF] + payload + [self.checksum(payload)]))
                time.sleep(0.002)
                waiting = self.ser.in_waiting
                response = self.ser.read(waiting if waiting >= 8 else 8)
                for index in range(max(0, len(response) - 7)):
                    if (
                        response[index] == 0xFF
                        and response[index + 1] == 0xFF
                        and response[index + 2] == motor_id
                    ):
                        return response[index + 5] + (response[index + 6] << 8)
            except (OSError, serial.SerialException):
                return None
        return None

    def write_pos_raw(self, motor_id, raw_position, speed=0):
        if not self.ser:
            return
        raw_position = max(0, min(4095, int(round(raw_position))))
        speed = max(0, min(65535, int(speed)))
        with self.lock:
            data = [
                raw_position & 0xFF,
                (raw_position >> 8) & 0xFF,
                0,
                0,
                speed & 0xFF,
                (speed >> 8) & 0xFF,
            ]
            payload = [motor_id, len(data) + 3, 0x03, 0x2A] + data
            self.ser.write(bytes([0xFF, 0xFF] + payload + [self.checksum(payload)]))


class SO101Follower:
    def __init__(self, config):
        self.calibration, self.calibration_path = load_calibration(
            getattr(config, "calibration_path", None)
        )
        self.driver = STS3215Driver(config.port)
        self.motor_names = list(MOTOR_NAMES)
        self.motor_ids = {
            name: self.calibration[name]["id"] for name in self.motor_names
        }
        self.body_speed = getattr(config, "body_speed", 250)
        self.gripper_speed = getattr(config, "gripper_speed", 250)
        print(f"Follower calibration loaded: {self.calibration_path}")

    def connect(self):
        if self.driver.ser is None:
            raise RuntimeError("Serial port error")

    def get_observation(self):
        observation = {}
        for name in self.motor_names:
            raw = self.driver.read_pos_raw(self.motor_ids[name])
            if raw is not None:
                observation[f"{name}.pos"] = raw_to_value(self.calibration, name, raw)
            time.sleep(0.001)
        return observation

    def set_action(self, action_dict):
        if not self.driver.ser:
            return
        for raw_name, target in action_dict.items():
            name = raw_name[:-4] if raw_name.endswith(".pos") else raw_name
            if name not in self.motor_ids:
                continue
            try:
                value = float(target.item() if hasattr(target, "item") else target)
                raw = value_to_raw(self.calibration, name, value)
                speed = self.gripper_speed if name == "gripper" else self.body_speed
                self.driver.write_pos_raw(self.motor_ids[name], raw, speed=speed)
            except (TypeError, ValueError, OSError, serial.SerialException):
                continue

    def disconnect(self):
        if self.driver.ser:
            self.driver.ser.close()
