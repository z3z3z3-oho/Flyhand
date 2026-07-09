#!/usr/bin/env python3
"""Unified telemetry bridge for autonomous, leader/follower and gesture control.

This node is the *only* owner of the telemetry serial port. It accepts:

* Task commands: start / hover / next / grasp / stop / return / calibrate
* Control modes: MODE,AUTO / MODE,TELEOP / MODE,GESTURE / MODE,HOLD / MODE,STOW
* Joint frames: J,seq,pan,lift,elbow,wrist_flex,wrist_roll,gripper
* Hold frames: H,seq
* Gesture JSON: {"mode":"servo","vx":...,"vy":...,"vz":...,"grip":...}
* Gesture reset JSON: {"mode":"reset"} or transport frame: GESTURE_RESET
* Legacy gesture pose JSON: {"x":...,"y":...,"z":...,"grip":...}

Joint/mode frames may include ``*CRC32`` before the newline. Legacy task
commands without a checksum remain supported.
"""

import json
import math
import queue
import threading
import time
import zlib

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError:
    serial = None


MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
VALID_MODES = {"AUTO", "TELEOP", "GESTURE", "HOLD", "STOW"}
COMMAND_ALIASES = {
    "start": "start",
    "hover": "hover",
    "next": "next",
    "grasp": "grasp",
    "stop": "stop",
    "return": "return",
    "calibrate": "calibrate",
    "arm,start": "start",
    "arm,hover": "hover",
    "arm,next": "next",
    "arm,grasp": "grasp",
    "arm,stop": "stop",
    "arm,return": "return",
    "arm,calibrate": "calibrate",
}


def normalize_command(line):
    if not isinstance(line, str):
        return None
    normalized = "".join(line.strip().split()).lower()
    return COMMAND_ALIASES.get(normalized)


def decode_transport_line(line):
    """Return the body after optional CRC32 validation."""
    body = line.strip()
    if not body:
        raise ValueError("empty line")
    if "*" not in body:
        return body
    body, checksum_text = body.rsplit("*", 1)
    if len(checksum_text) != 8:
        raise ValueError("invalid CRC length")
    expected = zlib.crc32(body.encode("ascii")) & 0xFFFFFFFF
    received = int(checksum_text, 16)
    if received != expected:
        raise ValueError("CRC mismatch")
    return body


def sequence_is_newer(sequence, previous):
    if previous is None:
        return True
    difference = (sequence - previous) & 0xFFFF
    return 0 < difference < 0x8000


class SerialCommandBridge(Node):
    def __init__(self):
        super().__init__("serial_command_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("command_topic", "/task/command")
        self.declare_parameter("mode_topic", "/control/mode")
        self.declare_parameter("teleop_joint_topic", "/arm/teleop_joint_target")
        self.declare_parameter("teleop_timeout_sec", 0.2)
        self.declare_parameter("gesture_timeout_sec", 0.5)
        self.declare_parameter("gesture_servo_deadband", 0.05)
        self.declare_parameter("workspace_x_min", 0.12)
        self.declare_parameter("workspace_x_max", 0.36)
        self.declare_parameter("workspace_y_min", -0.18)
        self.declare_parameter("workspace_y_max", 0.18)
        self.declare_parameter("workspace_z_min", 0.04)
        self.declare_parameter("workspace_z_max", 0.24)
        self.declare_parameter("gesture_smooth_alpha", 0.3)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("ack_enabled", True)
        self.declare_parameter("max_line_bytes", 256)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.teleop_joint_topic = str(
            self.get_parameter("teleop_joint_topic").value
        )
        self.teleop_timeout = max(
            0.05, float(self.get_parameter("teleop_timeout_sec").value)
        )
        self.gesture_timeout = max(
            0.1, float(self.get_parameter("gesture_timeout_sec").value)
        )
        self.gesture_servo_deadband = max(
            0.0,
            min(
                0.5,
                float(self.get_parameter("gesture_servo_deadband").value),
            ),
        )
        self.x_min = float(self.get_parameter("workspace_x_min").value)
        self.x_max = float(self.get_parameter("workspace_x_max").value)
        self.y_min = float(self.get_parameter("workspace_y_min").value)
        self.y_max = float(self.get_parameter("workspace_y_max").value)
        self.z_min = float(self.get_parameter("workspace_z_min").value)
        self.z_max = float(self.get_parameter("workspace_z_max").value)
        self.gesture_smooth_alpha = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter("gesture_smooth_alpha").value),
            ),
        )
        self.reconnect_interval = max(
            0.2, float(self.get_parameter("reconnect_interval_sec").value)
        )
        self.ack_enabled = bool(self.get_parameter("ack_enabled").value)
        self.max_line_bytes = max(
            32, int(self.get_parameter("max_line_bytes").value)
        )

        if serial is None or not hasattr(serial, "Serial"):
            raise RuntimeError(
                "pyserial is not installed; run: sudo apt install python3-serial"
            )
        if not self.port:
            raise ValueError("serial port cannot be empty")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        for axis, lower, upper in (
            ("x", self.x_min, self.x_max),
            ("y", self.y_min, self.y_max),
            ("z", self.z_min, self.z_max),
        ):
            if not lower < upper:
                raise ValueError(f"workspace_{axis}_min must be less than max")

        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)
        self.teleop_joint_pub = self.create_publisher(
            JointState, self.teleop_joint_topic, 1
        )
        self.gesture_pose_pub = self.create_publisher(
            PoseStamped, "/gesture/target_pose", 10
        )
        self.gesture_delta_pub = self.create_publisher(
            Twist, "/gesture/arm_delta", 10
        )
        self.gripper_pub = self.create_publisher(
            Bool, "/gesture/gripper_close", 10
        )
        self.gesture_reset_pub = self.create_publisher(
            Bool, "/gesture/reset", 10
        )

        self._mode = "HOLD"
        self._last_joint_sequence = None
        self._last_joint_time = None
        self._last_gesture_time = None
        self._last_joint_ack_time = 0.0
        self._last_gesture_reject_warning = 0.0
        # GESTURE only: repeated grip heartbeats must not continuously reset
        # the arm command ramp. Reset this cache whenever mode changes.
        self._last_gesture_grip = None
        self._pending_gesture_reset = False
        self._gesture_x = 0.251
        self._gesture_y = 0.0
        self._gesture_z = 0.142
        self._events = queue.Queue(maxsize=100)
        self._stop_event = threading.Event()
        self._serial_lock = threading.Lock()
        self._serial = None
        self._last_connect_warning = 0.0

        self.create_timer(0.05, self._drain_events)
        # Re-publish the mode so a restarted arm node safely resynchronizes.
        self.create_timer(1.0, self._publish_mode)
        self._worker = threading.Thread(
            target=self._serial_loop,
            name="serial-command-reader",
            daemon=True,
        )
        self._worker.start()

        self._publish_mode()
        self.get_logger().info(
            f"Unified serial bridge ready: {self.port} @ {self.baudrate}; "
            f"task={self.command_topic}, mode={self.mode_topic}, "
            f"teleop={self.teleop_joint_topic}, gesture=JSON, "
            f"gesture_reset=/gesture/reset"
        )

    def _put_event(self, kind, value=""):
        try:
            self._events.put_nowait((kind, value))
        except queue.Full:
            try:
                self._events.get_nowait()
            except queue.Empty:
                pass
            try:
                self._events.put_nowait((kind, value))
            except queue.Full:
                pass

    def _get_serial(self):
        with self._serial_lock:
            return self._serial

    def _set_serial(self, port):
        with self._serial_lock:
            self._serial = port

    def _close_serial(self):
        with self._serial_lock:
            port = self._serial
            self._serial = None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass

    def _try_connect(self):
        try:
            port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.2,
                write_timeout=0.5,
            )
            self._set_serial(port)
            self._last_connect_warning = 0.0
            self._put_event("connected")
            return True
        except (serial.SerialException, OSError, ValueError) as exc:
            now = time.monotonic()
            if now - self._last_connect_warning >= 10.0:
                self._last_connect_warning = now
                self._put_event("connect_failed", str(exc))
            return False

    def _serial_loop(self):
        buffer = bytearray()
        while not self._stop_event.is_set() and rclpy.ok():
            port = self._get_serial()
            if port is None or not port.is_open:
                self._close_serial()
                if not self._try_connect():
                    self._stop_event.wait(self.reconnect_interval)
                    continue
                port = self._get_serial()

            try:
                data = port.read(128)
                if not data:
                    continue
                buffer.extend(data)
                if len(buffer) > self.max_line_bytes and b"\n" not in buffer:
                    buffer.clear()
                    self._put_event("line_too_long")
                    continue

                while b"\n" in buffer:
                    raw_line, remainder = buffer.split(b"\n", 1)
                    buffer = bytearray(remainder)
                    if len(raw_line) > self.max_line_bytes:
                        self._put_event("line_too_long")
                        continue
                    line = raw_line.rstrip(b"\r").decode(
                        "ascii", errors="replace"
                    )
                    self._put_event("line", line)
            except (serial.SerialException, OSError) as exc:
                buffer.clear()
                self._close_serial()
                self._put_event("disconnected", str(exc))
                self._stop_event.wait(self.reconnect_interval)
            except Exception as exc:
                buffer.clear()
                self._close_serial()
                self._put_event("disconnected", f"unexpected error: {exc}")
                self._stop_event.wait(self.reconnect_interval)
        self._close_serial()

    def _send_feedback(self, text):
        if not self.ack_enabled:
            return
        port = self._get_serial()
        if port is None or not port.is_open:
            return
        try:
            port.write((text + "\n").encode("ascii"))
            port.flush()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f"serial ACK failed: {exc}")
            self._close_serial()

    @staticmethod
    def _clamp(value, lower, upper):
        return max(float(lower), min(float(upper), float(value)))

    def _publish_gesture_reset(self, reason=""):
        """Reset only the gesture target cache in the arm interface.

        This deliberately does not touch AUTO, TELEOP, task commands, HOME,
        calibration, or the shared motor driver.  The zero Twist is sent with
        the reset so a gesture-aware interface can immediately hold the
        current measured end-effector pose.
        """
        if self._mode != "GESTURE":
            return
        self.gesture_reset_pub.publish(Bool(data=True))
        self.gesture_delta_pub.publish(Twist())
        self._last_gesture_time = time.monotonic()
        if reason:
            self.get_logger().info(f"Gesture target reset: {reason}")

    def _publish_gesture_gripper(self, command):
        if "grip" not in command:
            return
        grip = command["grip"]
        if not isinstance(grip, bool):
            raise ValueError("gesture grip must be true/false")
        if grip == self._last_gesture_grip:
            return
        self._last_gesture_grip = grip
        self.gripper_pub.publish(Bool(data=grip))

    def _handle_gesture_velocity(self, command):
        values = [
            float(command.get("vx", 0.0)),
            float(command.get("vy", 0.0)),
            float(command.get("vz", 0.0)),
        ]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("non-finite gesture velocity")

        values = [self._clamp(value, -1.0, 1.0) for value in values]
        values = [
            0.0 if abs(value) < self.gesture_servo_deadband else value
            for value in values
        ]
        self._publish_gesture_gripper(command)

        # Publish zero velocity as a real gesture command.  In GESTURE mode
        # the arm interface uses the zero Twist to stop target integration and
        # hold the current pose.  This path is isolated from AUTO, whose
        # /servo/arm_delta handling is untouched.
        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = values
        self.gesture_delta_pub.publish(message)

    def _handle_gesture_pose(self, command):
        values = [
            float(command["x"]),
            float(command["y"]),
            float(command["z"]),
        ]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("non-finite gesture position")

        target_x = self._clamp(values[0], self.x_min, self.x_max)
        target_y = self._clamp(values[1], self.y_min, self.y_max)
        target_z = self._clamp(values[2], self.z_min, self.z_max)
        alpha = self.gesture_smooth_alpha
        self._gesture_x = (1.0 - alpha) * self._gesture_x + alpha * target_x
        self._gesture_y = (1.0 - alpha) * self._gesture_y + alpha * target_y
        self._gesture_z = (1.0 - alpha) * self._gesture_z + alpha * target_z
        self._publish_gesture_gripper(command)

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.pose.position.x = self._gesture_x
        message.pose.position.y = self._gesture_y
        message.pose.position.z = self._gesture_z
        message.pose.orientation.w = 1.0
        self.gesture_pose_pub.publish(message)

    def _handle_gesture_json(self, raw):
        command = json.loads(raw)
        if not isinstance(command, dict):
            raise ValueError("gesture JSON must be an object")

        if self._mode != "GESTURE":
            now = time.monotonic()
            if now - self._last_gesture_reject_warning >= 2.0:
                self._last_gesture_reject_warning = now
                self.get_logger().warning(
                    f"Ignoring gesture data while mode is {self._mode}"
                )
            return

        gesture_type = str(command.get("mode", "")).strip().lower()
        reset_requested = gesture_type in {"reset", "center", "recenter"} or (
            command.get("reset") is True
        )
        if reset_requested:
            self._pending_gesture_reset = False
            self._publish_gesture_reset("JSON reset")
            self._send_feedback("OK,GESTURE_RESET")
            return

        if self._pending_gesture_reset:
            self._pending_gesture_reset = False
            self._publish_gesture_reset("enter GESTURE")

        if gesture_type in {"servo", "velocity", "vel"} or any(
            key in command for key in ("vx", "vy", "vz")
        ):
            self._handle_gesture_velocity(command)
        elif all(key in command for key in ("x", "y", "z")):
            self._handle_gesture_pose(command)
        else:
            raise ValueError("unknown gesture JSON format")

        self._last_gesture_time = time.monotonic()

    def _publish_mode(self):
        self.mode_pub.publish(String(data=self._mode))

    def _set_mode(self, mode, stop_autonomy=True):
        mode = mode.upper()
        if mode not in VALID_MODES:
            raise ValueError(f"invalid mode: {mode}")
        if mode == self._mode:
            return

        previous = self._mode
        if stop_autonomy and previous == "AUTO" and mode != "AUTO":
            self.command_pub.publish(String(data="stop"))
        self._mode = mode
        now = time.monotonic()
        self._last_joint_time = now if mode == "TELEOP" else None
        self._last_gesture_time = now if mode == "GESTURE" else None
        self._last_gesture_grip = None
        self._pending_gesture_reset = mode == "GESTURE"
        self._publish_mode()
        self.get_logger().warning(f"Control mode: {previous} -> {mode}")

    def _handle_gesture_reset_frame(self):
        if self._mode != "GESTURE":
            self._send_feedback(f"ERR,MODE,{self._mode}")
            return
        self._pending_gesture_reset = False
        self._publish_gesture_reset("transport frame")
        self._send_feedback("OK,GESTURE_RESET")

    def _handle_task_command(self, command):
        if command in {"start", "hover", "next", "grasp", "calibrate"}:
            self._set_mode("AUTO", stop_autonomy=False)
            self.command_pub.publish(String(data=command))
        elif command == "stop":
            self.command_pub.publish(String(data=command))
            self._set_mode("HOLD", stop_autonomy=False)
        elif command == "return":
            self._set_mode("STOW")
            self.command_pub.publish(String(data=command))
        else:
            self.command_pub.publish(String(data=command))
        self.get_logger().info(f"Task command: {command}")
        self._send_feedback(f"OK,{command}")

    def _handle_mode(self, body):
        parts = [part.strip().upper() for part in body.split(",")]
        if len(parts) != 2 or parts[0] != "MODE":
            raise ValueError("invalid MODE frame")
        self._set_mode(parts[1])
        self._send_feedback(f"OK,MODE,{self._mode}")

    def _handle_joint(self, body):
        parts = body.split(",")
        if len(parts) != 8 or parts[0].upper() != "J":
            raise ValueError("invalid J frame")
        sequence = int(parts[1]) & 0xFFFF
        values = [float(value) for value in parts[2:]]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("non-finite joint value")
        if not sequence_is_newer(sequence, self._last_joint_sequence):
            return

        self._last_joint_sequence = sequence
        self._last_joint_time = time.monotonic()
        if self._mode != "TELEOP":
            self._send_feedback(f"ERR,MODE,{self._mode}")
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(MOTOR_NAMES)
        message.position = [math.radians(value) for value in values[:5]] + [
            values[5]
        ]
        self.teleop_joint_pub.publish(message)

        now = time.monotonic()
        if now - self._last_joint_ack_time >= 1.0:
            self._send_feedback(f"ACK,J,{sequence},TELEOP")
            self._last_joint_ack_time = now

    def _handle_hold(self, body):
        parts = body.split(",")
        if len(parts) != 2 or parts[0].upper() != "H":
            raise ValueError("invalid H frame")
        sequence = int(parts[1]) & 0xFFFF
        if sequence_is_newer(sequence, self._last_joint_sequence):
            self._last_joint_sequence = sequence
        self._set_mode("HOLD")

    def _handle_line(self, line):
        raw = line.strip()
        if not raw:
            return
        try:
            if raw.startswith("{"):
                self._handle_gesture_json(raw)
                return

            body = decode_transport_line(raw)
            upper = body.upper()
            if upper.startswith("J,"):
                self._handle_joint(body)
                return
            if upper.startswith("H,"):
                self._handle_hold(body)
                return
            if upper.startswith("MODE,"):
                self._handle_mode(body)
                return
            if upper in {"GESTURE_RESET", "GESTURE,RESET", "RESET,GESTURE"}:
                self._handle_gesture_reset_frame()
                return

            command = normalize_command(body)
            if command is None:
                raise ValueError("unknown command")
            self._handle_task_command(command)
        except (UnicodeError, ValueError, IndexError) as exc:
            safe_raw = raw.replace(",", "_")[:64]
            self.get_logger().warning(
                f"Invalid telemetry line ({exc}): {safe_raw}"
            )
            self._send_feedback(f"ERR,INVALID,{safe_raw}")

    def _check_teleop_timeout(self):
        if self._mode != "TELEOP" or self._last_joint_time is None:
            return
        age = time.monotonic() - self._last_joint_time
        if age > self.teleop_timeout:
            self.get_logger().error(
                f"Teleop timeout ({age * 1000:.0f} ms); switching to HOLD"
            )
            self._set_mode("HOLD")
            self._send_feedback("ERR,TELEOP_TIMEOUT,HOLD")

    def _check_gesture_timeout(self):
        if self._mode != "GESTURE" or self._last_gesture_time is None:
            return
        age = time.monotonic() - self._last_gesture_time
        if age > self.gesture_timeout:
            self.get_logger().error(
                f"Gesture timeout ({age * 1000:.0f} ms); switching to HOLD"
            )
            self._set_mode("HOLD")
            self._send_feedback("ERR,GESTURE_TIMEOUT,HOLD")

    def _drain_events(self):
        for _ in range(50):
            try:
                kind, value = self._events.get_nowait()
            except queue.Empty:
                break

            if kind == "line":
                self._handle_line(value)
            elif kind == "connected":
                self.get_logger().info(
                    f"Telemetry serial connected: {self.port} @ {self.baudrate}"
                )
                self._publish_mode()
            elif kind == "connect_failed":
                self.get_logger().warning(
                    f"Cannot open {self.port}: {value}; retrying"
                )
            elif kind == "disconnected":
                self.get_logger().warning(
                    f"Telemetry serial disconnected: {value}; retrying"
                )
                if self._mode in {"TELEOP", "GESTURE"}:
                    self._set_mode("HOLD")
            elif kind == "line_too_long":
                self.get_logger().warning(
                    f"Telemetry line exceeded {self.max_line_bytes} bytes"
                )
                self._send_feedback("ERR,LINE_TOO_LONG")
        self._check_teleop_timeout()
        self._check_gesture_timeout()

    def destroy_node(self):
        try:
            self._set_mode("HOLD")
        except Exception:
            pass
        self._stop_event.set()
        self._close_serial()
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialCommandBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"serial command bridge failed: {exc}")
        else:
            print(f"serial command bridge startup failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
