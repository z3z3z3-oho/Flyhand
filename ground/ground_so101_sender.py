#!/usr/bin/env python3
"""Windows unified controller for AUTO, TELEOP, GESTURE and HOLD.

This process is the only owner of the ground telemetry serial port.  It reads
the SO-101 leader for TELEOP, the camera for GESTURE, and sends autonomous task
commands for AUTO.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import threading
import time
import zlib
from pathlib import Path
from urllib.request import urlretrieve


MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
HOME_POSE = (0.0, -90.0, 90.0, 0.0, 0.0, 0.0)
EEG_DOWN_POSE = (0.0, -35.0, 0.0, -75.0, 0.0, 0.0)
JOINT_LIMITS = (
    (-150.0, 150.0),
    (-120.0, 100.0),
    (-120.0, 120.0),
    (-100.0, 100.0),
    (-180.0, 180.0),
    (0.0, 100.0),
)

BUILD_ID = "gesture-j1-fixed-j234-triangle-v15"


def validate_pose(values: tuple[float, ...]) -> None:
    if len(values) != len(MOTOR_NAMES):
        raise ValueError("pose must contain 6 values")
    for name, value, (lower, upper) in zip(MOTOR_NAMES, values, JOINT_LIMITS):
        if value < lower or value > upper:
            raise ValueError(f"{name}={value} outside [{lower}, {upper}]")


def parse_pose(text: str) -> tuple[float, ...]:
    values = tuple(float(part.strip()) for part in text.split(","))
    validate_pose(values)
    return values


def encode_frame(body: str) -> bytes:
    checksum = zlib.crc32(body.encode("ascii")) & 0xFFFFFFFF
    return f"{body}*{checksum:08X}\n".encode("ascii")


def poll_key() -> str | None:
    """Return one pressed console key without blocking."""
    if os.name == "nt":
        import msvcrt

        if msvcrt.kbhit():
            return msvcrt.getwch()
        return None

    import select

    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if readable:
        return sys.stdin.read(1)
    return None


def read_feedback(radio) -> None:
    while radio.in_waiting:
        raw = radio.readline().decode("ascii", errors="replace").strip()
        if raw:
            print(f"\r机载反馈: {raw}{' ' * 20}")


def close_gesture_async(controller) -> None:
    """Close camera/MediaPipe resources without blocking mode switching."""
    if controller is None:
        return

    def _worker() -> None:
        try:
            controller.close()
        except Exception:
            pass

    threading.Thread(
        target=_worker,
        name="so101-gesture-close",
        daemon=True,
    ).start()


class _LegacyHandOffsetGestureController:
    """Lazy camera/MediaPipe controller used only while gesture mode is open."""

    WRIST = 0
    THUMB_TIP = 4
    INDEX_TIP = 8
    MIDDLE_TIP = 12
    RING_TIP = 16
    PINKY_TIP = 20
    INDEX_MCP = 5
    PINKY_MCP = 17
    HAND_CONNECTIONS = (
        (0, 1), (1, 2), (2, 3), (3, 4),
        (0, 5), (5, 6), (6, 7), (7, 8),
        (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16),
        (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
    )
    HAND_MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
        "hand_landmarker/float16/1/hand_landmarker.task"
    )
    POSE_MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_full/float16/1/pose_landmarker_full.task"
    )
    LOCAL_HAND_MODEL = Path(__file__).parent / "hand_landmarker.task"
    LOCAL_POSE_MODEL = Path(__file__).parent / "pose_landmarker_full.task"
    HAND_MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
        "hand_landmarker/float16/1/hand_landmarker.task"
    )
    POSE_MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_full/float16/1/pose_landmarker_full.task"
    )
    LOCAL_HAND_MODEL = Path(__file__).parent / "hand_landmarker.task"
    LOCAL_POSE_MODEL = Path(__file__).parent / "pose_landmarker_full.task"
    DEFAULT_MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
        "hand_landmarker/float16/1/hand_landmarker.task"
    )
    FORCED_MODEL_PATH = Path(
        r"C:\Users\Lenovo\Desktop\SO101_3_modes\ground\hand_landmarker.task"
    )

    def __init__(self, camera_index: int, speed_scale: float, model_path=None):
        try:
            import cv2
            import mediapipe as mp
            import numpy as np
        except ImportError as exc:
            raise RuntimeError(
                "手势模式缺少依赖，请安装: pip install opencv-python mediapipe numpy"
            ) from exc

        self.cv2 = cv2
        self.mp = mp
        self.np = np
        self.speed_scale = max(0.1, min(1.0, float(speed_scale)))
        self.max_screen_x_offset = 0.14
        self.max_screen_y_offset = 0.11
        self.max_hand_size_offset = 0.06
        self.deadzone = 0.10
        self.velocity_filter_alpha = 0.60
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.previous_feature = None
        self.previous_feature_time = None
        self.motion_rate_deadzone = 0.075
        self.motion_size_rate_deadzone = 0.045
        self.motion_xy_gain = 2.8
        self.motion_size_gain = 4.0
        self.stationary_frames = 0
        self.stationary_required_frames = 2
        self.fist_ratio_threshold = 1.5
        self.hand_loss_hold_sec = 0.4
        self.base_wrist_x = None
        self.base_wrist_y = None
        self.base_hand_size = None
        self.last_hand_seen = None

        self._last_timestamp_ms = 0
        if hasattr(mp, "solutions"):
            self.backend = "solutions"
            self.detector = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                model_complexity=0,
                min_detection_confidence=0.7,
                min_tracking_confidence=0.5,
            )
        else:
            self.backend = "tasks"
            from urllib.request import urlretrieve

            # This machine must always load the model from the known ASCII path.
            # Ignore paths supplied by older UI builds so a Chinese install path
            # can never reach MediaPipe's native Windows loader.
            model = self.FORCED_MODEL_PATH
            if not model.exists():
                raise RuntimeError(f"固定手部模型不存在: {model}")

            # MediaPipe's Windows native loader can fail on paths containing
            # Chinese characters or punctuation.  Keep the bundled model as
            # the source, but load it from a stable ASCII-only cache path.
            try:
                str(model.resolve()).encode("ascii")
            except UnicodeEncodeError:
                cache_root = Path(os.environ.get("LOCALAPPDATA", Path.home()))
                cached_model = cache_root / "SO101" / "models" / "hand_landmarker.task"
                cached_model.parent.mkdir(parents=True, exist_ok=True)
                if (
                    not cached_model.exists()
                    or cached_model.stat().st_size != model.stat().st_size
                ):
                    shutil.copy2(model, cached_model)
                model = cached_model
                print(f"MediaPipe 模型使用英文缓存路径: {model}")
            options = mp.tasks.vision.HandLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(model_asset_path=str(model)),
                running_mode=mp.tasks.vision.RunningMode.VIDEO,
                num_hands=1,
                min_hand_detection_confidence=0.7,
                min_hand_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self.detector = mp.tasks.vision.HandLandmarker.create_from_options(
                options
            )
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.detector.close()
            raise RuntimeError(f"无法打开摄像头 {camera_index}")

    def recenter(self) -> None:
        self.base_wrist_x = None
        self.base_wrist_y = None
        self.base_hand_size = None
        self.filtered_velocity = [0.0, 0.0, 0.0]
        print("\n手势中心已重置，请把手放在舒适的中间位置。")

    @staticmethod
    def _distance(a, b) -> float:
        return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))

    def _features(self, landmarks):
        wrist = landmarks[self.WRIST]
        xs = [point.x for point in landmarks]
        ys = [point.y for point in landmarks]
        size = (max(xs) - min(xs) + max(ys) - min(ys)) / 2.0
        return float(wrist.x), float(wrist.y), float(size)

    def _is_fist(self, landmarks) -> bool:
        wrist = landmarks[self.WRIST]
        palm_width = self._distance(
            landmarks[self.INDEX_MCP],
            landmarks[self.PINKY_MCP],
        )
        if palm_width < 1e-6:
            return False
        tips = (
            self.THUMB_TIP,
            self.INDEX_TIP,
            self.MIDDLE_TIP,
            self.RING_TIP,
            self.PINKY_TIP,
        )
        average = sum(
            self._distance(landmarks[index], wrist) for index in tips
        ) / len(tips)
        return average / palm_width < self.fist_ratio_threshold

    def _axis_speed(self, delta: float, maximum: float) -> float:
        raw = max(-1.0, min(1.0, float(delta) / float(maximum)))
        magnitude = abs(raw)
        if magnitude < self.deadzone:
            return 0.0
        magnitude = (magnitude - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(min(1.0, magnitude) * self.speed_scale, raw)

    def _rate_speed(self, rate: float, deadzone: float, gain: float) -> float:
        magnitude = abs(float(rate))
        if magnitude <= deadzone:
            return 0.0
        speed = min(1.0, (magnitude - deadzone) * gain) * self.speed_scale
        return math.copysign(speed, rate)

    def _motion_velocity(self, feature):
        now = time.monotonic()
        if self.previous_feature is None or self.previous_feature_time is None:
            self.previous_feature = feature
            self.previous_feature_time = now
            return [0.0, 0.0, 0.0]

        dt = max(0.03, min(0.20, now - self.previous_feature_time))
        rates = [
            (float(current) - float(previous)) / dt
            for current, previous in zip(feature, self.previous_feature)
        ]
        self.previous_feature = feature
        self.previous_feature_time = now

        raw_velocity = [
            self._rate_speed(
                rates[2],
                self.motion_size_rate_deadzone,
                self.motion_size_gain,
            ),
            self._rate_speed(
                rates[0],
                self.motion_rate_deadzone,
                self.motion_xy_gain,
            ),
            self._rate_speed(
                -rates[1],
                self.motion_rate_deadzone,
                self.motion_xy_gain,
            ),
        ]
        if all(abs(value) < 1e-9 for value in raw_velocity):
            self.stationary_frames += 1
            if self.stationary_frames >= self.stationary_required_frames:
                return [0.0, 0.0, 0.0]
        else:
            self.stationary_frames = 0
        return raw_velocity

    def _detect_landmarks(self, rgb):
        if self.backend == "solutions":
            result = self.detector.process(rgb)
            if not result.multi_hand_landmarks:
                return None
            return list(result.multi_hand_landmarks[0].landmark)

        timestamp_ms = max(
            self._last_timestamp_ms + 1,
            int(time.monotonic() * 1000),
        )
        self._last_timestamp_ms = timestamp_ms
        image = self.mp.Image(
            image_format=self.mp.ImageFormat.SRGB,
            data=rgb,
        )
        result = self.detector.detect_for_video(image, timestamp_ms)
        if not result.hand_landmarks:
            return None
        return list(result.hand_landmarks[0])

    def _draw_landmarks(self, frame, landmarks) -> None:
        height, width = frame.shape[:2]
        points = [
            (int(point.x * width), int(point.y * height))
            for point in landmarks
        ]
        for start, end in self.HAND_CONNECTIONS:
            self.cv2.line(frame, points[start], points[end], (255, 0, 0), 2)
        for point in points:
            self.cv2.circle(frame, point, 3, (0, 255, 0), -1)

    def update(self):
        """Return (gesture command or None, pressed camera key, hand_lost)."""
        ok, frame = self.cap.read()
        if not ok:
            return self._command(self._zero_velocity(), force=True), None, True

        cv2 = self.cv2
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        landmarks = self._detect_landmarks(rgb)
        command = None
        lost = False
        status = "No hand - waiting in HOLD"
        color = (0, 0, 255)

        if landmarks is not None:
            wrist_x, wrist_y, hand_size = self._features(landmarks)
            self.last_hand_seen = time.monotonic()
            if self.base_wrist_x is None:
                self.base_wrist_x = wrist_x
                self.base_wrist_y = wrist_y
                self.base_hand_size = hand_size

            raw_velocity = self._motion_velocity((wrist_x, wrist_y, hand_size))
            if all(abs(value) < 1e-9 for value in raw_velocity):
                self.filtered_velocity = [0.0, 0.0, 0.0]
            else:
                alpha = self.velocity_filter_alpha
                self.filtered_velocity = [
                    (1.0 - alpha) * previous + alpha * current
                    for previous, current in zip(
                        self.filtered_velocity,
                        raw_velocity,
                    )
                ]
            vx, vy, vz = self.filtered_velocity
            grip = self._is_fist(landmarks)
            command = {
                "mode": "servo",
                "vx": vx,
                "vy": vy,
                "vz": vz,
                "grip": grip,
            }
            status = (
                f"GESTURE vx:{vx:+.2f} vy:{vy:+.2f} vz:{vz:+.2f} "
                f"grip:{'CLOSE' if grip else 'OPEN'}"
            )
            color = (0, 255, 0)
            self._draw_landmarks(frame, landmarks)
        elif (
            self.last_hand_seen is not None
            and time.monotonic() - self.last_hand_seen >= self.hand_loss_hold_sec
        ):
            lost = True
            self.last_hand_seen = None
            self.recenter()

        cv2.putText(
            frame,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )
        cv2.putText(
            frame,
            "T:teleop G:gesture A:auto H:hold C:center Q:quit",
            (10, frame.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (220, 220, 220),
            1,
        )
        cv2.imshow("SO101 Unified Ground Controller", frame)
        key_code = cv2.waitKey(1) & 0xFF
        key = chr(key_code) if key_code not in (0xFF, 255) else None
        return command, key, lost

    def close(self) -> None:
        try:
            self.cap.release()
            self.detector.close()
            self.cv2.destroyAllWindows()
        except Exception:
            pass


class GestureController:
    """Shoulder-origin arm follower with reach-ratio depth control.

    Lateral/up-down movement uses wrist position relative to the shoulder,
    normalized by the operator's arm length. Forward/backward movement uses
    shoulder-to-wrist reach divided by upper-arm plus forearm length, so elbow
    flexion and extension produce a clear command without relying on noisy
    monocular depth.
    """

    # MediaPipe hand landmarks.
    HAND_WRIST = 0
    THUMB_CMC = 1
    THUMB_MCP = 2
    THUMB_IP = 3
    THUMB_TIP = 4
    INDEX_MCP = 5
    INDEX_PIP = 6
    INDEX_DIP = 7
    INDEX_TIP = 8
    MIDDLE_MCP = 9
    MIDDLE_PIP = 10
    MIDDLE_DIP = 11
    MIDDLE_TIP = 12
    RING_MCP = 13
    RING_PIP = 14
    RING_DIP = 15
    RING_TIP = 16
    PINKY_MCP = 17
    PINKY_PIP = 18
    PINKY_DIP = 19
    PINKY_TIP = 20

    # MediaPipe pose landmarks.
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12
    LEFT_ELBOW = 13
    RIGHT_ELBOW = 14
    LEFT_WRIST = 15
    RIGHT_WRIST = 16

    HAND_CONNECTIONS = (
        (0, 1), (1, 2), (2, 3), (3, 4),
        (0, 5), (5, 6), (6, 7), (7, 8),
        (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16),
        (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
    )

    HAND_MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
        "hand_landmarker/float16/1/hand_landmarker.task"
    )
    POSE_MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_full/float16/1/pose_landmarker_full.task"
    )

    LOCAL_HAND_MODEL = Path(__file__).parent / "hand_landmarker.task"
    LOCAL_POSE_MODEL = Path(__file__).parent / "pose_landmarker_full.task"

    def __init__(
        self,
        camera_index: int,
        speed_scale: float,
        model_path: str | os.PathLike | None = None,
        pose_model_path: str | os.PathLike | None = None,
        hand_detection_confidence: float = 0.42,
        hand_tracking_confidence: float = 0.38,
        pose_detection_confidence: float = 0.42,
        pose_tracking_confidence: float = 0.38,
        show_window: bool = True,
        jpeg_quality: int = 82,
    ):
        try:
            import cv2
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError(
                "缺少手势模块依赖，请执行: pip install opencv-python mediapipe"
            ) from exc

        from collections import deque

        self.cv2 = cv2
        self.mp = mp
        self.deque = deque
        self.show_window = bool(show_window)
        self.jpeg_quality = max(55, min(95, int(jpeg_quality)))
        self._preview_lock = threading.Lock()
        self._preview_jpeg: bytes | None = None
        self._preview_status = "摄像头尚未启动"
        self._preview_timestamp = 0.0
        self._preview_frame_id = 0

        self.speed_scale = max(0.10, min(1.00, float(speed_scale)))
        self.hand_detection_confidence = self._clamp_confidence(
            hand_detection_confidence
        )
        self.hand_tracking_confidence = self._clamp_confidence(
            hand_tracking_confidence
        )
        self.pose_detection_confidence = self._clamp_confidence(
            pose_detection_confidence
        )
        self.pose_tracking_confidence = self._clamp_confidence(
            pose_tracking_confidence
        )

        # Motion-control gains. Features are shoulder-relative and scale
        # invariant. We differentiate them over a short time window so a still
        # operator produces exactly zero robot velocity instead of a persistent
        # position-error command.
        self.reach_rate_deadzone = 0.055
        self.xy_rate_deadzone = 0.075
        self.reach_rate_gain = 4.2
        self.xy_rate_gain = 2.8
        self.minimum_active_speed = 0.22
        self.maximum_feature_rate = 2.5

        # Filtering and safety. A five-frame median/low-pass removes landmark
        # spikes. Commands are sent at 20 Hz because the aircraft integrates
        # each command into a Cartesian delta; 30 Hz unnecessarily amplifies
        # pose noise and IK workload.
        self.feature_window = deque(maxlen=5)
        self.feature_filter_alpha = 0.34
        self.motion_history = deque(maxlen=7)
        self.motion_window_sec = 0.14
        self.velocity_time_constant = 0.045
        self.max_velocity_slew_per_sec = 1.6
        self.send_interval = 1.0 / 20.0
        self.pose_loss_hold_sec = 0.30
        self.minimum_pose_quality = 0.24
        self.calibration_frames_required = 14
        self.side_switch_confirm_frames = 8
        self.stationary_confirm_frames = 2

        # GESTURE zone controller.  Use hand-palm screen position instead of
        # pose-derived shoulder/wrist velocity for the command path.  This
        # prevents pose-estimation drift from becoming a persistent base
        # rotation.  Coordinates are filtered before zone decisions.
        self.palm_point_window = deque(maxlen=5)
        self.palm_point_filter_alpha = 0.25
        self.palm_point_filtered: list[float] | None = None
        self.horizontal_zone_state = "center"
        self.horizontal_zone_candidate = "center"
        self.horizontal_zone_count = 0
        self.vertical_zone_state = "center"
        self.vertical_zone_candidate = "center"
        self.vertical_zone_count = 0
        self.zone_enter_confirm_frames = 3
        self.zone_center_confirm_frames = 2
        self.last_zone_speed = [0.0, 0.0]

        # Absolute target mapping for GESTURE.  The operator shoulder is the
        # robot base reference; elbow extension maps to X, wrist lateral
        # offset maps to Y, and wrist vertical offset maps to Z.  This sends a
        # target pose instead of velocity, so a still operator produces a
        # stable target rather than continuous motion.
        self.target_x_min = 0.15
        self.target_x_max = 0.33
        self.target_y_span = 0.16
        self.target_z_min = 0.065
        self.target_z_max = 0.235
        self.target_z_center = 0.14
        self.target_lateral_full_scale = 0.42
        self.target_vertical_full_scale = 0.32
        self.target_y_sign = 1.0
        self.target_up_x = 0.18
        self.target_up_x_blend = 0.82
        self.target_extension_min = 0.0
        self.target_extension_full = 1.0
        self.target_safe_center = [0.251, 0.0, 0.142]
        self.target_safe_radius = [0.115, 0.155, 0.105]
        self.target_min_horizontal_radius = 0.130
        self.target_filter_alpha = 0.18
        self.target_window = deque(maxlen=10)
        self.target_filtered: list[float] | None = None
        self.max_target_step_m = 0.010

        # GESTURE joint mode. This path maps image-plane arm angles to relative
        # joint offsets: shoulder->elbow angle drives joint 1, shoulder->wrist
        # angle drives joint 3, and hand open/close drives the gripper. Joints
        # 2 and 4 are intentionally left locked by the airborne interface.
        self.joint_pan_span_deg = 90.0
        self.joint_pan_sign = 1.0
        self.joint_pan_center_x = 0.5
        self.joint_pan_full_span_norm = 0.38
        # Joint 1 is movement-driven, not screen-position-driven.  The palm
        # point is already median/low-pass filtered; this second, short motion
        # filter stabilizes dx before integrating it into a session offset.
        self.joint_pan_motion_gain_deg = 180.0
        self.joint_pan_motion_deadband_norm = 0.0015
        self.joint_pan_motion_filter_alpha = 0.30
        self.joint_pan_motion_max_step_deg = 1.2
        self.joint_pan_stationary_span_norm = 0.006
        self.joint_pan_offset_limit_deg = 180.0
        self.joint_pan_motion_window = deque(maxlen=5)
        self.joint_pan_position_window = deque(maxlen=5)
        self.joint_pan_last_x: float | None = None
        self.joint_pan_filtered_dx = 0.0
        self.joint_pan_offset_deg = 0.0
        self.robot_upper_arm_m = 0.116
        self.robot_lower_arm_m = 0.135
        self.joint_shoulder_min_deg = -100.0
        self.joint_shoulder_max_deg = 80.0
        self.joint_elbow_min_deg = 5.0
        self.joint_elbow_max_deg = 96.0
        self.joint_wrist_comp_min_deg = -95.0
        self.joint_wrist_comp_max_deg = 95.0
        self.joint_screen_deadband = 0.035
        self.joint_vertical_full_scale = 0.70
        self.joint_filter_alpha = 0.12
        self.joint_max_step_deg = 1.6
        self.joint_window = deque(maxlen=10)
        self.joint_filtered: list[float] | None = None
        self.joint_angle_window = deque(maxlen=7)
        self.joint_angle_filter_alpha = 0.14
        self.joint_angle_filtered: list[float] | None = None
        self.joint_angle_baseline: list[float] | None = None
        self.joint_pan_angle_gain = 1.0
        self.joint_pan_angle_limit_deg = 85.0
        self.joint_elbow_angle_gain = 1.15
        self.joint_elbow_angle_limit_deg = 98.0
        self.joint_angle_max_step_deg = 0.75
        self.joint_angle_deadband_deg = 0.35
        self.joint_angle_output: list[float] | None = None

        self.feature_filtered: list[float] | None = None
        self.baseline_feature: list[float] | None = None
        self.calibration_samples: list[list[float]] = []
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.last_velocity_time = time.monotonic()
        self.last_send_time = 0.0
        self.last_pose_seen: float | None = None
        self.selected_side: str | None = None
        self.pending_side: str | None = None
        self.pending_side_count = 0
        self.stationary_count = 0
        self.last_motion_rates = [0.0, 0.0, 0.0]

        # Gripper state. grip_value=0 means fully open; 1 means fully closed.
        self.last_grip = False
        self.grip_value = 0.0
        self.grip_candidate = False
        self.grip_candidate_count = 0
        self.grip_close_confirm_frames = 4
        self.grip_open_confirm_frames = 7
        self.grip_close_score = 0.66
        self.grip_open_score = 0.20
        self.grip_recent_observations = deque(maxlen=6)
        self.grip_changed = False
        self.last_hand_seen: float | None = None

        self._last_timestamp_ms = 0

        if not hasattr(mp, "tasks") or not hasattr(mp.tasks, "vision"):
            self.backend = "solutions"
            self.hand_detector = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                model_complexity=1,
                min_detection_confidence=self.hand_detection_confidence,
                min_tracking_confidence=self.hand_tracking_confidence,
            )
            self.pose_detector = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=2,
                smooth_landmarks=True,
                enable_segmentation=False,
                min_detection_confidence=self.pose_detection_confidence,
                min_tracking_confidence=self.pose_tracking_confidence,
            )
        else:
            self.backend = "tasks"
            hand_model = self._prepare_model(
                supplied=model_path,
                local_default=self.LOCAL_HAND_MODEL,
                cache_name="hand_landmarker.task",
                download_url=self.HAND_MODEL_URL,
                display_name="手部模型",
            )
            pose_model = self._prepare_model(
                supplied=pose_model_path,
                local_default=self.LOCAL_POSE_MODEL,
                cache_name="pose_landmarker_full.task",
                download_url=self.POSE_MODEL_URL,
                display_name="姿态模型",
            )

            hand_options = mp.tasks.vision.HandLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(
                    model_asset_path=str(hand_model)
                ),
                running_mode=mp.tasks.vision.RunningMode.VIDEO,
                num_hands=1,
                min_hand_detection_confidence=self.hand_detection_confidence,
                min_hand_presence_confidence=self.hand_tracking_confidence,
                min_tracking_confidence=self.hand_tracking_confidence,
            )
            pose_options = mp.tasks.vision.PoseLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(
                    model_asset_path=str(pose_model)
                ),
                running_mode=mp.tasks.vision.RunningMode.VIDEO,
                num_poses=1,
                min_pose_detection_confidence=self.pose_detection_confidence,
                min_pose_presence_confidence=self.pose_tracking_confidence,
                min_tracking_confidence=self.pose_tracking_confidence,
                output_segmentation_masks=False,
            )
            self.hand_detector = (
                mp.tasks.vision.HandLandmarker.create_from_options(hand_options)
            )
            self.pose_detector = (
                mp.tasks.vision.PoseLandmarker.create_from_options(pose_options)
            )

        backend = cv2.CAP_DSHOW if os.name == "nt" else 0
        self.cap = cv2.VideoCapture(camera_index, backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.close()
            raise RuntimeError(f"无法打开摄像头: {camera_index}")

        print("\n手势模式：肩关节为原点，肩-肘-腕三点跟踪")
        print("肩-肘-腕夹角控制前后；手腕相对肩膀的左右/上下控制 Y/Z")
        print("保持肩膀、手肘、手腕和手掌同时出现在画面中")

    @staticmethod
    def _clamp_confidence(value: float) -> float:
        return max(0.10, min(0.95, float(value)))

    @staticmethod
    def _cache_root() -> Path:
        root = Path(os.environ.get("LOCALAPPDATA", Path.home()))
        cache = root / "SO101" / "models"
        cache.mkdir(parents=True, exist_ok=True)
        return cache

    def _prepare_model(
        self,
        supplied,
        local_default: Path,
        cache_name: str,
        download_url: str,
        display_name: str,
    ) -> Path:
        candidates = []
        if supplied:
            candidates.append(Path(supplied).expanduser())
        candidates.append(local_default)
        cached = self._cache_root() / cache_name
        candidates.append(cached)

        source = next((path for path in candidates if path.exists()), None)
        if source is None:
            print(f"正在下载{display_name}到: {cached}")
            try:
                urlretrieve(download_url, cached)
            except Exception as exc:
                raise RuntimeError(
                    f"缺少{display_name}，自动下载失败。请把模型保存为: "
                    f"{local_default}。原始错误: {exc}"
                ) from exc
            source = cached

        try:
            str(source.resolve()).encode("ascii")
            return source.resolve()
        except UnicodeEncodeError:
            if not cached.exists() or cached.stat().st_size != source.stat().st_size:
                shutil.copy2(source, cached)
            return cached.resolve()

    def recenter(self, reset_side: bool = True) -> None:
        self.feature_window.clear()
        self.motion_history.clear()
        self.feature_filtered = None
        self.baseline_feature = None
        self.calibration_samples.clear()
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.last_motion_rates = [0.0, 0.0, 0.0]
        self.stationary_count = 0
        self.palm_point_window.clear()
        self.palm_point_filtered = None
        self.horizontal_zone_state = "center"
        self.horizontal_zone_candidate = "center"
        self.horizontal_zone_count = 0
        self.vertical_zone_state = "center"
        self.vertical_zone_candidate = "center"
        self.vertical_zone_count = 0
        self.last_zone_speed = [0.0, 0.0]
        self.target_window.clear()
        self.target_filtered = None
        self.joint_window.clear()
        self.joint_filtered = None
        self.joint_pan_center_x = 0.5
        self.joint_pan_motion_window.clear()
        self.joint_pan_position_window.clear()
        self.joint_pan_last_x = None
        self.joint_pan_filtered_dx = 0.0
        self.joint_angle_window.clear()
        self.joint_angle_filtered = None
        self.joint_angle_baseline = None
        self.joint_angle_output = None
        # A full recenter starts a new relative-motion session.  An internal
        # left/right arm-track switch keeps the accumulated robot offset, but
        # still drops the old image sample so the switch cannot create a jump.
        if reset_side:
            self.joint_pan_offset_deg = 0.0
        self.grip_recent_observations.clear()
        self.grip_changed = False
        if reset_side:
            self.selected_side = None
            self.pending_side = None
            self.pending_side_count = 0
        self.last_velocity_time = time.monotonic()
        print("\n正在重新定中：保持肩、肘、腕稳定约 0.7 秒")

    @staticmethod
    def _distance_2d(a, b) -> float:
        return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))

    @staticmethod
    def _distance_3d(a, b) -> float:
        return math.sqrt(
            (float(a.x) - float(b.x)) ** 2
            + (float(a.y) - float(b.y)) ** 2
            + (float(a.z) - float(b.z)) ** 2
        )

    @staticmethod
    def _quality(point) -> float:
        visibility = float(getattr(point, "visibility", 1.0))
        presence = float(getattr(point, "presence", 1.0))
        return min(visibility, presence)

    @staticmethod
    def _median(values: list[float]) -> float:
        ordered = sorted(values)
        count = len(ordered)
        middle = count // 2
        if count % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])

    @staticmethod
    def _angle(a, b, c) -> float:
        """Return angle ABC in degrees using image coordinates."""
        bax = float(a.x) - float(b.x)
        bay = float(a.y) - float(b.y)
        bcx = float(c.x) - float(b.x)
        bcy = float(c.y) - float(b.y)
        norm1 = math.hypot(bax, bay)
        norm2 = math.hypot(bcx, bcy)
        if norm1 < 1e-6 or norm2 < 1e-6:
            return 0.0
        cosine = max(-1.0, min(1.0, (bax * bcx + bay * bcy) / (norm1 * norm2)))
        return math.degrees(math.acos(cosine))

    @staticmethod
    def _vector_angle_deg(a, b) -> float:
        """Return image-plane vector angle; positive y points upward."""
        return math.degrees(
            math.atan2(-(float(b.y) - float(a.y)), float(b.x) - float(a.x))
        )

    @staticmethod
    def _angle_delta_deg(current: float, baseline: float) -> float:
        """Return shortest signed angle delta in degrees."""
        return (float(current) - float(baseline) + 180.0) % 360.0 - 180.0

    def _hand_close_score(self, landmarks) -> float | None:
        """Return 0=open, 1=closed from all five fingers."""
        if landmarks is None or len(landmarks) < 21:
            return None

        palm = max(
            1e-5,
            0.5
            * (
                self._distance_2d(
                    landmarks[self.HAND_WRIST], landmarks[self.MIDDLE_MCP]
                )
                + self._distance_2d(
                    landmarks[self.INDEX_MCP], landmarks[self.PINKY_MCP]
                )
            ),
        )

        fingers = (
            (self.INDEX_MCP, self.INDEX_PIP, self.INDEX_DIP, self.INDEX_TIP),
            (self.MIDDLE_MCP, self.MIDDLE_PIP, self.MIDDLE_DIP, self.MIDDLE_TIP),
            (self.RING_MCP, self.RING_PIP, self.RING_DIP, self.RING_TIP),
            (self.PINKY_MCP, self.PINKY_PIP, self.PINKY_DIP, self.PINKY_TIP),
        )

        curl_scores = []
        for mcp, pip, dip, tip in fingers:
            pip_angle = self._angle(
                landmarks[mcp], landmarks[pip], landmarks[dip]
            )
            dip_angle = self._angle(
                landmarks[pip], landmarks[dip], landmarks[tip]
            )
            angle_open = max(0.0, min(1.0, (0.5 * (pip_angle + dip_angle) - 70.0) / 90.0))

            tip_distance = self._distance_2d(
                landmarks[tip], landmarks[self.HAND_WRIST]
            ) / palm
            distance_open = max(0.0, min(1.0, (tip_distance - 1.15) / 1.10))
            open_score = 0.60 * angle_open + 0.40 * distance_open
            curl_scores.append(1.0 - open_score)

        thumb_distance = self._distance_2d(
            landmarks[self.THUMB_TIP], landmarks[self.PINKY_MCP]
        ) / palm
        thumb_open = max(0.0, min(1.0, (thumb_distance - 0.85) / 1.00))
        thumb_curl = 1.0 - thumb_open

        close_score = 0.88 * (sum(curl_scores) / len(curl_scores)) + 0.12 * thumb_curl
        return max(0.0, min(1.0, close_score))

    def _update_grip(self, landmarks) -> None:
        score = self._hand_close_score(landmarks)
        if score is None:
            return
        self.last_hand_seen = time.monotonic()

        # Light filtering keeps continuous grip_value usable while binary state
        # still switches quickly through hysteresis.
        self.grip_value += 0.72 * (score - self.grip_value)

        if self.grip_value >= self.grip_close_score:
            raw_state = True
            confirm = self.grip_close_confirm_frames
        elif self.grip_value <= self.grip_open_score:
            raw_state = False
            confirm = self.grip_open_confirm_frames
        else:
            # Low-confidence middle band: keep the previous gripper command.
            self.grip_candidate_count = 0
            return

        self.grip_recent_observations.append(raw_state)
        transitions = sum(
            1
            for old, new in zip(
                self.grip_recent_observations,
                list(self.grip_recent_observations)[1:],
            )
            if old != new
        )
        if transitions >= 3:
            # Continuous jumping means the hand shape is not reliable enough.
            self.grip_candidate_count = 0
            return

        if raw_state != self.grip_candidate:
            self.grip_candidate = raw_state
            self.grip_candidate_count = 1
        else:
            self.grip_candidate_count += 1

        if self.grip_candidate_count >= confirm and self.last_grip != self.grip_candidate:
            self.last_grip = self.grip_candidate
            self.grip_changed = True

    def _detect(self, rgb):
        if self.backend == "solutions":
            hand_result = self.hand_detector.process(rgb)
            pose_result = self.pose_detector.process(rgb)
            hand = None
            pose_image = None
            pose_world = None
            if hand_result.multi_hand_landmarks:
                hand = list(hand_result.multi_hand_landmarks[0].landmark)
            if pose_result.pose_landmarks:
                pose_image = list(pose_result.pose_landmarks.landmark)
            if getattr(pose_result, "pose_world_landmarks", None):
                pose_world = list(pose_result.pose_world_landmarks.landmark)
            return hand, pose_image, pose_world

        timestamp_ms = max(
            self._last_timestamp_ms + 1,
            int(time.monotonic() * 1000),
        )
        self._last_timestamp_ms = timestamp_ms
        image = self.mp.Image(
            image_format=self.mp.ImageFormat.SRGB,
            data=rgb,
        )
        hand_result = self.hand_detector.detect_for_video(image, timestamp_ms)
        pose_result = self.pose_detector.detect_for_video(image, timestamp_ms)
        hand = list(hand_result.hand_landmarks[0]) if hand_result.hand_landmarks else None
        pose_image = (
            list(pose_result.pose_landmarks[0])
            if pose_result.pose_landmarks
            else None
        )
        pose_world = (
            list(pose_result.pose_world_landmarks[0])
            if getattr(pose_result, "pose_world_landmarks", None)
            else None
        )
        return hand, pose_image, pose_world

    def _candidate_feature(
        self,
        side: str,
        hand,
        pose_image,
        pose_world,
    ):
        """Build a shoulder-origin arm feature.

        X uses elbow extension, combining elbow angle and shoulder-to-wrist
        reach. This is much more responsive than monocular Z and explicitly
        requires the shoulder landmark. Y/Z use wrist displacement relative to
        the selected shoulder, normalized by total arm length.
        """
        if side == "left":
            indices = (self.LEFT_SHOULDER, self.LEFT_ELBOW, self.LEFT_WRIST)
        else:
            indices = (self.RIGHT_SHOULDER, self.RIGHT_ELBOW, self.RIGHT_WRIST)

        shoulder_i, elbow_i, wrist_i = indices
        shoulder = pose_image[shoulder_i]
        elbow = pose_image[elbow_i]
        wrist = pose_image[wrist_i]

        quality = min(
            self._quality(shoulder),
            self._quality(elbow),
            self._quality(wrist),
        )
        if quality < self.minimum_pose_quality:
            return None

        # The selected shoulder must actually be visible inside the image.
        if not (-0.04 <= float(shoulder.x) <= 1.04 and -0.04 <= float(shoulder.y) <= 1.04):
            return None

        upper_2d = self._distance_2d(shoulder, elbow)
        lower_2d = self._distance_2d(elbow, wrist)
        arm_2d = upper_2d + lower_2d
        if upper_2d < 0.035 or lower_2d < 0.035 or arm_2d < 0.10:
            return None

        # Elbow angle: shoulder-elbow-wrist. Bent arm is small, straight arm is
        # close to 180 degrees. This provides a strong extension signal.
        elbow_angle = self._angle(shoulder, elbow, wrist)
        angle_extension = max(0.0, min(1.0, (elbow_angle - 45.0) / 125.0))

        # Scale-invariant reach ratio. This reinforces the angle measurement and
        # remains useful when part of the arm rotates toward the camera.
        reach_ratio = self._distance_2d(shoulder, wrist) / arm_2d
        reach_ratio = max(0.20, min(1.0, reach_ratio))
        ratio_extension = max(0.0, min(1.0, (reach_ratio - 0.38) / 0.60))

        extension = 0.68 * angle_extension + 0.32 * ratio_extension
        hand_wrist = hand[self.HAND_WRIST] if hand is not None else None
        palm = self._palm_center(hand)
        control_x = palm[0] if palm is not None else float(wrist.x)
        control_y = palm[1] if palm is not None else float(wrist.y)
        lateral = (control_x - float(shoulder.x)) / arm_2d
        vertical = (control_y - float(shoulder.y)) / arm_2d

        hand_distance = (
            self._distance_2d(hand_wrist, wrist)
            if hand_wrist is not None
            else 0.25
        )
        score = hand_distance - 0.16 * quality
        if side == self.selected_side:
            score -= 0.12

        return {
            "side": side,
            "score": score,
            "quality": quality,
            "shoulder": shoulder,
            "elbow": elbow,
            "wrist": wrist,
            "feature": [extension, lateral, vertical],
            "control_point": [control_x, control_y],
            "reach_source": f"ANGLE:{elbow_angle:.0f}",
            "elbow_angle": elbow_angle,
            "reach_ratio": reach_ratio,
            "upper_2d": upper_2d,
            "lower_2d": lower_2d,
        }

    def _select_arm(self, hand, pose_image, pose_world):
        if pose_image is None or len(pose_image) <= self.RIGHT_WRIST:
            return None

        candidates = []
        for side in ("left", "right"):
            candidate = self._candidate_feature(
                side, hand, pose_image, pose_world
            )
            if candidate is not None:
                candidates.append(candidate)
        if not candidates:
            return None

        best = min(candidates, key=lambda item: item["score"])
        best_side = best["side"]

        if self.selected_side is None:
            self.selected_side = best_side
        elif best_side != self.selected_side:
            if self.pending_side == best_side:
                self.pending_side_count += 1
            else:
                self.pending_side = best_side
                self.pending_side_count = 1
            if self.pending_side_count < self.side_switch_confirm_frames:
                same_side = next(
                    (item for item in candidates if item["side"] == self.selected_side),
                    None,
                )
                if same_side is not None:
                    best = same_side
            else:
                self.selected_side = best_side
                self.pending_side = None
                self.pending_side_count = 0
                self.recenter(reset_side=False)
        else:
            self.pending_side = None
            self.pending_side_count = 0

        return best

    def _filter_feature(self, feature: list[float]) -> list[float]:
        self.feature_window.append(list(feature))
        median_feature = [
            self._median([sample[axis] for sample in self.feature_window])
            for axis in range(3)
        ]
        if self.feature_filtered is None:
            self.feature_filtered = list(median_feature)
            return list(median_feature)
        alpha = self.feature_filter_alpha
        self.feature_filtered = [
            old + alpha * (new - old)
            for old, new in zip(self.feature_filtered, median_feature)
        ]
        return list(self.feature_filtered)

    def _palm_center(self, hand) -> list[float] | None:
        """Return stable palm center from wrist and MCP points."""
        if hand is None or len(hand) <= self.PINKY_MCP:
            return None
        points = (
            hand[self.HAND_WRIST],
            hand[self.INDEX_MCP],
            hand[self.MIDDLE_MCP],
            hand[self.PINKY_MCP],
        )
        return [
            sum(float(point.x) for point in points) / len(points),
            sum(float(point.y) for point in points) / len(points),
        ]

    def _filter_palm_point(self, point: list[float]) -> list[float]:
        """Median filter followed by low-pass filtering before zone logic."""
        self.palm_point_window.append(list(point))
        median_point = [
            self._median([sample[axis] for sample in self.palm_point_window])
            for axis in range(2)
        ]
        if self.palm_point_filtered is None:
            self.palm_point_filtered = list(median_point)
            return list(median_point)
        alpha = self.palm_point_filter_alpha
        self.palm_point_filtered = [
            old + alpha * (new - old)
            for old, new in zip(self.palm_point_filtered, median_point)
        ]
        return list(self.palm_point_filtered)

    @staticmethod
    def _horizontal_zone_speed(x: float) -> tuple[str, float]:
        x = max(0.0, min(1.0, float(x)))
        if x <= 0.30:
            return "left", -1.0
        if x < 0.45:
            return "left", -((0.45 - x) / 0.15)
        if x <= 0.55:
            return "center", 0.0
        if x < 0.70:
            return "right", (x - 0.55) / 0.15
        return "right", 1.0

    @staticmethod
    def _vertical_zone_speed(y: float) -> tuple[str, float]:
        y = max(0.0, min(1.0, float(y)))
        if y <= 0.30:
            return "up", 1.0
        if y < 0.45:
            return "up", (0.45 - y) / 0.15
        if y <= 0.55:
            return "center", 0.0
        if y < 0.70:
            return "down", -((y - 0.55) / 0.15)
        return "down", -1.0

    def _debounced_zone(
        self,
        axis: str,
        raw_zone: str,
    ) -> str:
        """Require stable zone observations before changing state."""
        if axis == "horizontal":
            state = self.horizontal_zone_state
            candidate = self.horizontal_zone_candidate
            count = self.horizontal_zone_count
        else:
            state = self.vertical_zone_state
            candidate = self.vertical_zone_candidate
            count = self.vertical_zone_count

        if raw_zone == state:
            candidate = raw_zone
            count = 0
        else:
            if raw_zone == candidate:
                count += 1
            else:
                candidate = raw_zone
                count = 1
            required = (
                self.zone_center_confirm_frames
                if raw_zone == "center"
                else self.zone_enter_confirm_frames
            )
            if count >= required:
                state = raw_zone
                count = 0

        if axis == "horizontal":
            self.horizontal_zone_state = state
            self.horizontal_zone_candidate = candidate
            self.horizontal_zone_count = count
        else:
            self.vertical_zone_state = state
            self.vertical_zone_candidate = candidate
            self.vertical_zone_count = count
        return state

    def _screen_zone_target(self, hand) -> tuple[list[float], dict] | None:
        """Map filtered palm screen position to gesture velocity.

        Horizontal hand position controls the robot lateral/base direction
        through vy.  Vertical hand position controls end-effector height
        through vz.  vx is held at zero to avoid unintended forward drift.
        """
        center = self._palm_center(hand)
        if center is None:
            return None
        filtered = self._filter_palm_point(center)

        raw_h_zone, raw_h_speed = self._horizontal_zone_speed(filtered[0])
        raw_v_zone, raw_v_speed = self._vertical_zone_speed(filtered[1])
        h_zone = self._debounced_zone("horizontal", raw_h_zone)
        v_zone = self._debounced_zone("vertical", raw_v_zone)

        if h_zone == "center":
            h_speed = 0.0
        elif raw_h_zone == h_zone:
            h_speed = raw_h_speed
        else:
            h_speed = self.last_zone_speed[0]

        if v_zone == "center":
            v_speed = 0.0
        elif raw_v_zone == v_zone:
            v_speed = raw_v_speed
        else:
            v_speed = self.last_zone_speed[1]

        h_speed = max(-1.0, min(1.0, h_speed)) * self.speed_scale
        v_speed = max(-1.0, min(1.0, v_speed)) * self.speed_scale
        self.last_zone_speed = [h_speed, v_speed]

        return [0.0, h_speed, v_speed], {
            "palm_x": filtered[0],
            "palm_y": filtered[1],
            "raw_h_zone": raw_h_zone,
            "raw_v_zone": raw_v_zone,
            "h_zone": h_zone,
            "v_zone": v_zone,
        }

    def _feature_to_target_pose(
        self,
        selected: dict,
        feature: list[float],
    ) -> tuple[list[float], dict]:
        """Map shoulder-origin human arm posture to robot end-effector XYZ."""
        raw_extension = max(0.0, min(1.0, float(feature[0])))
        extension = (
            (raw_extension - self.target_extension_min)
            / max(1e-6, self.target_extension_full - self.target_extension_min)
        )
        extension = max(0.0, min(1.0, extension))
        lateral = max(
            -1.0,
            min(1.0, float(feature[1]) / self.target_lateral_full_scale),
        )
        vertical = max(
            -1.0,
            min(1.0, float(feature[2]) / self.target_vertical_full_scale),
        )

        target_x = self.target_x_min + extension * (
            self.target_x_max - self.target_x_min
        )
        up_amount = max(0.0, min(1.0, -vertical))
        if up_amount > 0.0:
            blend = self.target_up_x_blend * up_amount
            target_x = (1.0 - blend) * target_x + blend * self.target_up_x
        target_y = lateral * self.target_y_span * self.target_y_sign
        target_z = self.target_z_center - vertical * (
            0.5 * (self.target_z_max - self.target_z_min)
        )
        target_z = max(self.target_z_min, min(self.target_z_max, target_z))
        raw_target = [target_x, target_y, target_z]
        safe_target, workspace_meta = self._project_target_to_safe_workspace(
            raw_target
        )

        return safe_target, {
            "extension": extension,
            "raw_extension": raw_extension,
            "lateral": lateral,
            "vertical": vertical,
            "up_amount": up_amount,
            "elbow_angle": float(selected.get("elbow_angle", 0.0)),
            "reach_ratio": float(selected.get("reach_ratio", 0.0)),
            **workspace_meta,
        }

    def _project_target_to_safe_workspace(
        self,
        target: list[float],
    ) -> tuple[list[float], dict]:
        """Keep GESTURE XYZ inside a conservative reachable shell.

        The bridge and arm interface still keep their own mode-local safety
        checks. This ground-side projection prevents gesture mapping from
        repeatedly sending box-corner or near-base targets that are likely to
        make IK reject the whole command.
        """
        x = max(self.target_x_min, min(self.target_x_max, float(target[0])))
        y = max(-self.target_y_span, min(self.target_y_span, float(target[1])))
        z = max(self.target_z_min, min(self.target_z_max, float(target[2])))

        projected = False
        horizontal = math.hypot(x, y)
        if horizontal < self.target_min_horizontal_radius:
            projected = True
            if horizontal < 1e-6:
                x = self.target_min_horizontal_radius
                y = 0.0
            else:
                scale = self.target_min_horizontal_radius / horizontal
                x *= scale
                y *= scale

        cx, cy, cz = self.target_safe_center
        rx, ry, rz = self.target_safe_radius
        dx = (x - cx) / max(rx, 1e-6)
        dy = (y - cy) / max(ry, 1e-6)
        dz = (z - cz) / max(rz, 1e-6)
        ellipsoid = math.sqrt(dx * dx + dy * dy + dz * dz)
        if ellipsoid > 1.0:
            projected = True
            x = cx + (x - cx) / ellipsoid
            y = cy + (y - cy) / ellipsoid
            z = cz + (z - cz) / ellipsoid

        x = max(self.target_x_min, min(self.target_x_max, x))
        y = max(-self.target_y_span, min(self.target_y_span, y))
        z = max(self.target_z_min, min(self.target_z_max, z))
        return [x, y, z], {
            "workspace_projected": projected,
            "workspace_radius": ellipsoid,
        }

    def _filter_target_pose(self, target: list[float]) -> list[float]:
        """Filter absolute target coordinates and limit frame-to-frame jumps."""
        self.target_window.append(list(target))
        averaged_target = [
            sum(sample[axis] for sample in self.target_window)
            / len(self.target_window)
            for axis in range(3)
        ]
        if self.target_filtered is None:
            self.target_filtered = list(averaged_target)
            return list(averaged_target)

        alpha = self.target_filter_alpha
        updated = []
        for previous, desired in zip(self.target_filtered, averaged_target):
            smoothed = previous + alpha * (desired - previous)
            change = max(
                -self.max_target_step_m,
                min(self.max_target_step_m, smoothed - previous),
            )
            updated.append(previous + change)
        self.target_filtered = updated
        return list(updated)

    def _target_command(self, target: list[float], force: bool = False):
        now = time.monotonic()
        if not force and now - self.last_send_time < self.send_interval:
            return None
        self.last_send_time = now
        grip_closed = bool(self.last_grip)
        self.grip_changed = False
        return {
            "mode": "target",
            "x": float(target[0]),
            "y": float(target[1]),
            "z": float(target[2]),
            "grip": grip_closed,
            "gripper": grip_closed,
            "gripper_close": grip_closed,
            "grip_value": float(max(0.0, min(1.0, self.grip_value))),
        }

    def _screen_to_joint_target(self, palm: list[float]) -> tuple[list[float], dict]:
        """Map filtered palm screen position to absolute gesture joint angles."""
        x = max(0.0, min(1.0, float(palm[0])))
        y = max(0.0, min(1.0, float(palm[1])))

        x_offset = x - 0.5
        if abs(x_offset) <= self.joint_screen_deadband:
            x_offset = 0.0
        pan = x_offset * 2.0 * self.joint_pan_span_deg * self.joint_pan_sign

        elbow = self.joint_elbow_top_deg + y * (
            self.joint_elbow_bottom_deg - self.joint_elbow_top_deg
        )
        raw = [pan, elbow]

        if self.joint_filtered is None:
            self.joint_filtered = list(raw)
        else:
            updated = []
            for previous, desired in zip(self.joint_filtered, raw):
                smoothed = previous + self.joint_filter_alpha * (desired - previous)
                delta = max(
                    -self.joint_max_step_deg,
                    min(self.joint_max_step_deg, smoothed - previous),
                )
                updated.append(previous + delta)
            self.joint_filtered = updated

        return list(self.joint_filtered), {
            "palm_x": x,
            "palm_y": y,
            "raw_pan": pan,
            "raw_elbow": elbow,
        }

    def _update_pan_motion_offset(self, palm_x: float) -> tuple[float, dict]:
        """Integrate filtered horizontal hand movement into a pan offset.

        The returned value is the cumulative offset for the current GESTURE
        session.  Re-sending it is safe: the aircraft compares consecutive
        offsets and applies only their difference.
        """
        x = max(0.0, min(1.0, float(palm_x)))
        self.joint_pan_position_window.append(x)

        if self.joint_pan_last_x is None:
            self.joint_pan_last_x = x
            self.joint_pan_center_x = x
            self.joint_pan_motion_window.clear()
            self.joint_pan_filtered_dx = 0.0
            return self.joint_pan_offset_deg, {
                "pan_center_x": self.joint_pan_center_x,
                "pan_raw_dx": 0.0,
                "pan_filtered_dx": 0.0,
                "pan_step_deg": 0.0,
                "pan_offset_deg": self.joint_pan_offset_deg,
                "pan_stationary": True,
            }

        raw_dx = x - self.joint_pan_last_x
        self.joint_pan_last_x = x

        # Reject a detector re-acquisition jump; the next genuine frame starts
        # from the newly observed point instead of moving the real arm.
        if abs(raw_dx) > 0.12:
            raw_dx = 0.0
            self.joint_pan_motion_window.clear()
            self.joint_pan_filtered_dx = 0.0

        self.joint_pan_motion_window.append(raw_dx)
        averaged_dx = sum(self.joint_pan_motion_window) / len(
            self.joint_pan_motion_window
        )
        stationary = (
            len(self.joint_pan_position_window)
            == self.joint_pan_position_window.maxlen
            and max(self.joint_pan_position_window)
            - min(self.joint_pan_position_window)
            <= self.joint_pan_stationary_span_norm
        )

        if stationary:
            self.joint_pan_motion_window.clear()
            self.joint_pan_filtered_dx = 0.0
        else:
            alpha = self.joint_pan_motion_filter_alpha
            self.joint_pan_filtered_dx += alpha * (
                averaged_dx - self.joint_pan_filtered_dx
            )
            if (
                abs(self.joint_pan_filtered_dx)
                <= self.joint_pan_motion_deadband_norm
            ):
                self.joint_pan_filtered_dx = 0.0

        pan_step = (
            self.joint_pan_filtered_dx
            * self.joint_pan_motion_gain_deg
            * self.joint_pan_sign
        )
        pan_step = max(
            -self.joint_pan_motion_max_step_deg,
            min(self.joint_pan_motion_max_step_deg, pan_step),
        )
        self.joint_pan_offset_deg = max(
            -self.joint_pan_offset_limit_deg,
            min(
                self.joint_pan_offset_limit_deg,
                self.joint_pan_offset_deg + pan_step,
            ),
        )
        return self.joint_pan_offset_deg, {
            "pan_center_x": self.joint_pan_center_x,
            "pan_raw_dx": raw_dx,
            "pan_filtered_dx": self.joint_pan_filtered_dx,
            "pan_step_deg": pan_step,
            "pan_offset_deg": self.joint_pan_offset_deg,
            "pan_stationary": stationary,
        }

    def _arm_triangle_to_joint_target(
        self,
        selected: dict,
        palm: list[float],
    ) -> tuple[list[float], dict]:
        """Map human shoulder-elbow-wrist triangle to robot joints 2/3/4.

        The human shoulder/wrist vector defines the 2-link endpoint direction,
        and the human elbow included angle defines the robot elbow bend.  The
        robot link lengths come from the calibrated URDF:
        shoulder_lift->elbow_flex ≈ 0.116 m, elbow_flex->wrist_flex ≈ 0.135 m.
        """
        palm_x = max(0.0, min(1.0, float(palm[0])))
        palm_y = max(0.0, min(1.0, float(palm[1])))

        pan, pan_meta = self._update_pan_motion_offset(palm_x)

        human_elbow_angle = max(35.0, min(175.0, float(selected["elbow_angle"])))

        shoulder = selected["shoulder"]
        wrist = selected["wrist"]
        upper = max(1e-6, float(selected.get("upper_2d", 0.0)))
        lower = max(1e-6, float(selected.get("lower_2d", 0.0)))
        human_arm = max(1e-6, upper + lower)
        reach_ratio = max(0.18, min(1.0, float(selected.get("reach_ratio", 0.5))))

        vertical = (float(wrist.y) - float(shoulder.y)) / human_arm
        vertical_norm = max(
            -1.0,
            min(1.0, vertical / max(1e-6, self.joint_vertical_full_scale)),
        )

        l1 = self.robot_upper_arm_m
        l2 = self.robot_lower_arm_m
        robot_reach = reach_ratio * (l1 + l2) * 0.98
        z = -vertical_norm * (l1 + l2) * 0.45
        x = max(0.05, robot_reach)
        dist = math.hypot(x, z)
        dist_min = abs(l1 - l2) + 0.025
        dist_max = l1 + l2 - 0.006
        if dist < dist_min:
            scale = dist_min / max(1e-6, dist)
            x *= scale
            z *= scale
            dist = dist_min
        elif dist > dist_max:
            scale = dist_max / dist
            x *= scale
            z *= scale
            dist = dist_max

        cos_elbow = (dist * dist - l1 * l1 - l2 * l2) / max(1e-6, 2.0 * l1 * l2)
        cos_elbow = max(-0.98, min(0.98, cos_elbow))
        q_elbow = math.acos(cos_elbow)
        elbow_from_distance = math.degrees(q_elbow)
        elbow_from_human = 180.0 - human_elbow_angle
        elbow_flex = 0.72 * elbow_from_distance + 0.28 * elbow_from_human
        elbow_flex = max(
            self.joint_elbow_min_deg,
            min(self.joint_elbow_max_deg, elbow_flex),
        )
        q_elbow = math.radians(elbow_flex)
        target_angle = math.atan2(z, x)
        shoulder_math = target_angle - math.atan2(
            l2 * math.sin(q_elbow),
            l1 + l2 * math.cos(q_elbow),
        )
        shoulder_lift = -90.0 + math.degrees(shoulder_math)
        shoulder_lift = max(
            self.joint_shoulder_min_deg,
            min(self.joint_shoulder_max_deg, shoulder_lift),
        )

        wrist_flex = -(math.degrees(shoulder_math) + elbow_flex)
        wrist_flex = max(
            self.joint_wrist_comp_min_deg,
            min(self.joint_wrist_comp_max_deg, wrist_flex),
        )

        # Pan already has its own movement filter.  Keep the existing target
        # averaging/low-pass path only for joints 2/3/4 so pan does not acquire
        # a long tail after the hand stops.
        raw = [shoulder_lift, elbow_flex, wrist_flex]
        self.joint_window.append(list(raw))
        averaged = [
            sum(sample[axis] for sample in self.joint_window) / len(self.joint_window)
            for axis in range(3)
        ]

        if self.joint_filtered is None:
            self.joint_filtered = list(averaged)
        else:
            updated = []
            for previous, desired in zip(self.joint_filtered, averaged):
                smoothed = previous + self.joint_filter_alpha * (desired - previous)
                delta = max(
                    -self.joint_max_step_deg,
                    min(self.joint_max_step_deg, smoothed - previous),
                )
                updated.append(previous + delta)
            self.joint_filtered = updated

        return [pan, *self.joint_filtered], {
            "palm_x": palm_x,
            "palm_y": palm_y,
            **pan_meta,
            "vertical_norm": vertical_norm,
            "elbow_angle": human_elbow_angle,
            "reach_ratio": reach_ratio,
            "target_x": x,
            "target_z": z,
        }

    def _arm_angle_to_joint_target(self, selected: dict) -> tuple[list[float], dict]:
        """Map shoulder-elbow angle to joint1 and shoulder-wrist angle to joint4."""
        shoulder = selected["shoulder"]
        elbow = selected["elbow"]
        wrist = selected["wrist"]

        upper_angle = self._vector_angle_deg(shoulder, elbow)
        wrist_angle = self._vector_angle_deg(shoulder, wrist)
        self.joint_angle_window.append([upper_angle, wrist_angle])
        averaged = [
            self._median([sample[axis] for sample in self.joint_angle_window])
            for axis in range(2)
        ]

        if self.joint_angle_filtered is None:
            self.joint_angle_filtered = list(averaged)
        else:
            updated = []
            for previous, desired in zip(self.joint_angle_filtered, averaged):
                delta = self._angle_delta_deg(desired, previous)
                smoothed = previous + self.joint_angle_filter_alpha * delta
                step = self._angle_delta_deg(smoothed, previous)
                step = max(
                    -self.joint_angle_max_step_deg,
                    min(self.joint_angle_max_step_deg, step),
                )
                updated.append(previous + step)
            self.joint_angle_filtered = updated

        if self.joint_angle_baseline is None:
            self.joint_angle_baseline = list(self.joint_angle_filtered)

        pan_delta = self._angle_delta_deg(
            self.joint_angle_filtered[0], self.joint_angle_baseline[0]
        )
        wrist_delta = self._angle_delta_deg(
            self.joint_angle_filtered[1], self.joint_angle_baseline[1]
        )
        pan = max(
            -self.joint_pan_angle_limit_deg,
            min(self.joint_pan_angle_limit_deg, pan_delta * self.joint_pan_angle_gain),
        )
        elbow = max(
            -self.joint_elbow_angle_limit_deg,
            min(
                self.joint_elbow_angle_limit_deg,
                wrist_delta * self.joint_elbow_angle_gain,
            ),
        )
        desired = [pan, elbow]
        if self.joint_angle_output is None:
            self.joint_angle_output = list(desired)
        else:
            updated = []
            for previous, value in zip(self.joint_angle_output, desired):
                if abs(value - previous) < self.joint_angle_deadband_deg:
                    updated.append(previous)
                else:
                    updated.append(value)
            self.joint_angle_output = updated
        pan, elbow = self.joint_angle_output
        return [pan, elbow], {
            "upper_angle": upper_angle,
            "wrist_angle": wrist_angle,
            "upper_angle_filtered": self.joint_angle_filtered[0],
            "wrist_angle_filtered": self.joint_angle_filtered[1],
            "pan_offset": pan,
            "elbow_offset": elbow,
        }

    def _arm_triangle_body_joint_target(self, selected: dict) -> tuple[list[float], dict]:
        """Map the human shoulder-elbow-wrist triangle to robot joints 2/3/4 only.

        Joint 1 is intentionally not computed here.  Its current control path is
        kept unchanged because the pan direction/120-degree spike issue has been
        fixed separately.
        """
        human_elbow_angle = max(35.0, min(175.0, float(selected["elbow_angle"])))

        shoulder = selected["shoulder"]
        wrist = selected["wrist"]
        upper = max(1e-6, float(selected.get("upper_2d", 0.0)))
        lower = max(1e-6, float(selected.get("lower_2d", 0.0)))
        human_arm = max(1e-6, upper + lower)
        reach_ratio = max(0.18, min(1.0, float(selected.get("reach_ratio", 0.5))))

        vertical = (float(wrist.y) - float(shoulder.y)) / human_arm
        vertical_norm = max(
            -1.0,
            min(1.0, vertical / max(1e-6, self.joint_vertical_full_scale)),
        )

        l1 = self.robot_upper_arm_m
        l2 = self.robot_lower_arm_m
        robot_reach = reach_ratio * (l1 + l2) * 0.98
        z = -vertical_norm * (l1 + l2) * 0.45
        x = max(0.05, robot_reach)
        dist = math.hypot(x, z)
        dist_min = abs(l1 - l2) + 0.025
        dist_max = l1 + l2 - 0.006
        if dist < dist_min:
            scale = dist_min / max(1e-6, dist)
            x *= scale
            z *= scale
            dist = dist_min
        elif dist > dist_max:
            scale = dist_max / dist
            x *= scale
            z *= scale
            dist = dist_max

        cos_elbow = (dist * dist - l1 * l1 - l2 * l2) / max(1e-6, 2.0 * l1 * l2)
        cos_elbow = max(-0.98, min(0.98, cos_elbow))
        q_elbow = math.acos(cos_elbow)
        elbow_from_distance = math.degrees(q_elbow)
        elbow_from_human = 180.0 - human_elbow_angle
        elbow_flex = 0.72 * elbow_from_distance + 0.28 * elbow_from_human
        elbow_flex = max(
            self.joint_elbow_min_deg,
            min(self.joint_elbow_max_deg, elbow_flex),
        )

        q_elbow = math.radians(elbow_flex)
        target_angle = math.atan2(z, x)
        shoulder_math = target_angle - math.atan2(
            l2 * math.sin(q_elbow),
            l1 + l2 * math.cos(q_elbow),
        )
        shoulder_lift = -90.0 + math.degrees(shoulder_math)
        shoulder_lift = max(
            self.joint_shoulder_min_deg,
            min(self.joint_shoulder_max_deg, shoulder_lift),
        )

        wrist_flex = -(math.degrees(shoulder_math) + elbow_flex)
        wrist_flex = max(
            self.joint_wrist_comp_min_deg,
            min(self.joint_wrist_comp_max_deg, wrist_flex),
        )

        raw = [shoulder_lift, elbow_flex, wrist_flex]
        self.joint_window.append(list(raw))
        averaged = [
            sum(sample[axis] for sample in self.joint_window) / len(self.joint_window)
            for axis in range(3)
        ]

        if self.joint_filtered is None:
            self.joint_filtered = list(averaged)
        else:
            updated = []
            for previous, desired in zip(self.joint_filtered, averaged):
                smoothed = previous + self.joint_filter_alpha * (desired - previous)
                delta = max(
                    -self.joint_max_step_deg,
                    min(self.joint_max_step_deg, smoothed - previous),
                )
                updated.append(previous + delta)
            self.joint_filtered = updated

        return list(self.joint_filtered), {
            "vertical_norm": vertical_norm,
            "elbow_angle": human_elbow_angle,
            "reach_ratio": reach_ratio,
            "target_x": x,
            "target_z": z,
            "shoulder_lift": self.joint_filtered[0],
            "elbow_flex": self.joint_filtered[1],
            "wrist_flex": self.joint_filtered[2],
        }

    def _joint_command(self, joints: list[float], force: bool = False):
        now = time.monotonic()
        if not force and now - self.last_send_time < self.send_interval:
            return None
        self.last_send_time = now
        grip_closed = bool(self.last_grip)
        self.grip_changed = False
        pan = float(joints[0])
        shoulder_lift = float(joints[1])
        elbow_flex = float(joints[2])
        wrist_flex = float(joints[3])
        return {
            "mode": "joint",
            "joint1": round(pan, 1),
            "joint2": round(shoulder_lift, 1),
            "joint3": round(elbow_flex, 1),
            "joint4": round(wrist_flex, 1),
            "grip": grip_closed,
        }

    def _axis_speed(self, delta: float, maximum: float, deadzone: float) -> float:
        magnitude = abs(float(delta))
        if magnitude <= deadzone:
            return 0.0
        usable = max(1e-6, maximum - deadzone)
        normalized = min(1.0, (magnitude - deadzone) / usable)
        shaped = normalized ** self.response_power
        return math.copysign(shaped * self.speed_scale, delta)

    def _rate_to_speed(self, rate: float, deadzone: float, gain: float) -> float:
        """Map human feature-rate to a decisive, jitter-free robot command."""
        rate = max(-self.maximum_feature_rate, min(self.maximum_feature_rate, float(rate)))
        magnitude = abs(rate)
        if magnitude <= deadzone:
            return 0.0
        speed = (magnitude - deadzone) * gain
        speed = min(self.speed_scale, speed)
        if 0.0 < speed < self.minimum_active_speed:
            speed = self.minimum_active_speed
        return math.copysign(speed, rate)

    def _motion_target(self, feature: list[float]) -> tuple[list[float], list[float]]:
        """Return robot velocity from recent shoulder-relative human motion.

        The aircraft already integrates every servo command into a Cartesian
        delta. Sending displacement-from-neutral as velocity causes drift and
        jitter. Here we send the derivative of the filtered human-arm feature,
        so a stationary hand always converges to a true zero command.
        """
        now = time.monotonic()
        self.motion_history.append((now, list(feature)))
        if len(self.motion_history) < 4:
            self.last_motion_rates = [0.0, 0.0, 0.0]
            return [0.0, 0.0, 0.0], list(self.last_motion_rates)

        oldest_time, oldest_feature = self.motion_history[0]
        # Prefer an observation roughly motion_window_sec old. This suppresses
        # single-frame landmark noise while retaining deliberate arm movement.
        for sample_time, sample_feature in self.motion_history:
            if now - sample_time >= self.motion_window_sec:
                oldest_time, oldest_feature = sample_time, sample_feature
            else:
                break
        dt = max(0.06, now - oldest_time)
        rates = [
            (float(current) - float(previous)) / dt
            for current, previous in zip(feature, oldest_feature)
        ]
        self.last_motion_rates = list(rates)

        target = [
            self._rate_to_speed(
                rates[0], self.reach_rate_deadzone, self.reach_rate_gain
            ),
            self._rate_to_speed(
                rates[1], self.xy_rate_deadzone, self.xy_rate_gain
            ),
            self._rate_to_speed(
                -rates[2], self.xy_rate_deadzone, self.xy_rate_gain
            ),
        ]

        if max(abs(value) for value in target) < self.minimum_active_speed:
            self.stationary_count += 1
            if self.stationary_count >= self.stationary_confirm_frames:
                target = [0.0, 0.0, 0.0]
        else:
            self.stationary_count = 0
        return target, rates

    def _smooth_velocity(self, target: list[float]) -> list[float]:
        now = time.monotonic()
        dt = max(0.001, min(0.10, now - self.last_velocity_time))
        self.last_velocity_time = now
        alpha = 1.0 - math.exp(-dt / self.velocity_time_constant)
        max_step = self.max_velocity_slew_per_sec * dt

        updated = []
        for previous, desired in zip(self.filtered_velocity, target):
            smoothed = previous + alpha * (desired - previous)
            change = max(-max_step, min(max_step, smoothed - previous))
            value = previous + change
            if desired == 0.0 and abs(value) < 0.055:
                value = 0.0
            updated.append(value)
        self.filtered_velocity = updated
        return list(updated)

    def _zero_velocity(self) -> list[float]:
        return self._smooth_velocity([0.0, 0.0, 0.0])

    def _command(self, velocity: list[float], force: bool = False):
        now = time.monotonic()
        if not force and now - self.last_send_time < self.send_interval:
            return None
        self.last_send_time = now
        vx = float(velocity[0])
        vy = float(velocity[1])
        vz = float(velocity[2])

        # Velocity-only GESTURE protocol.
        # Do not send legacy dx/dy/dz: old bridges accumulate those values onto
        # fixed init coordinates and create an absolute pose target. vx/vy/vz
        # must be routed to /gesture/arm_delta, whose IK seed is the latest
        # measured mechanical pose.
        grip_closed = bool(self.last_grip)
        return {
            "mode": "servo",
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "grip": grip_closed,
            "gripper": grip_closed,
            "gripper_close": grip_closed,
            "grip_value": float(max(0.0, min(1.0, self.grip_value))),
        }

    def _draw_hand(self, frame, landmarks) -> None:
        if landmarks is None:
            return
        height, width = frame.shape[:2]
        points = [
            (int(point.x * width), int(point.y * height))
            for point in landmarks
        ]
        for start, end in self.HAND_CONNECTIONS:
            self.cv2.line(frame, points[start], points[end], (255, 120, 0), 1)
        for point in points:
            self.cv2.circle(frame, point, 2, (0, 220, 0), -1)

    def _draw_arm(self, frame, selected) -> None:
        if selected is None:
            return
        height, width = frame.shape[:2]
        shoulder = selected["shoulder"]
        elbow = selected["elbow"]
        wrist = selected["wrist"]
        shoulder_xy = (int(shoulder.x * width), int(shoulder.y * height))
        elbow_xy = (int(elbow.x * width), int(elbow.y * height))
        wrist_xy = (int(wrist.x * width), int(wrist.y * height))

        self.cv2.line(frame, shoulder_xy, elbow_xy, (0, 220, 255), 4)
        self.cv2.line(frame, elbow_xy, wrist_xy, (0, 255, 160), 4)
        self.cv2.circle(frame, shoulder_xy, 10, (0, 0, 255), -1)
        self.cv2.circle(frame, elbow_xy, 8, (0, 200, 255), -1)
        self.cv2.circle(frame, wrist_xy, 7, (0, 255, 0), -1)
        self.cv2.putText(
            frame,
            "ORIGIN: SHOULDER",
            (shoulder_xy[0] + 10, shoulder_xy[1] - 10),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
        )

        if self.baseline_feature is not None:
            arm_2d = self._distance_2d(shoulder, elbow) + self._distance_2d(elbow, wrist)
            neutral = (
                int((float(shoulder.x) + self.baseline_feature[1] * arm_2d) * width),
                int((float(shoulder.y) + self.baseline_feature[2] * arm_2d) * height),
            )
            self.cv2.circle(frame, neutral, 7, (255, 0, 255), 2)
            self.cv2.line(frame, shoulder_xy, neutral, (255, 0, 255), 1)

    def _publish_preview(self, frame, status: str) -> None:
        try:
            ok, encoded = self.cv2.imencode(
                ".jpg",
                frame,
                [self.cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if not ok:
                return
            with self._preview_lock:
                self._preview_jpeg = encoded.tobytes()
                self._preview_status = str(status)
                self._preview_timestamp = time.time()
                self._preview_frame_id += 1
        except Exception:
            return

    def get_preview_snapshot(self) -> dict:
        with self._preview_lock:
            return {
                "jpeg": self._preview_jpeg,
                "status": self._preview_status,
                "timestamp": self._preview_timestamp,
                "frame_id": self._preview_frame_id,
                "build_id": BUILD_ID,
            }

    def update(self):
        """Return (command or None, camera key or None, tracking_lost)."""
        ok, frame = self.cap.read()
        if not ok:
            return None, None, True

        cv2 = self.cv2
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False

        hand, pose_image, pose_world = self._detect(rgb)
        self._update_grip(hand)
        selected = self._select_arm(hand, pose_image, pose_world)

        command = None
        lost = False
        status = "Show shoulder, elbow, wrist and hand"
        color = (0, 0, 255)

        palm_center = self._palm_center(hand)
        if palm_center is not None and selected is not None:
            now = time.monotonic()
            self.last_pose_seen = now
            self._filter_palm_point(palm_center)
            pan_joints, pan_meta = self._arm_angle_to_joint_target(selected)
            body_joints, body_meta = self._arm_triangle_body_joint_target(selected)
            joints = [pan_joints[0], *body_joints]
            command = self._joint_command(joints, force=self.grip_changed)
            status = (
                f"ANG-{selected['side'].upper()} "
                f"upper:{pan_meta['upper_angle_filtered']:+.1f} "
                f"tri:{body_meta['elbow_angle']:.0f} "
                f"pan_off:{joints[0]:+.1f} "
                f"j2:{joints[1]:+.1f} "
                f"j3:{joints[2]:+.1f} "
                f"j4:{joints[3]:+.1f} "
                f"grip:{'CLOSE' if self.last_grip else 'OPEN'} "
                f"g:{self.grip_value:.2f}"
            )
            color = (0, 255, 0)

            if False:
                self.calibration_samples.append(filtered_feature)
                if len(self.calibration_samples) >= self.calibration_frames_required:
                    # Median calibration rejects occasional pose jumps.
                    self.baseline_feature = [
                        self._median(
                            [sample[axis] for sample in self.calibration_samples]
                        )
                        for axis in range(3)
                    ]
                    self.calibration_samples.clear()
                    self.filtered_velocity = [0.0, 0.0, 0.0]
                    print("\n肩关节运动跟随已就绪：移动手臂时机械臂运动，停手时机械臂停止")
                velocity = self._zero_velocity()
                command = self._command(velocity)
                status = (
                    f"Calibrating shoulder origin "
                    f"{min(len(self.calibration_samples), self.calibration_frames_required)}"
                    f"/{self.calibration_frames_required}"
                )
                color = (0, 220, 255)
            elif False:
                target, rates = self._motion_target(filtered_feature)
                velocity = self._smooth_velocity(target)
                command = self._command(velocity)
                moving = max(abs(value) for value in target) > 0.0
                status = (
                    f"MOTION-{selected['side'].upper()} q:{selected['quality']:.2f} "
                    f"r:{rates[0]:+.2f},{rates[1]:+.2f},{rates[2]:+.2f} "
                    f"v:{velocity[0]:+.2f},{velocity[1]:+.2f},{velocity[2]:+.2f} "
                    f"{'MOVE' if moving else 'STOP'} "
                    f"grip:{'CLOSE' if self.last_grip else 'OPEN'} "
                    f"g:{self.grip_value:.2f} {selected['reach_source']}"
                )
                color = (0, 255, 0) if moving else (0, 220, 255)
        else:
            command = None
            if self.last_pose_seen is not None:
                elapsed = time.monotonic() - self.last_pose_seen
                status = f"Arm/hand temporarily lost - holding ({elapsed:.2f}s)"
                if elapsed >= self.pose_loss_hold_sec:
                    lost = True
                    self.last_pose_seen = None
                    self.recenter()
            else:
                status = "Waiting for shoulder, elbow, wrist and hand"

        self._draw_hand(frame, hand)
        self._draw_arm(frame, selected)
        cv2.putText(
            frame,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2,
        )
        cv2.putText(
            frame,
            "GESTURE natural mode | shoulder-elbow=joint1 | shoulder-elbow-wrist=joint2/3/4 | open/close=gripper | C:center",
            (10, frame.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (230, 230, 230),
            1,
        )
        self._publish_preview(frame, status)

        key = None
        if self.show_window:
            cv2.imshow("SO101 Gesture Joint Control", frame)
            key_code = cv2.waitKey(1) & 0xFF
            key = chr(key_code) if key_code not in (0xFF, 255) else None
        return command, key, lost

    def close(self) -> None:
        try:
            if hasattr(self, "cap"):
                self.cap.release()
            if hasattr(self, "hand_detector"):
                self.hand_detector.close()
            if hasattr(self, "pose_detector"):
                self.pose_detector.close()
            if hasattr(self, "cv2") and self.show_window:
                self.cv2.destroyAllWindows()
            if hasattr(self, "_preview_lock"):
                with self._preview_lock:
                    self._preview_jpeg = None
                    self._preview_status = "手势摄像头已关闭"
                    self._preview_timestamp = time.time()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SO-101 自主/主从/手势统一地面控制器"
    )
    parser.add_argument("--leader-port", help="主臂串口，例如 COM5")
    parser.add_argument("--radio-port", help="地面数传串口，例如 COM7")
    parser.add_argument("--leader-id", default="ground_leader", help="主臂原校准 ID")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--hz", type=float, default=30.0, help="控制循环频率")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument(
        "--hand-model",
        help="MediaPipe Tasks hand_landmarker.task 路径；留空则自动下载",
    )
    parser.add_argument(
        "--gesture-only",
        action="store_true",
        help="不连接主臂，启动后直接等待手势",
    )
    parser.add_argument(
        "--gesture-speed-scale",
        type=float,
        default=0.90,
        help="手势速度缩放，建议 0.3~1.0",
    )
    parser.add_argument(
        "--eeg-only",
        action="store_true",
        help="EEG mode only; do not connect the leader arm",
    )
    parser.add_argument(
        "--eeg-down-pose",
        type=parse_pose,
        default=EEG_DOWN_POSE,
        help="EEG down preset: pan,lift,elbow,wrist_flex,wrist_roll,gripper",
    )
    parser.add_argument(
        "--eeg-move-sec",
        type=float,
        default=3.0,
        help="seconds to stream the EEG down preset",
    )
    parser.add_argument(
        "--eeg-open-gripper",
        type=float,
        default=0.0,
        help="remembered gripper value after EEG open",
    )
    parser.add_argument(
        "--eeg-close-gripper",
        type=float,
        default=45.0,
        help="remembered gripper value after EEG close",
    )
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def run_self_test() -> None:
    sample = encode_frame("J,1,0.00,-90.00,90.00,0.00,0.00,0.00")
    assert sample.startswith(b"J,1,") and sample.endswith(b"\n")
    gesture = json.dumps(
        {"mode": "servo", "vx": 0.0, "vy": 0.0, "vz": 0.0, "grip": False},
        separators=(",", ":"),
    )
    assert gesture.startswith("{") and gesture.endswith("}")
    print("统一地面控制器协议自检通过")


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if not args.radio_port:
        raise ValueError("必须提供 --radio-port")
    if not (args.gesture_only or args.eeg_only) and not args.leader_port:
        raise ValueError("非手势专用模式必须提供 --leader-port")
    if not 0 < args.hz <= 50:
        raise ValueError("--hz 必须在 0 到 50 之间")

    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("缺少 pyserial，请执行: pip install pyserial") from exc

    leader = None
    if not (args.gesture_only or args.eeg_only):
        try:
            from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
        except ImportError as exc:
            raise RuntimeError(
                "找不到 LeRobot，请先激活原来能读取 SO-101 主臂的 Python 环境。"
            ) from exc

        leader_id = None if args.leader_id.lower() in {"none", "null"} else args.leader_id
        leader = SO101Leader(
            SO101LeaderConfig(
                port=args.leader_port,
                id=leader_id,
                use_degrees=True,
            )
        )
    radio = None
    gesture = None
    mode = "HOLD"
    sequence = 0
    period = 1.0 / args.hz
    values = [0.0] * len(MOTOR_NAMES)
    eeg_gripper_value = float(args.eeg_down_pose[5])

    def close_gesture():
        nonlocal gesture
        current = gesture
        gesture = None
        close_gesture_async(current)

    def enter_hold(reason=""):
        nonlocal mode
        mode = "HOLD"
        radio.write(encode_frame("MODE,HOLD"))
        radio.write(encode_frame(f"H,{sequence}"))
        radio.flush()
        if reason:
            print(f"\n状态: HOLD（{reason}）")
        else:
            print("\n状态: HOLD")

    def send_eeg_command(command):
        radio.write(encode_frame("MODE,EEG"))
        radio.flush()
        time.sleep(0.05)
        radio.write(encode_frame(f"EEG,{command}"))
        radio.flush()

    def enter_eeg():
        nonlocal mode
        mode = "EEG"
        radio.write(encode_frame("MODE,EEG"))
        radio.flush()
        close_gesture()
        print("\n状态: EEG 脑电控制")

    def stream_eeg_down():
        nonlocal mode, sequence, values
        mode = "EEG"
        close_gesture()
        radio.write(encode_frame("MODE,EEG"))
        radio.flush()
        end_time = time.monotonic() + args.eeg_move_sec
        while time.monotonic() < end_time:
            values = list(args.eeg_down_pose[:5]) + [eeg_gripper_value]
            body = "J," + str(sequence) + "," + ",".join(
                f"{value:.2f}" for value in values
            )
            radio.write(encode_frame(body))
            read_feedback(radio)
            sequence = (sequence + 1) & 0xFFFF
            time.sleep(period)
        radio.write(encode_frame("MODE,EEG"))
        radio.flush()
        print("\n状态: EEG 指地动作完成")

    if leader is not None:
        print("正在连接主臂……")
        leader.connect()
    try:
        print("正在连接数传……")
        radio = serial.Serial(
            port=args.radio_port,
            baudrate=args.baudrate,
            timeout=0,
            write_timeout=0.2,
        )
        time.sleep(0.5)
        radio.reset_input_buffer()
        radio.write(encode_frame("MODE,HOLD"))
        radio.flush()
        print("连接完成。T/空格=主从，G=手势，A=自主，H=保持，C=定中，Q=退出。")
        print("首次启动为 HOLD，确认从臂周围安全后再切换。")

        if args.gesture_only:
            gesture = GestureController(
                args.camera_index,
                args.gesture_speed_scale,
                args.hand_model,
            )
            mode = "GESTURE_WAIT"
            print("手势专用模式已启动，检测到手后自动进入 GESTURE。")

        next_tick = time.monotonic()
        while True:
            gesture_command = None
            camera_key = None
            hand_lost = False
            if gesture is not None:
                gesture_command, camera_key, hand_lost = gesture.update()

            key = poll_key() or camera_key
            if key:
                lower = key.lower()
                if lower == "q":
                    break
                if key == " " or lower == "t":
                    if leader is None:
                        enter_hold("未连接主臂，无法进入 TELEOP")
                        continue
                    if mode == "TELEOP":
                        enter_hold()
                        close_gesture()
                    else:
                        mode = "TELEOP"
                        radio.write(encode_frame("MODE,TELEOP"))
                        radio.flush()
                        close_gesture()
                        print("\n状态: TELEOP 主从跟随")
                elif lower == "g":
                    if gesture is None:
                        try:
                            gesture = GestureController(
                                args.camera_index,
                                args.gesture_speed_scale,
                                args.hand_model,
                            )
                            mode = "GESTURE_WAIT"
                            radio.write(encode_frame("MODE,HOLD"))
                            radio.flush()
                            print("\n手势摄像头已打开，检测到手后自动进入 GESTURE。")
                        except Exception as exc:
                            enter_hold(f"手势启动失败: {exc}")
                    else:
                        gesture.recenter()
                        if mode == "GESTURE":
                            radio.write(encode_frame("GESTURE_RESET"))
                            radio.flush()
                elif lower == "a":
                    mode = "AUTO"
                    radio.write(encode_frame("MODE,AUTO"))
                    radio.write(encode_frame("start"))
                    radio.flush()
                    close_gesture()
                    print("\n状态: AUTO 自主抓取")
                elif lower == "e":
                    enter_eeg()
                elif lower == "o":
                    eeg_gripper_value = args.eeg_open_gripper
                    send_eeg_command("OPEN")
                    mode = "EEG"
                    print("\n状态: EEG 夹爪打开")
                elif lower == "p":
                    eeg_gripper_value = args.eeg_close_gripper
                    send_eeg_command("CLOSE")
                    mode = "EEG"
                    print("\n状态: EEG 夹爪闭合")
                elif lower == "d":
                    stream_eeg_down()
                elif lower == "s":
                    eeg_gripper_value = args.eeg_open_gripper
                    send_eeg_command("OPEN")
                    mode = "STOW"
                    radio.write(encode_frame("MODE,STOW"))
                    radio.flush()
                    close_gesture()
                    print("\n状态: EEG 回零位 / STOW")
                elif lower == "h":
                    enter_hold()
                    close_gesture()
                elif lower == "c" and gesture is not None:
                    gesture.recenter()
                    if mode == "GESTURE":
                        radio.write(encode_frame("GESTURE_RESET"))
                        radio.flush()

            if hand_lost and mode == "GESTURE":
                enter_hold("丢失手势")
                mode = "GESTURE_WAIT"

            if gesture_command is not None and mode in {"GESTURE", "GESTURE_WAIT"}:
                if mode == "GESTURE_WAIT":
                    mode = "GESTURE"
                    radio.write(encode_frame("MODE,GESTURE"))
                    print("\n状态: GESTURE 手势控制")
                payload = json.dumps(gesture_command, separators=(",", ":"))
                payload_bytes = payload.encode("ascii")
                if len(payload_bytes) > 220:
                    print(f"\n手势指令过长，已丢弃: {len(payload_bytes)} bytes")
                else:
                    radio.write(payload_bytes + b"\n")

            if leader is not None:
                try:
                    action = leader.get_action()
                    values = [float(action[f"{name}.pos"]) for name in MOTOR_NAMES]
                except Exception as exc:
                    enter_hold(f"主臂读取失败: {exc}")
                    close_gesture()
                    time.sleep(0.2)
                    continue

                if not all(math.isfinite(value) for value in values):
                    enter_hold("主臂数据无效")

                    close_gesture()

            if mode == "TELEOP":
                body = "J," + str(sequence) + "," + ",".join(
                    f"{value:.2f}" for value in values
                )
                radio.write(encode_frame(body))

            read_feedback(radio)
            compact = " ".join(f"{value:7.1f}" for value in values)
            print(f"\r{mode:12s} seq={sequence:5d}  {compact}", end="", flush=True)

            sequence = (sequence + 1) & 0xFFFF
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        close_gesture()
        if radio is not None and radio.is_open:
            try:
                radio.write(encode_frame("MODE,HOLD"))
                radio.write(encode_frame(f"H,{sequence}"))
                radio.flush()
            except Exception:
                pass
            radio.close()
        if leader is not None:
            leader.disconnect()
        print("\n已进入 HOLD，主臂、摄像头和数传串口已关闭。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
