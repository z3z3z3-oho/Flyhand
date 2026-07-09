#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SO-101 ground controller for TELEOP, GESTURE, AUTO and HOLD.

Gesture mode uses the operator's shoulder as the coordinate origin. The
shoulder, elbow and wrist form a normalized arm model. Unlike the previous
position-offset controller, this build converts filtered human-arm MOTION into
robot velocity. The robot therefore moves while the hand moves and stops when
the hand stops, which prevents neutral-point drift and repeated micro-jitter.
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

BUILD_ID = "2026-07-09-current-pose-zero-v9"
PROTOCOL_BUILD_ID = "2026-07-09-single-radio-core-v10"


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
            print(f"\r串口反馈: {raw}{' ' * 20}")


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
        self.max_velocity_slew_per_sec = 10.0
        self.send_interval = 1.0 / 20.0
        self.pose_loss_hold_sec = 0.65
        self.minimum_pose_quality = 0.24
        self.calibration_frames_required = 14
        self.side_switch_confirm_frames = 8
        self.stationary_confirm_frames = 2

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
        self.grip_close_confirm_frames = 1
        self.grip_open_confirm_frames = 1
        self.grip_close_score = 0.54
        self.grip_open_score = 0.32
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

    def recenter(self) -> None:
        self.feature_window.clear()
        self.motion_history.clear()
        self.feature_filtered = None
        self.baseline_feature = None
        self.calibration_samples.clear()
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.last_motion_rates = [0.0, 0.0, 0.0]
        self.stationary_count = 0
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

        if self.last_grip:
            raw_state = not (self.grip_value <= self.grip_open_score)
            confirm = self.grip_open_confirm_frames
        else:
            raw_state = self.grip_value >= self.grip_close_score
            confirm = self.grip_close_confirm_frames

        if raw_state != self.grip_candidate:
            self.grip_candidate = raw_state
            self.grip_candidate_count = 1
        else:
            self.grip_candidate_count += 1

        if self.grip_candidate_count >= confirm:
            self.last_grip = self.grip_candidate

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
        lateral = (float(wrist.x) - float(shoulder.x)) / arm_2d
        vertical = (float(wrist.y) - float(shoulder.y)) / arm_2d

        hand_wrist = hand[self.HAND_WRIST] if hand is not None else None
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
            "reach_source": f"ANGLE:{elbow_angle:.0f}",
            "elbow_angle": elbow_angle,
            "reach_ratio": reach_ratio,
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
                self.recenter()
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
            return self._command(self._zero_velocity(), force=True), None, True

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

        if selected is not None:
            now = time.monotonic()
            self.last_pose_seen = now
            filtered_feature = self._filter_feature(selected["feature"])

            if self.baseline_feature is None:
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
            else:
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
            velocity = self._zero_velocity()
            command = self._command(velocity)
            if self.last_pose_seen is not None:
                elapsed = time.monotonic() - self.last_pose_seen
                status = f"Pose temporarily lost - stopping ({elapsed:.2f}s)"
                if elapsed >= self.pose_loss_hold_sec:
                    lost = True
                    self.last_pose_seen = None
                    self.recenter()
            else:
                status = "Waiting for shoulder, elbow and wrist"

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
            "Shoulder=origin | MOVE while arm moves | still arm = STOP | C:center",
            (10, frame.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (230, 230, 230),
            1,
        )
        self._publish_preview(frame, status)

        key = None
        if self.show_window:
            cv2.imshow("SO101 Shoulder-Origin Arm Follower", frame)
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


class LeaderSampler:
    """Continuously sample the leader without blocking radio transmission."""

    def __init__(self, leader) -> None:
        self.leader = leader
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="so101-leader-sampler",
            daemon=True,
        )
        self._values: list[float] | None = None
        self._sample_time = 0.0
        self._error: str | None = None

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                action = self.leader.get_action()
                values = [float(action[f"{name}.pos"]) for name in MOTOR_NAMES]
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError("领导机返回了非有限数值")
                with self._lock:
                    self._values = values
                    self._sample_time = time.monotonic()
                    self._error = None
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                self._stop.wait(0.01)

    def snapshot(self) -> tuple[list[float] | None, float, str | None]:
        with self._lock:
            values = None if self._values is None else list(self._values)
            sample_time = self._sample_time
            error = self._error
        age = float("inf") if sample_time <= 0.0 else time.monotonic() - sample_time
        return values, age, error

    def wait_ready(self, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            values, age, _ = self.snapshot()
            if values is not None and age < 0.5:
                return True
            if self._stop.wait(0.02):
                break
        return False

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.5)


class TeleopSender:
    """Send joint frames at a conservative fixed rate, independent of the UI loop."""

    def __init__(self, radio, radio_lock, leader_sampler, hz: float = 15.0) -> None:
        self.radio = radio
        self.radio_lock = radio_lock
        self.leader_sampler = leader_sampler
        self.hz = max(5.0, min(30.0, float(hz)))
        self._active = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="so101-teleop-radio-stream",
            daemon=True,
        )
        self._sequence = 0
        self._last_send = 0.0
        self._sent_count = 0
        self._last_error: str | None = None
        self._stats_lock = threading.Lock()

    def start(self) -> None:
        self._thread.start()

    def activate(self, sequence: int = 0) -> None:
        with self._stats_lock:
            self._sequence = int(sequence) & 0xFFFF
            self._last_send = 0.0
            self._last_error = None
        self._active.set()

    def deactivate(self) -> None:
        self._active.clear()
        # Wait for any in-flight write. The sender checks _active again after
        # acquiring this same lock, so no J frame can be emitted after return.
        with self.radio_lock:
            pass

    def _run(self) -> None:
        period = 1.0 / self.hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            if not self._active.is_set():
                self._stop.wait(0.005)
                next_tick = time.monotonic()
                continue

            values, age, error = self.leader_sampler.snapshot()
            if values is None or age > 0.80:
                with self._stats_lock:
                    self._last_error = error or f"leader sample age={age:.3f}s"
            else:
                with self._stats_lock:
                    sequence = self._sequence
                # One decimal keeps the frame short, which is safer for small
                # transparent radio packet buffers and is sufficient for servos.
                body = "J," + str(sequence) + "," + ",".join(
                    f"{value:.2f}" for value in values
                )
                frame = encode_frame(body)
                try:
                    with self.radio_lock:
                        if self._active.is_set() and self.radio.is_open:
                            self.radio.write(frame)
                    if self._active.is_set():
                        now = time.monotonic()
                        with self._stats_lock:
                            self._sequence = (sequence + 1) & 0xFFFF
                            self._last_send = now
                            self._sent_count += 1
                            self._last_error = None
                except Exception as exc:
                    with self._stats_lock:
                        self._last_error = str(exc)

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_tick = time.monotonic()

    def stats(self) -> tuple[int, float, int, str | None]:
        with self._stats_lock:
            sequence = self._sequence
            last_send = self._last_send
            sent_count = self._sent_count
            error = self._last_error
        age = float("inf") if last_send <= 0.0 else time.monotonic() - last_send
        return sequence, age, sent_count, error

    def stop(self) -> None:
        self._active.clear()
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.5)



class RadioProtocol:
    """Single source of truth for the transparent-radio framing sequence.

    Both the command-line program and Web/Tk UI must call this class. Keeping
    startup synchronization and mode transitions here prevents the UI from
    silently diverging from the already tested command-line behavior.
    """

    def __init__(self, radio, radio_lock: threading.Lock) -> None:
        self.radio = radio
        self.radio_lock = radio_lock

    def write(self, data: bytes, flush: bool = False) -> None:
        if self.radio is None or not self.radio.is_open:
            return
        with self.radio_lock:
            self.radio.write(data)
            if flush:
                self.radio.flush()

    def frame(self, body: str, flush: bool = False) -> None:
        self.write(encode_frame(body), flush=flush)

    def sync_startup(self) -> None:
        """Clear stale fragments and enter HOLD exactly as the CLI does."""
        try:
            self.radio.reset_input_buffer()
        except Exception:
            pass
        try:
            self.radio.reset_output_buffer()
        except Exception:
            pass

        self.frame("MODE,HOLD", flush=True)
        time.sleep(0.15)

        # Drop the acknowledgement and any stale data from the previous
        # process, then send one clean command that becomes the known baseline.
        try:
            self.radio.reset_input_buffer()
        except Exception:
            pass
        self.frame("MODE,HOLD", flush=True)

    def enter_hold(self, sequence: int) -> None:
        """Atomically send the same two HOLD frames used by the CLI."""
        self.write(
            encode_frame("MODE,HOLD")
            + encode_frame(f"H,{int(sequence) & 0xFFFF}"),
            flush=True,
        )

    def enter_teleop(
        self,
        latest: list[float],
        sequence: int,
        teleop_sender,
    ) -> int:
        """Enter TELEOP using the tested guard time and three prime frames."""
        teleop_sender.deactivate()
        self.frame("MODE,TELEOP", flush=True)
        time.sleep(0.12)

        prime_sequence = int(sequence) & 0xFFFF
        for _ in range(3):
            body = "J," + str(prime_sequence) + "," + ",".join(
                f"{float(value):.2f}" for value in latest
            )
            self.frame(body, flush=True)
            prime_sequence = (prime_sequence + 1) & 0xFFFF
            time.sleep(0.06)

        teleop_sender.activate(prime_sequence)
        return prime_sequence

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SO-101 自动/遥控/手势统一前端控制器"
    )
    parser.add_argument("--leader-port", help="SO-101 领导机串口，例如: COM7")
    parser.add_argument("--radio-port", help="地面无线电串口，例如: COM10")
    parser.add_argument("--leader-id", default="blue", help="领导机校准 ID")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--hz", type=float, default=30.0, help="控制循环频率")
    parser.add_argument(
        "--teleop-tx-hz",
        type=float,
        default=15.0,
        help="主从关节帧发送频率；透明数传建议 10~20Hz",
    )
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument(
        "--hand-model",
        help="hand_landmarker.task 路径；留空时使用同目录文件或自动下载",
    )
    parser.add_argument(
        "--pose-model",
        help="pose_landmarker_full.task 路径；留空时使用同目录文件或自动下载",
    )
    parser.add_argument(
        "--gesture-only",
        action="store_true",
        help="仅手势模式，禁用领导机遥控",
    )
    parser.add_argument(
        "--gesture-speed-scale",
        type=float,
        default=0.90,
        help="手势运动跟随最大速度，建议 0.75~1.00",
    )
    parser.add_argument(
        "--hand-detection-confidence",
        type=float,
        default=0.42,
        help="手部初始检测阈值，建议 0.35~0.50",
    )
    parser.add_argument(
        "--hand-tracking-confidence",
        type=float,
        default=0.38,
        help="手部连续跟踪阈值，建议 0.30~0.45",
    )
    parser.add_argument(
        "--pose-detection-confidence",
        type=float,
        default=0.42,
        help="人体姿态初始检测阈值，建议 0.35~0.50",
    )
    parser.add_argument(
        "--pose-tracking-confidence",
        type=float,
        default=0.38,
        help="人体姿态连续跟踪阈值，建议 0.30~0.45",
    )
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def run_self_test() -> None:
    sample = encode_frame("J,1,0.00,-90.00,90.00,0.00,0.00,0.00")
    assert sample.startswith(b"J,1,") and sample.endswith(b"\n")
    gesture = json.dumps(
        {"mode": "servo", "vx": 0.0, "vy": 0.0, "vz": 0.0, "grip": False, "grip_value": 0.0},
        separators=(",", ":"),
    )
    assert gesture.startswith("{") and gesture.endswith("}")
    print(f"统一前端控制模块基本自测通过 | {BUILD_ID}")


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if not args.radio_port:
        raise ValueError("必须提供 --radio-port")
    if not args.gesture_only and not args.leader_port:
        raise ValueError("非纯手势模式时必须提供 --leader-port")
    if not 0 < args.hz <= 50:
        raise ValueError("--hz 必须在 0 到 50 之间")
    if not 5 <= args.teleop_tx_hz <= 30:
        raise ValueError("--teleop-tx-hz 必须在 5 到 30 之间")

    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("缺少 pyserial，请执行: pip install pyserial") from exc

    leader = None
    if not args.gesture_only:
        try:
            from so101_leader import SO101Leader, SO101LeaderConfig
        except ImportError as exc:
            raise RuntimeError("无法导入 so101_leader.py") from exc

        leader_id = None if args.leader_id.lower() in {"none", "null"} else args.leader_id
        leader = SO101Leader(
            SO101LeaderConfig(
                port=args.leader_port,
                id=leader_id,
                use_degrees=True,
            )
        )

    radio = None
    radio_lock = threading.Lock()
    protocol = None
    gesture = None
    leader_sampler = None
    teleop_sender = None
    mode = "HOLD"
    sequence = 0
    period = 1.0 / args.hz
    values = [0.0] * len(MOTOR_NAMES)

    print(f"构建版本: {BUILD_ID}")
    print(f"运行文件: {Path(__file__).resolve()}")

    def send_radio(data: bytes, flush: bool = False) -> None:
        if protocol is not None:
            protocol.write(data, flush=flush)
            return
        if radio is None or not radio.is_open:
            return
        with radio_lock:
            radio.write(data)
            if flush:
                radio.flush()

    def close_gesture():
        nonlocal gesture
        if gesture is not None:
            gesture.close()
            gesture = None

    def open_gesture():
        return GestureController(
            args.camera_index,
            args.gesture_speed_scale,
            args.hand_model,
            args.pose_model,
            args.hand_detection_confidence,
            args.hand_tracking_confidence,
            args.pose_detection_confidence,
            args.pose_tracking_confidence,
        )

    def enter_hold(reason=""):
        nonlocal mode
        mode = "HOLD"
        if teleop_sender is not None:
            teleop_sender.deactivate()
        if protocol is not None:
            protocol.enter_hold(sequence)
        elif radio is not None and radio.is_open:
            send_radio(
                encode_frame("MODE,HOLD") + encode_frame(f"H,{sequence}"),
                flush=True,
            )
        if reason:
            print(f"\n提示: HOLD，原因: {reason}")
        else:
            print("\n提示: HOLD")

    if leader is not None:
        print("正在连接领导机...")
        leader.connect()
        leader_sampler = LeaderSampler(leader)
        leader_sampler.start()
        if leader_sampler.wait_ready(2.0):
            print("领导机数据采样已就绪")
        else:
            _, _, startup_error = leader_sampler.snapshot()
            print(f"警告：领导机首帧尚未就绪: {startup_error or '等待数据'}")

    try:
        print("正在连接无线电...")
        radio = serial.Serial(
            port=args.radio_port,
            baudrate=args.baudrate,
            timeout=0,
            write_timeout=0.2,
        )
        time.sleep(0.7)
        protocol = RadioProtocol(radio, radio_lock)
        protocol.sync_startup()
        if leader_sampler is not None:
            teleop_sender = TeleopSender(
                radio,
                radio_lock,
                leader_sampler,
                hz=args.teleop_tx_hz,
            )
            teleop_sender.start()
        print("连接完成。T=teleop, G=gesture, A=auto, H=hold, C=center, Q=quit")
        print("安全提示：首次测试建议 --gesture-speed-scale 0.35")

        if args.gesture_only:
            gesture = open_gesture()
            mode = "GESTURE_WAIT"
            print("手势模式正在初始化，识别并定中后自动进入 GESTURE")

        next_tick = time.monotonic()
        while True:
            gesture_command = None
            camera_key = None
            tracking_lost = False
            if gesture is not None:
                gesture_command, camera_key, tracking_lost = gesture.update()

            key = poll_key() or camera_key
            if key:
                lower = key.lower()
                if lower == "q":
                    break
                if key == " " or lower == "t":
                    if leader is None:
                        enter_hold("当前没有连接领导机，无法进入 TELEOP")
                        continue
                    close_gesture()
                    if mode == "TELEOP":
                        enter_hold()
                    else:
                        latest, sample_age, sample_error = (
                            leader_sampler.snapshot()
                            if leader_sampler is not None
                            else (None, float("inf"), "领导机采样器未启动")
                        )
                        if latest is None or sample_age > 0.50:
                            enter_hold(
                                "领导机数据未就绪或已过期: "
                                + (sample_error or f"age={sample_age:.3f}s")
                            )
                            continue
                        values = latest
                        if teleop_sender is None:
                            enter_hold("TELEOP 独立发送器未启动")
                            continue
                        mode = "TELEOP"
                        sequence = protocol.enter_teleop(
                            latest,
                            sequence,
                            teleop_sender,
                        )
                        print(
                            f"\n提示: TELEOP 进入主从跟随"
                            f"（独立 {args.teleop_tx_hz:.1f}Hz 数据流）"
                        )
                elif lower == "g":
                    if gesture is None:
                        try:
                            gesture = open_gesture()
                            mode = "GESTURE_WAIT"
                            if teleop_sender is not None:
                                teleop_sender.deactivate()
                            send_radio(encode_frame("MODE,HOLD"), flush=True)
                            print("\n手势控制器已打开，保持肩、肘、腕稳定完成定中")
                        except Exception as exc:
                            enter_hold(f"手势开启失败: {exc}")
                    else:
                        gesture.recenter()
                        if mode == "GESTURE":
                            send_radio(encode_frame("GESTURE_RESET"), flush=True)
                elif lower == "a":
                    close_gesture()
                    mode = "AUTO"
                    if teleop_sender is not None:
                        teleop_sender.deactivate()
                    send_radio(
                        encode_frame("MODE,AUTO") + encode_frame("start"),
                        flush=True,
                    )
                    print("\n提示: AUTO 自动接管")
                elif lower == "h":
                    close_gesture()
                    enter_hold()
                elif lower == "c" and gesture is not None:
                    gesture.recenter()
                    if mode == "GESTURE":
                        send_radio(encode_frame("GESTURE_RESET"), flush=True)

            if tracking_lost and mode == "GESTURE":
                enter_hold("肩、肘或腕跟踪丢失")
                mode = "GESTURE_WAIT"

            if gesture_command is not None and mode in {"GESTURE", "GESTURE_WAIT"}:
                # Do not enter GESTURE until calibration has completed. A nonzero
                # command or an existing baseline indicates that tracking is ready.
                ready = gesture is not None and gesture.baseline_feature is not None
                if mode == "GESTURE_WAIT" and ready:
                    mode = "GESTURE"
                    if teleop_sender is not None:
                        teleop_sender.deactivate()
                    send_radio(encode_frame("MODE,GESTURE"), flush=True)
                    time.sleep(0.05)
                    send_radio(encode_frame("GESTURE_RESET"), flush=True)
                    print("\n提示: GESTURE 当前机械臂实测姿态自动设为零位")
                if mode == "GESTURE":
                    payload = json.dumps(gesture_command, separators=(",", ":"))
                    send_radio((payload + "\n").encode("ascii"))

            if leader_sampler is not None:
                latest, sample_age, sample_error = leader_sampler.snapshot()
                if latest is not None:
                    values = latest

            if mode == "TELEOP":
                if leader_sampler is None or teleop_sender is None:
                    enter_hold("领导机采样器或 TELEOP 发送器未启动")
                    continue
                latest, sample_age, sample_error = leader_sampler.snapshot()
                tx_sequence, tx_age, tx_count, tx_error = teleop_sender.stats()
                if latest is None or sample_age > 0.80:
                    enter_hold(
                        "领导机数据中断: "
                        + (sample_error or f"age={sample_age:.3f}s")
                    )
                    continue
                if tx_count > 0 and tx_age > 0.25:
                    enter_hold(
                        "TELEOP 无线电发送中断: "
                        + (tx_error or f"tx_age={tx_age:.3f}s")
                    )
                    continue
                values = latest
                sequence = tx_sequence

            read_feedback(radio)
            if mode == "TELEOP":
                compact = " ".join(f"{value:7.1f}" for value in values)
                line = f"\r{mode:12s} seq={sequence:5d}  {compact}"
            else:
                line = f"\r{mode:12s} seq={sequence:5d}"
            print(line, end="", flush=True)

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
        if teleop_sender is not None:
            teleop_sender.deactivate()
            teleop_sender.stop()
        if radio is not None and radio.is_open:
            try:
                if protocol is not None:
                    protocol.enter_hold(sequence)
                else:
                    send_radio(
                        encode_frame("MODE,HOLD")
                        + encode_frame(f"H,{sequence}"),
                        flush=True,
                    )
            except Exception:
                pass
            radio.close()
        if leader_sampler is not None:
            leader_sampler.stop()
        if leader is not None:
            try:
                leader.disconnect()
            except Exception:
                pass
        print("\n已退出。HOLD，所有舵机和数据流已停止并关闭。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
