#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
so101_arm_interface.py — 终极修复版 (2025-04)
=========================================
修复内容：
1. 删除ROS2一次性定时器坑，控制环直接启动
2. 强制打印cur/tgt，确保能看到
3. 硬件同步后直接启动，避免初始抽搐
4. 保留 AUTO/TELEOP/HOLD/STOW，新增 GESTURE 模式
5. 手势增量控制独立于 AUTO 视觉伺服，不共用 delta_scale/停止逻辑/目标领先限制
6. 手势夹爪端点独立配置，避免影响 AUTO 自主抓取夹爪
7. 修复 GESTURE 工作空间硬限幅导致目标瞬间跳到不可达点的问题
"""

import os
import sys
import glob
import threading
import time
import queue

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

# ── 可选依赖 ────────────────────────────────────────────────
# 🔥 关键：在导入placo之前设置ROS_PACKAGE_PATH
import os
import sys
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('pine_harvester')

# 添加当前包的share目录到ROS_PACKAGE_PATH
for envkey in ("ROS_PACKAGE_PATH", "AMENT_PREFIX_PATH"):
    old = os.environ.get(envkey, "")
    if pkg_share not in old:
        os.environ[envkey] = pkg_share + (":" + old if old else "")

# 现在再导入placo
try:
    import placo
    PLACO_OK = True
except ImportError:
    PLACO_OK = False

try:
    from scipy.spatial.transform import Rotation as SciR
    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False

# ── sts3215_driver 导入（多路径兼容）───────────────────────
try:
    from pine_harvester.sts3215_driver import SO101Follower
except ImportError:
    _d = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(_d))
    from sts3215_driver import SO101Follower

# ── 常量 ────────────────────────────────────────────────────
MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]
IK_NAMES    = MOTOR_NAMES[:5]

HOME_ANGLES = {
    "shoulder_pan":   0.0,
    "shoulder_lift": -90.0,
    "elbow_flex":     90.0,
    "wrist_flex":      0.0,
    "wrist_roll":      0.0,
    "gripper":         0.0,
}
JOINT_LIMITS = {
    "shoulder_pan":  (-150, 150),
    "shoulder_lift": (-120, 100),
    "elbow_flex":    (-120, 120),
    "wrist_flex":    (-100, 100),
    "wrist_roll":    (-180, 180),
    "gripper":       (   0, 100),
}


# ════════════════════════════════════════════════════════════
# 运动学（placo 封装）
# ════════════════════════════════════════════════════════════
class ArmKinematics:
    def __init__(self, urdf_path: str,
             tip_frame: str = "gripper_frame_link"):
        # 🔥 终极解决方案：在内存中直接修改URDF内容
        # 读取URDF文件内容
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        
        # 强制将所有/lxc/路径替换为/coolpi/路径
        urdf_content = urdf_content.replace(
            "/home/lxc/ros2_ws/src/pine_harvester/assets/",
            "/home/coolpi/ros2_ws/src/pine_harvester/assets/"
        )
        
        # 将修改后的URDF写入临时文件
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
            f.write(urdf_content)
            temp_urdf_path = f.name
        
        # 加载修改后的临时URDF文件
        self.robot = placo.RobotWrapper(temp_urdf_path)
        
        # 删除临时文件
        import os
        os.unlink(temp_urdf_path)
        
        # 其余代码保持不变
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.tip_name    = tip_frame
        self.joint_names = list(IK_NAMES)
        self._task = self.solver.add_frame_task(tip_frame, np.eye(4))
        self._fk_api = self._detect_fk_api()
    
    def _detect_fk_api(self):
        for n in self.joint_names:
            self.robot.set_joint(n, 0.0)
        self.robot.update_kinematics()
        for api_name in ("get_T_world_frame", "get_transform", "get_frame"):
            fn = getattr(self.robot, api_name, None)
            if fn is not None:
                try:
                    T = fn(self.tip_name)
                    if isinstance(T, np.ndarray) and T.shape == (4, 4):
                        print(f"[ArmKinematics] FK API: robot.{api_name}() ✅")
                        return api_name
                except Exception as e:
                    print(f"[ArmKinematics] {api_name} 调用失败: {e}")
        raise RuntimeError(
            "无法找到可用的 placo FK API。"
            "已尝试: get_T_world_frame / get_transform / get_frame。"
        )

    def fk(self, angles_deg: dict) -> np.ndarray:
        for n in self.joint_names:
            self.robot.set_joint(n, float(np.deg2rad(angles_deg[n])))
        self.robot.update_kinematics()
        T = getattr(self.robot, self._fk_api)(self.tip_name)
        return np.array(T)

    def ik(self, T_target: np.ndarray, seed_deg: dict,
           pos_w: float = 1.0, ori_w: float = 0.01,
           iterations: int = 1) -> np.ndarray:
        for n in self.joint_names:
            self.robot.set_joint(n, float(np.deg2rad(seed_deg[n])))
        self._task.T_world_frame = T_target
        self._task.configure(self.tip_name, "soft", pos_w, ori_w)
        # Placo advances the solver from the current seed.  AUTO may use a
        # small, mode-specific number of passes to improve borderline IK
        # residuals; all other modes keep the original single pass.
        for _ in range(max(1, int(iterations))):
            self.solver.solve(True)
        self.robot.update_kinematics()
        return np.array([
            np.rad2deg(self.robot.get_joint(n)) for n in self.joint_names
        ])

# ════════════════════════════════════════════════════════════
# 节点
# ════════════════════════════════════════════════════════════
class AutoController:
    """AUTO autonomous controller.

    It keeps the working AUTO logic from so101_arm_interface(18).py:
    incoming pose is already in arm_base, then one IK solve writes joint target.
    """

    def __init__(self, node):
        self.node = node

    def target_pose(self, msg: PoseStamped):
        self.node._apply_target_pose(msg)

    def servo_delta(self, msg: Twist):
        dx = msg.linear.x * self.node.delta_scale
        dy = msg.linear.y * self.node.delta_scale
        dz = msg.linear.z * self.node.delta_scale
        if abs(dx) + abs(dy) + abs(dz) < 1e-5:
            return
        with self.node._lock:
            cur = dict(self.node._current)
        T = self.node._fk(cur)
        if T is None:
            return
        T = T.copy()
        T[0, 3] += dx
        T[1, 3] += dy
        T[2, 3] += dz
        result = self.node._ik(T, cur, ori_w=0.0)
        if result is not None:
            with self.node._lock:
                self.node._target = result
                self.node._last_cmd = None
            self.node._motion_done = False

    def gripper(self, msg: Bool):
        pct = self.node.gripper_close if msg.data else self.node.gripper_open
        self.node._set_gripper_target(pct)


class GestureController:
    """GESTURE controller. Gesture scaling/gripper/IK stay isolated from AUTO."""

    def __init__(self, node):
        self.node = node

    def target_pose(self, msg: PoseStamped):
        self.node._apply_gesture_target_pose(msg)

    def joint_target(self, msg: JointState):
        self.node._apply_gesture_joint_target(msg)

    def servo_delta(self, msg: Twist):
        is_zero = (
            abs(msg.linear.x) <= self.node.gesture_zero_deadband
            and abs(msg.linear.y) <= self.node.gesture_zero_deadband
            and abs(msg.linear.z) <= self.node.gesture_zero_deadband
        )
        if is_zero:
            now = time.monotonic()
            if self.node._gesture_zero_since is None:
                self.node._gesture_zero_since = now
            if (
                not self.node._gesture_zero_hold_applied
                and now - self.node._gesture_zero_since >= self.node.gesture_stop_delay
            ):
                self.node._hold_gesture_at_current("sustained zero velocity")
                self.node._gesture_zero_hold_applied = True
            return

        self.node._gesture_zero_since = None
        self.node._gesture_zero_hold_applied = False
        self.node._apply_gesture_servo_delta(msg)

    def gripper(self, msg: Bool):
        grip_command = bool(msg.data)
        if grip_command == self.node._last_gesture_grip_command:
            return
        self.node._last_gesture_grip_command = grip_command
        pct = self.node.gesture_gripper_close if msg.data else self.node.gesture_gripper_open
        self.node._set_gripper_target(pct)

    def reset(self, msg: Bool):
        if not msg.data:
            return
        self.node._hold_gesture_at_current("/gesture/reset")
        self.node._gesture_zero_since = None
        self.node._gesture_zero_hold_applied = True


class TeleopController:
    """TELEOP controller. Converts leader JointState into follower targets."""

    def __init__(self, node):
        self.node = node

    def joint_target(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.node.get_logger().warn("Invalid teleop JointState lengths")
            return
        received = dict(zip(msg.name, msg.position))
        if any(name not in received for name in MOTOR_NAMES):
            self.node.get_logger().warn("Teleop JointState is missing SO101 joints")
            return

        target = {}
        for name in IK_NAMES:
            value = float(np.rad2deg(received[name]))
            if not np.isfinite(value):
                self.node.get_logger().warn(f"Invalid teleop value for {name}")
                return
            lo, hi = JOINT_LIMITS[name]
            target[name] = max(lo, min(hi, value))

        gripper = float(received["gripper"])
        if not np.isfinite(gripper):
            self.node.get_logger().warn("Invalid teleop gripper value")
            return
        input_span = (
            self.node.teleop_gripper_input_open
            - self.node.teleop_gripper_input_close
        )
        if abs(input_span) < 1e-6:
            self.node.get_logger().error("Invalid teleop gripper input calibration")
            return
        gripper = (
            (gripper - self.node.teleop_gripper_input_close)
            * 100.0
            / input_span
        )
        lo, hi = JOINT_LIMITS["gripper"]
        target["gripper"] = max(lo, min(hi, gripper))

        with self.node._lock:
            self.node._target = target
        self.node._last_teleop_time = time.monotonic()
        self.node._motion_done = False


class EEGController:
    """EEG controller for discrete safe commands.

    The trigger command is intentionally small and mode-specific:
    MOVE_FWD_DOWN moves the current end-effector target forward and down by the
    configured offsets. It reuses the shared FK/IK execution path but does not
    change AUTO, TELEOP or GESTURE scaling/thresholds.
    """

    def __init__(self, node):
        self.node = node

    def command(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd in {"HOLD", "STOP"}:
            self.node._set_control_mode("HOLD")
        elif cmd in {"AUTO", "TELEOP", "GESTURE", "EEG", "STOW"}:
            self.node._set_control_mode(cmd)
        elif cmd in {"MOVE_FWD_DOWN", "FWD_DOWN", "FORWARD_DOWN", "TRIGGER", "PEAK"}:
            self.move_forward_down()
        elif cmd in {"OPEN", "GRIPPER_OPEN"}:
            self.node._set_gripper_target(self.node.eeg_gripper_open)
        elif cmd in {"CLOSE", "GRIPPER_CLOSE"}:
            self.node._set_gripper_target(self.node.eeg_gripper_close)
        elif cmd in {"HOME", "GO_HOME"}:
            with self.node._lock:
                self.node._target = dict(HOME_ANGLES)
                self.node._last_cmd = None
            self.node._motion_done = False
            self.node.pub_done.publish(Bool(data=False))
        else:
            self.node.get_logger().warn(f"Ignoring invalid EEG command: {cmd}")

    def move_forward_down(self):
        if self.node._control_mode != "EEG":
            self.node.get_logger().warn(
                f"Ignoring EEG movement while mode={self.node._control_mode}"
            )
            return

        now = time.monotonic()
        if now - self.node._last_eeg_motion_time < self.node.eeg_cooldown_sec:
            return

        with self.node._lock:
            cur = dict(self.node._current)
        T = self.node._fk(cur)
        if T is None:
            return

        T = T.copy()
        T[0, 3] += self.node.eeg_forward_m * self.node.eeg_x_sign
        T[2, 3] -= self.node.eeg_down_m * self.node.eeg_z_down_sign

        result = self.node._ik(T, cur)
        if result is None:
            self.node.get_logger().warn("EEG IK 失败")
            return

        with self.node._lock:
            self.node._target = result
            self.node._last_cmd = None
        self.node._last_eeg_motion_time = now
        self.node._motion_done = False
        self.node.pub_done.publish(Bool(data=False))
        self.node.get_logger().warn(
            f"EEG move: forward={self.node.eeg_forward_m:.3f}m, "
            f"down={self.node.eeg_down_m:.3f}m"
        )


class SO101ArmInterface(Node):

    @staticmethod
    def _clamp(value, min_val, max_val):
        """数值限幅，保证工作空间和夹爪端点不越界。"""
        return max(min(float(value), float(max_val)), float(min_val))

    def __init__(self):
        super().__init__("so101_arm_interface")

        # ── 参数 ✅ 优化版，防抽搐 ──────────────────────
        self.declare_parameter("port",              "/dev/ttyACM0")
        self.declare_parameter("urdf_path",         "/opt/models/so101_new_calib.urdf")
        self.declare_parameter(
            "calibration_path",
            "/home/coolpi/ros2_ws/src/pine_harvester/pine_harvester/config/drone_follower.json",
        )
        self.declare_parameter("control_hz",        40.0)
        self.declare_parameter("max_rel_target",    4.0)
        self.declare_parameter("auto_max_rel_target", 2.0)
        self.declare_parameter("auto_ik_iterations", 1)
        # 原值 7°/5% 会让主从长期落后，并让小幅手势被当作已到位。
        self.declare_parameter("body_done_tolerance_deg", 0.8)
        self.declare_parameter("gripper_done_tolerance_pct", 0.5)
        self.declare_parameter("auto_body_done_tolerance_deg", 7.0)
        self.declare_parameter("auto_gripper_done_tolerance_pct", 5.0)
        # AUTO entry pre-pose: move one wrist joint out of the camera view when
        # switching into AUTO. This is mode-local and does not change IK, TELEOP,
        # GESTURE, HOLD/STOW, or the shared driver conversion.
        self.declare_parameter("auto_entry_pose_enabled", True)
        self.declare_parameter("auto_entry_joint_name", "wrist_flex")
        self.declare_parameter("auto_entry_joint_angle_deg", -90.0)

        # AUTO 自主抓取夹爪端点：保持原语义和原默认值，避免影响视觉抓取。
        # 协议语义保持 true=闭合，false=张开。
        self.declare_parameter("gripper_close_pct", 60.0)
        self.declare_parameter("gripper_open_pct",  100.0)
        # EEG 演示夹爪端点：只作用于 EEG,CLOSE / EEG,OPEN，不影响 AUTO 或 GESTURE。
        self.declare_parameter("eeg_gripper_close_pct", 45.0)
        self.declare_parameter("eeg_gripper_open_pct", 100.0)

        # GESTURE 手势夹爪端点：只作用于 /gesture/gripper_close。
        # 你反馈手势夹爪方向反了、闭合不到位，所以默认 close=0 / open=100。
        # 如果实测仍然相反，只对调 gesture_gripper_*，不要改 AUTO 参数。
        self.declare_parameter("gesture_gripper_close_pct", 0.0)
        self.declare_parameter("gesture_gripper_open_pct",  100.0)

        # AUTO 视觉伺服继续使用 delta_scale_m。
        self.declare_parameter("delta_scale_m",     0.008)
        # GESTURE 手势速度缩放独立，避免调手势速度时影响自主视觉伺服。
        self.declare_parameter("gesture_delta_scale_m", 0.010)
        self.declare_parameter("gesture_zero_deadband", 0.02)
        # Ignore isolated neutral frames caused by hand-tracking jitter.
        self.declare_parameter("gesture_stop_delay_sec", 0.20)
        self.declare_parameter("gesture_ik_error_mm", 50.0)
        self.declare_parameter("gesture_max_target_lead_m", 0.05)
        self.declare_parameter("gesture_x_sign", 1.0)
        self.declare_parameter("gesture_y_sign", 1.0)
        self.declare_parameter("gesture_z_sign", 1.0)
        # GESTURE-only pan range; do not change the shared joint limits.
        self.declare_parameter("gesture_pan_min_deg", -72.0)
        self.declare_parameter("gesture_pan_max_deg", 110.0)
        self.declare_parameter("gesture_pan_soft_margin_deg", 10.0)
        self.declare_parameter("gesture_elbow_min_deg", -97.0)
        self.declare_parameter("gesture_elbow_max_deg", 98.0)
        self.declare_parameter("sim_mode",          False)
        self.declare_parameter("body_speed",        250)
        self.declare_parameter("gripper_speed",     250)
        self.declare_parameter("hw_hz",             30)
        self.declare_parameter("teleop_gripper_input_close", 1.52)
        self.declare_parameter("teleop_gripper_input_open", 96.97)

        # 手势速度遥控/增量控制的末端安全工作空间。
        # 防止手一直偏离基准时，末端目标持续积分到不可达区域。
        self.declare_parameter("workspace_x_min",   0.12)
        self.declare_parameter("workspace_x_max",   0.36)
        self.declare_parameter("workspace_y_min",  -0.18)
        self.declare_parameter("workspace_y_max",   0.18)
        self.declare_parameter("workspace_z_min",   0.04)
        self.declare_parameter("workspace_z_max",   0.24)
        # GESTURE 专用：默认不用“硬夹紧到工作空间边界”。
        # 原来的硬限幅在当前 FK 位姿落在工作空间外时，会把目标瞬间拉到边界，
        # 容易产生 100mm 以上 IK 误差，表现为手势有话题但机械臂不动。
        self.declare_parameter("gesture_hard_workspace_clamp", False)
        self.declare_parameter("gesture_debug_log", False)

        self.declare_parameter("teleop_timeout_sec", 0.25)
        self.declare_parameter("eeg_forward_m", 0.10)
        self.declare_parameter("eeg_down_m", 0.10)
        self.declare_parameter("eeg_cooldown_sec", 2.0)
        self.declare_parameter("eeg_x_sign", 1.0)
        self.declare_parameter("eeg_z_down_sign", 1.0)

        self.port           = self.get_parameter("port").value
        self.urdf_path      = self.get_parameter("urdf_path").value
        self.calibration_path = self.get_parameter("calibration_path").value
        self.control_hz     = self.get_parameter("control_hz").value
        self.max_rel_target = self.get_parameter("max_rel_target").value
        self.auto_max_rel_target = float(self.get_parameter("auto_max_rel_target").value)
        self.auto_ik_iterations = max(
            1, int(self.get_parameter("auto_ik_iterations").value)
        )
        self.body_done_tolerance = max(
            0.1,
            float(self.get_parameter("body_done_tolerance_deg").value),
        )
        self.gripper_done_tolerance = max(
            0.1,
            float(self.get_parameter("gripper_done_tolerance_pct").value),
        )
        self.auto_body_done_tolerance = max(
            0.1,
            float(self.get_parameter("auto_body_done_tolerance_deg").value),
        )
        self.auto_gripper_done_tolerance = max(
            0.1,
            float(self.get_parameter("auto_gripper_done_tolerance_pct").value),
        )
        self.auto_entry_pose_enabled = bool(
            self.get_parameter("auto_entry_pose_enabled").value
        )
        self.auto_entry_joint_name = str(
            self.get_parameter("auto_entry_joint_name").value
        ).strip()
        self.auto_entry_joint_angle_deg = float(
            self.get_parameter("auto_entry_joint_angle_deg").value
        )
        if self.auto_entry_joint_name not in JOINT_LIMITS:
            self.get_logger().warn(
                f"Invalid auto_entry_joint_name={self.auto_entry_joint_name}; "
                "AUTO entry pre-pose disabled"
            )
            self.auto_entry_pose_enabled = False
        self.gripper_close  = float(self.get_parameter("gripper_close_pct").value)
        self.gripper_open   = float(self.get_parameter("gripper_open_pct").value)
        self.eeg_gripper_close = float(
            self.get_parameter("eeg_gripper_close_pct").value
        )
        self.eeg_gripper_open = float(
            self.get_parameter("eeg_gripper_open_pct").value
        )
        self.gesture_gripper_close = float(
            self.get_parameter("gesture_gripper_close_pct").value
        )
        self.gesture_gripper_open = float(
            self.get_parameter("gesture_gripper_open_pct").value
        )
        self.delta_scale    = self.get_parameter("delta_scale_m").value
        self.gesture_delta_scale = float(
            self.get_parameter("gesture_delta_scale_m").value
        )
        self.gesture_zero_deadband = max(
            0.0,
            float(self.get_parameter("gesture_zero_deadband").value),
        )
        self.gesture_stop_delay = max(
            0.05,
            float(self.get_parameter("gesture_stop_delay_sec").value),
        )
        self.gesture_ik_error_mm = max(
            1.0,
            float(self.get_parameter("gesture_ik_error_mm").value),
        )
        self.gesture_max_target_lead = max(
            0.0,
            float(self.get_parameter("gesture_max_target_lead_m").value),
        )
        self.gesture_x_sign = 1.0 if float(self.get_parameter("gesture_x_sign").value) >= 0 else -1.0
        self.gesture_y_sign = 1.0 if float(self.get_parameter("gesture_y_sign").value) >= 0 else -1.0
        self.gesture_z_sign = 1.0 if float(self.get_parameter("gesture_z_sign").value) >= 0 else -1.0
        self.gesture_pan_min = float(self.get_parameter("gesture_pan_min_deg").value)
        self.gesture_pan_max = float(self.get_parameter("gesture_pan_max_deg").value)
        if self.gesture_pan_min >= self.gesture_pan_max:
            self.get_logger().warn("Invalid gesture pan range; using -72..110 deg")
            self.gesture_pan_min = -72.0
            self.gesture_pan_max = 110.0
        self.gesture_pan_soft_margin = max(
            0.1, float(self.get_parameter("gesture_pan_soft_margin_deg").value)
        )
        self.gesture_elbow_min = float(
            self.get_parameter("gesture_elbow_min_deg").value
        )
        self.gesture_elbow_max = float(
            self.get_parameter("gesture_elbow_max_deg").value
        )
        if self.gesture_elbow_min >= self.gesture_elbow_max:
            self.get_logger().warn("Invalid gesture elbow range; using -97..98 deg")
            self.gesture_elbow_min = -97.0
            self.gesture_elbow_max = 98.0
        self.sim_mode       = self.get_parameter("sim_mode").value
        self.body_speed     = int(self.get_parameter("body_speed").value)
        self.gripper_speed  = int(self.get_parameter("gripper_speed").value)
        self.hw_hz          = int(self.get_parameter("hw_hz").value)
        self.teleop_gripper_input_close = float(
            self.get_parameter("teleop_gripper_input_close").value
        )
        self.teleop_gripper_input_open = float(
            self.get_parameter("teleop_gripper_input_open").value
        )

        self.workspace_x_min = float(self.get_parameter("workspace_x_min").value)
        self.workspace_x_max = float(self.get_parameter("workspace_x_max").value)
        self.workspace_y_min = float(self.get_parameter("workspace_y_min").value)
        self.workspace_y_max = float(self.get_parameter("workspace_y_max").value)
        self.workspace_z_min = float(self.get_parameter("workspace_z_min").value)
        self.workspace_z_max = float(self.get_parameter("workspace_z_max").value)
        self.gesture_hard_workspace_clamp = bool(
            self.get_parameter("gesture_hard_workspace_clamp").value
        )
        self.gesture_debug_log = bool(
            self.get_parameter("gesture_debug_log").value
        )

        self.teleop_timeout = max(
            0.1, float(self.get_parameter("teleop_timeout_sec").value)
        )
        self.eeg_forward_m = max(
            0.0, float(self.get_parameter("eeg_forward_m").value)
        )
        self.eeg_down_m = max(
            0.0, float(self.get_parameter("eeg_down_m").value)
        )
        self.eeg_cooldown_sec = max(
            0.1, float(self.get_parameter("eeg_cooldown_sec").value)
        )
        self.eeg_x_sign = 1.0 if float(self.get_parameter("eeg_x_sign").value) >= 0 else -1.0
        self.eeg_z_down_sign = 1.0 if float(self.get_parameter("eeg_z_down_sign").value) >= 0 else -1.0

        # ── 共享状态 ─────────────────────────────────────
        self._current     = dict(HOME_ANGLES)
        self._target      = dict(HOME_ANGLES)
        self._motion_done = True
        self._hw_enabled  = True
        self._lock        = threading.Lock()
        self._last_cmd    = None
        self._control_mode = "HOLD"
        self._last_teleop_time = None
        # 只给 GESTURE 路径使用的目标缓存。AUTO 和 TELEOP 不读取它。
        self._gesture_target = None
        self._gesture_joint_lock = None
        self._gesture_pan_input_offset = 0.0
        self._gesture_pan_applied_target = None
        self._gesture_last_valid_pan = None
        self._gesture_pan_spike_threshold = 118.0
        self._gesture_zero_since = None
        self._gesture_zero_hold_applied = False
        self._last_gesture_grip_command = None
        self._last_eeg_motion_time = 0.0
        self._last_auto_ik_success = None

        # Mode controllers: one hardware interface, isolated mode logic.
        self.auto_controller = AutoController(self)
        self.gesture_controller = GestureController(self)
        self.teleop_controller = TeleopController(self)
        self.eeg_controller = EEGController(self)

        self._cmd_queue: queue.Queue = queue.Queue(maxsize=3)
        self._hw_running = False

        self.arm = None
        self.kin = None

        # ── 硬件初始化 ───────────────────────────────────
        if not self.sim_mode:
            connected = self._connect_arm()
            self._init_kinematics()
            if connected:
                self._start_hw_thread()
            else:
                self._hw_enabled = False
                self.get_logger().error(
                    "SO101 硬件未连接，已禁止启动 IO 线程和运动指令"
                )
        else:
            self.get_logger().warn("SO101: 仿真模式（无硬件）")

        # ── 订阅 / 发布 ──────────────────────────────────
        self.create_subscription(PoseStamped, "/arm/target_pose", self._cb_target_pose, 10)
        self.create_subscription(Bool,        "/arm/go_home",     self._cb_go_home,     10)
        self.create_subscription(Twist,       "/servo/arm_delta", self._cb_servo_delta,  10)
        self.create_subscription(Bool,        "/gripper/close",   self._cb_gripper,      10)
        self.create_subscription(
            PoseStamped,
            "/gesture/target_pose",
            self._cb_gesture_target_pose,
            10,
        )
        self.create_subscription(
            JointState,
            "/gesture/joint_target",
            self._cb_gesture_joint_target,
            10,
        )
        self.create_subscription(
            Twist,
            "/gesture/arm_delta",
            self._cb_gesture_servo_delta,
            10,
        )
        self.create_subscription(
            Bool,
            "/gesture/gripper_close",
            self._cb_gesture_gripper,
            10,
        )
        self.create_subscription(
            Bool,
            "/gesture/reset",
            self._cb_gesture_reset,
            10,
        )
        self.create_subscription(Bool,        "/arm/enable_hw",   self._cb_hw_enable,    10)
        self.create_subscription(String,      "/control/mode", self._cb_control_mode, 10)
        self.create_subscription(String,      "/eeg/command", self._cb_eeg_command, 10)
        self.create_subscription(
            JointState,
            "/arm/teleop_joint_target",
            self._cb_teleop_joint_target,
            1,
        )

        self.pub_ee     = self.create_publisher(PoseStamped, "/arm/end_effector_pose", 10)
        self.pub_joints = self.create_publisher(JointState,  "/arm/joint_states",      10)
        self.pub_done   = self.create_publisher(Bool,        "/arm/motion_done",       10)
        self.pub_status = self.create_publisher(String,      "/arm/status",            10)

        # ✅ 直接启动控制环，无延迟
        self.create_timer(1.0 / self.control_hz, self._control_loop)
        self.create_timer(0.1, self._publish_state)
        self.create_timer(0.05, self._check_teleop_watchdog)

        self.get_logger().info(
            f"SO101ArmInterface 就绪 | "
            f"{'仿真' if self.sim_mode else '真实硬件'} | "
            f"port={self.port} | IK={'OK' if self.kin else '不可用'} | hw_hz={self.hw_hz} | "
            f"auto_gripper(close={self.gripper_close:.1f}, open={self.gripper_open:.1f}) | "
            f"gesture_gripper(close={self.gesture_gripper_close:.1f}, open={self.gesture_gripper_open:.1f})"
        )
        # ✅ 启动就打印，确认代码是最新的
        print("[✅ 代码已加载] 机械臂节点初始化完成，控制环已启动")

    def _connect_arm(self):
        try:
            class _Cfg:
                pass
            cfg = _Cfg()
            cfg.port          = self._resolve_arm_port(self.port)
            self.port         = cfg.port
            cfg.body_speed    = self.body_speed
            cfg.gripper_speed = self.gripper_speed
            cfg.calibration_path = self.calibration_path
            self.arm = SO101Follower(cfg)
            self.arm.connect()

            # ✅ 等待硬件角度同步（最多5s）
            self.get_logger().info("正在同步硬件角度...")
            timeout = 5.0
            start = time.time()
            sync_success = False
            while time.time() - start < timeout:
                obs = self.arm.get_observation()
                if obs and all(f"{n}.pos" in obs for n in MOTOR_NAMES):
                    with self._lock:
                        for name in MOTOR_NAMES:
                            self._current[name] = float(obs[f"{name}.pos"])
                            self._target[name] = float(obs[f"{name}.pos"])
                    sync_success = True
                    self.get_logger().info(f"✅ 硬件角度同步完成: {self._current}")
                    
                    # ✅ 强制发送一次当前角度，覆盖驱动乱码
                    self.arm.set_action(self._current)
                    time.sleep(0.5)
                    break
                time.sleep(0.1)
            
            if not sync_success:
                self.get_logger().error("❌ 硬件角度同步超时！切换到仿真模式")
                try:
                    self.arm.disconnect()
                except Exception:
                    pass
                self.arm = None
                return False

            return True

        except Exception as e:
            self.get_logger().error(f"SO101 连接失败: {e}")
            try:
                if self.arm is not None:
                    self.arm.disconnect()
            except Exception:
                pass
            self.arm = None
            return False

    def _resolve_arm_port(self, configured_port: str) -> str:
        """Resolve SO101 follower port while never scanning telemetry ttyUSB.

        The arm is a CDC/ACM device on this setup.  After reboot/replug Linux
        may enumerate it as /dev/ttyACM0 or /dev/ttyACM1.  The telemetry radio
        is normally /dev/ttyUSB*, so ttyUSB is intentionally excluded here.
        """
        configured_port = str(configured_port or "").strip()
        candidates = []

        if configured_port and configured_port.lower() not in {"auto", "none"}:
            candidates.append(configured_port)

        for by_id_port in sorted(glob.glob("/dev/serial/by-id/*")):
            if os.path.basename(os.path.realpath(by_id_port)).startswith("ttyACM"):
                candidates.append(by_id_port)
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))

        seen = set()
        unique_candidates = []
        for port in candidates:
            if port and port not in seen:
                seen.add(port)
                unique_candidates.append(port)

        for port in unique_candidates:
            if os.path.exists(port):
                if port != configured_port:
                    self.get_logger().warn(
                        f"SO101 arm port fallback: {configured_port} -> {port}"
                    )
                return port

        raise FileNotFoundError(
            "SO101 arm serial port not found; tried: "
            + ", ".join(unique_candidates or [configured_port, "/dev/ttyACM*"])
        )

    def _init_kinematics(self):
        if not PLACO_OK:
            self.get_logger().error("placo 未安装: pip install placo")
            return
        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF 不存在: {self.urdf_path}")
            return
        try:
            self.kin = ArmKinematics(self.urdf_path)
            T0 = self.kin.fk(HOME_ANGLES)
            self.get_logger().info(
                f"placo 运动学就绪 ✅ | "
                f"零位末端: X={T0[0,3]:.3f}m Y={T0[1,3]:.3f}m Z={T0[2,3]:.3f}m"
            )
        except Exception as e:
            self.get_logger().error(f"placo 初始化失败: {e}")

    def _start_hw_thread(self):
        if self.arm is None:
            self.get_logger().error("SO101 硬件未连接，跳过 IO 线程启动")
            return
        self._hw_running = True
        t = threading.Thread(target=self._hw_worker, name="hw_io", daemon=True)
        t.start()
        self.get_logger().info(f"硬件 IO 线程已启动 ({self.hw_hz}Hz)")

    def _hw_worker(self):
        last_read_time = 0
        last_sent_cmd = None
        cmd_send_interval = 1.0 / self.hw_hz
        last_cmd_send_time = 0

        while self._hw_running:
            now = time.time()

            # 1. 读取硬件状态（按频率）
            if now - last_read_time > 0.03:
                try:
                    obs = self.arm.get_observation()
                    if obs:
                        with self._lock:
                            for name in MOTOR_NAMES:
                                v = obs.get(f"{name}.pos")
                                if v is not None:
                                    value = float(v)
                                    if name == "shoulder_pan":
                                        if value < self._gesture_pan_spike_threshold:
                                            self._gesture_last_valid_pan = value
                                        elif self._control_mode == "GESTURE":
                                            continue
                                    self._current[name] = value
                    last_read_time = now
                except Exception as e:
                    self.get_logger().error(f"读取硬件状态失败: {e}", throttle_duration_sec=5.0)

            # 2. 发送指令（按hw_hz频率）
            if now - last_cmd_send_time > cmd_send_interval:
                cmd = None
                try:
                    cmd = self._cmd_queue.get_nowait()
                except queue.Empty:
                    pass

                if cmd is not None and cmd != last_sent_cmd:
                    try:
                        self.arm.set_action(cmd)
                        last_sent_cmd = cmd
                        last_cmd_send_time = now
                    except Exception as e:
                        self.get_logger().error(f"set_action 异常: {e}", throttle_duration_sec=5.0)

            time.sleep(0.005)

    def _fk(self, angles: dict):
        if self.kin is None:
            return None
        try:
            return self.kin.fk(angles)
        except Exception as e:
            self.get_logger().error(f"FK 计算失败: {e}", throttle_duration_sec=5.0)
            return None

    def _ik(self, T_target: np.ndarray, current: dict, ori_w: float = 0.0001):
        if self.kin is None:
            return None
        try:
            T_seed = self._fk(current)
            if T_seed is not None:
                seed_err_mm = np.linalg.norm(T_seed[:3, 3] - T_target[:3, 3]) * 1000
                if seed_err_mm <= 10.0:
                    return dict(current)

            if self._control_mode == "AUTO":
                return self._ik_auto_multi_seed(T_target, current, ori_w)

            iterations = 1
            q = self.kin.ik(
                T_target,
                current,
                pos_w=1.0,
                ori_w=float(ori_w),
                iterations=iterations,
            )
            result = dict(current)
            for i, name in enumerate(IK_NAMES):
                result[name] = float(q[i])

            for name in IK_NAMES:
                lo, hi = JOINT_LIMITS[name]
                if not (lo <= result[name] <= hi):
                    self.get_logger().warn(f"IK 结果超限: {name}={result[name]:.1f}°")
                    return None

            T_check = self._fk(result)
            if T_check is not None:
                err_mm = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3]) * 1000
                if err_mm > 50:
                    self.get_logger().warn(f"IK 收敛误差 {err_mm:.1f}mm，目标不可达")
                    return None
            return result
        except Exception as e:
            self.get_logger().warn(f"IK 求解异常: {e}")
            return None

    def _ik_auto_multi_seed(self, T_target: np.ndarray, current: dict, ori_w: float):
        seeds = [current]
        for seed in (self._last_auto_ik_success, HOME_ANGLES):
            if seed is not None:
                seeds.append(seed)

        for joint_name, delta_deg in (
            ("elbow_flex", 20.0),
            ("elbow_flex", -20.0),
        ):
            seed = dict(current)
            lo, hi = JOINT_LIMITS[joint_name]
            seed[joint_name] = self._clamp(seed[joint_name] + delta_deg, lo, hi)
            seeds.append(seed)

        best_result = None
        best_err_mm = float("inf")
        best_limit_msg = None
        seen = set()

        for seed in seeds:
            key = tuple(round(float(seed[name]), 3) for name in IK_NAMES)
            if key in seen:
                continue
            seen.add(key)

            try:
                q = self.kin.ik(
                    T_target,
                    seed,
                    pos_w=1.0,
                    ori_w=float(ori_w),
                    iterations=1,
                )
            except Exception as exc:
                self.get_logger().warn(f"AUTO IK seed solve failed: {exc}")
                continue

            result = dict(current)
            for i, name in enumerate(IK_NAMES):
                result[name] = float(q[i])

            limit_msg = None
            for name in IK_NAMES:
                lo, hi = JOINT_LIMITS[name]
                if not (lo <= result[name] <= hi):
                    limit_msg = f"IK result out of limit: {name}={result[name]:.1f}deg"
                    break
            if limit_msg is not None:
                best_limit_msg = best_limit_msg or limit_msg
                continue

            T_check = self._fk(result)
            if T_check is None:
                continue
            err_mm = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3]) * 1000
            if err_mm < best_err_mm:
                best_err_mm = err_mm
                best_result = result
                if err_mm <= 50:
                    break

        if best_result is None:
            if best_limit_msg is not None:
                self.get_logger().warn(best_limit_msg)
            return None

        if best_err_mm > 50:
            self.get_logger().warn(f"IK converged with {best_err_mm:.1f}mm error; target unreachable")
            return None

        self._last_auto_ik_success = dict(best_result)
        return best_result

    def _ik_gesture(self, T_target: np.ndarray, seed_deg: dict, warn: bool = True):
        """GESTURE 专用 IK：不改 AUTO 使用的 _ik() 误差门限和方向逻辑。"""
        if self.kin is None:
            return None
        try:
            q = self.kin.ik(T_target, seed_deg, pos_w=1.0, ori_w=0.0001)
            result = dict(seed_deg)
            for i, name in enumerate(IK_NAMES):
                result[name] = float(q[i])

            for name in IK_NAMES:
                lo, hi = JOINT_LIMITS[name]
                if not (lo <= result[name] <= hi):
                    if warn:
                        self.get_logger().warn(
                            f"GESTURE IK 结果超限: {name}={result[name]:.1f}°"
                        )
                    return None

            T_check = self._fk(result)
            if T_check is not None:
                err_mm = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3]) * 1000
                if err_mm > self.gesture_ik_error_mm:
                    if warn:
                        self.get_logger().warn(
                            f"GESTURE IK 收敛误差 {err_mm:.1f}mm，目标不可达"
                        )
                    return None
            return result
        except Exception as e:
            if warn:
                self.get_logger().warn(f"GESTURE IK 求解异常: {e}")
            return None

    def _ik_gesture_or_nearest_on_ray(self, T_target: np.ndarray, seed_deg: dict):
        """GESTURE only: if target pose is unreachable, move toward it as far as IK allows."""
        result = self._ik_gesture(T_target, seed_deg, warn=True)
        if result is not None:
            return result

        T_seed = self._fk(seed_deg)
        if T_seed is None:
            return None
        delta = T_target[:3, 3] - T_seed[:3, 3]
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return None

        best_result = None
        best_fraction = 0.0
        for fraction in (0.85, 0.70, 0.55, 0.40, 0.25, 0.12):
            T_probe = T_target.copy()
            T_probe[:3, 3] = T_seed[:3, 3] + delta * fraction
            probe = self._ik_gesture(T_probe, seed_deg, warn=False)
            if probe is not None:
                best_result = probe
                best_fraction = fraction
                break

        if best_result is not None:
            self.get_logger().warn(
                f"GESTURE target projected to nearest reachable ray point "
                f"({best_fraction * 100:.0f}% of requested move)",
                throttle_duration_sec=0.5,
            )
            return best_result
        return None

    def _control_loop(self):
        # ✅ 强制打印，现在100%会执行
        with self._lock:
            tgt = dict(self._target)
            cur = (
                self._gesture_current_without_pan_spike()
                if self._control_mode == "GESTURE"
                else dict(self._current)
            )
        # print(f"[强制打印] 当前角度: {cur['shoulder_pan']:.1f} | 目标角度: {tgt['shoulder_pan']:.1f}")

        # 误差
        body_err = max(abs(tgt[n] - cur[n]) for n in IK_NAMES)
        grip_err = abs(tgt["gripper"] - cur["gripper"])

        if self._control_mode == "AUTO":
            body_done_tolerance = self.auto_body_done_tolerance
            gripper_done_tolerance = self.auto_gripper_done_tolerance
            mode_max_step = self.auto_max_rel_target
        else:
            body_done_tolerance = self.body_done_tolerance
            gripper_done_tolerance = self.gripper_done_tolerance
            mode_max_step = self.max_rel_target

        if (
            body_err < body_done_tolerance
            and grip_err < gripper_done_tolerance
        ):
            if not self._motion_done:
                self._motion_done = True
                try:
                    self._cmd_queue.put_nowait(tgt)
                except:
                    pass
                self._last_cmd = dict(tgt)
                self.pub_done.publish(Bool(data=True))
                self.get_logger().info(f"Motion stable ✅ err={body_err:.2f}")
            return

        self._motion_done = False

        if self.sim_mode:
            with self._lock:
                self._current = dict(tgt)
            return

        # 用 last_cmd 推进
        step = dict(self._last_cmd) if self._last_cmd else dict(cur)
        for n in IK_NAMES:
            d = tgt[n] - step[n]
            max_step = mode_max_step
            if abs(d) < max_step:
                step[n] = tgt[n]
            else:
                step[n] = step[n] + max_step * (1 if d > 0 else -1)

        # gripper
        gd = tgt["gripper"] - step["gripper"]
        if abs(gd) < mode_max_step * 2:
            step["gripper"] = tgt["gripper"]
        else:
            step["gripper"] = step["gripper"] + mode_max_step * 2 * (1 if gd > 0 else -1)

        # 限幅
        for n in MOTOR_NAMES:
            lo, hi = JOINT_LIMITS[n]
            step[n] = max(lo, min(hi, step[n]))

        # 发送
        try:
            self._cmd_queue.put_nowait(step)
        except queue.Full:
            try:
                self._cmd_queue.get_nowait()
            except:
                pass
            self._cmd_queue.put_nowait(step)

        self._last_cmd = dict(step)
			
    def _publish_state(self):
        with self._lock:
            cur = dict(self._current)

        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name     = MOTOR_NAMES
        # ROS body joints are radians; the SO-101 gripper remains 0..100 percent.
        js.position = [float(np.deg2rad(cur[n])) for n in IK_NAMES]
        js.position.append(float(cur["gripper"]))
        self.pub_joints.publish(js)
        self.pub_status.publish(String(data=self._control_mode))

        T = self._fk(cur)
        if T is not None:
            ee = PoseStamped()
            ee.header.stamp    = stamp
            ee.header.frame_id = "arm_base"
            ee.pose.position   = Point(x=float(T[0,3]), y=float(T[1,3]), z=float(T[2,3]))
            if SCIPY_OK:
                qx, qy, qz, qw = SciR.from_matrix(T[:3, :3]).as_quat()
                ee.pose.orientation.x = float(qx)
                ee.pose.orientation.y = float(qy)
                ee.pose.orientation.z = float(qz)
                ee.pose.orientation.w = float(qw)
            self.pub_ee.publish(ee)

    def _cb_target_pose(self, msg: PoseStamped):
        if not self._hw_enabled or self._control_mode != "AUTO":
            return
        self.auto_controller.target_pose(msg)

    def _cb_gesture_target_pose(self, msg: PoseStamped):
        if not self._hw_enabled or self._control_mode != "GESTURE":
            return
        self.gesture_controller.target_pose(msg)

    def _cb_gesture_joint_target(self, msg: JointState):
        if not self._hw_enabled or self._control_mode != "GESTURE":
            return
        self.gesture_controller.joint_target(msg)

    def _pose_msg_to_matrix(self, msg: PoseStamped) -> np.ndarray:
        p, q = msg.pose.position, msg.pose.orientation
        T = np.eye(4)
        T[:3, 3] = [p.x, p.y, p.z]
        if SCIPY_OK:
            T[:3, :3] = SciR.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return T

    def _apply_target_pose(self, msg: PoseStamped):
        # AUTO 自主抓取专用：只使用视觉 position。
        # orientation 沿用当前末端姿态，避免状态机固定姿态导致倒挂抓取 IK 不可达。
        T = self._pose_msg_to_matrix(msg)

        with self._lock:
            cur = dict(self._current)
        T_cur = self._fk(cur)
        if T_cur is not None:
            T[:3, :3] = T_cur[:3, :3]
        self.get_logger().info(
            f"AUTO target position-only IK: "
            f"x={T[0,3]:.3f}, y={T[1,3]:.3f}, z={T[2,3]:.3f}",
            throttle_duration_sec=1.0,
        )
        result = self._ik(T, cur, ori_w=0.00001)
        if result is None:
            self.get_logger().warn(f"IK 失败")
            return

        with self._lock:
            self._target  = result
            self._last_cmd = None
        self._motion_done = False
        self.pub_done.publish(Bool(data=False))

    def _apply_gesture_target_pose(self, msg: PoseStamped):
        # GESTURE 绝对位姿入口：使用手势专用 IK，避免调整手势误差门限时影响 AUTO。
        T = self._pose_msg_to_matrix(msg)
        T[0, 3] = self._clamp(T[0, 3], self.workspace_x_min, self.workspace_x_max)
        T[1, 3] = self._clamp(T[1, 3], self.workspace_y_min, self.workspace_y_max)
        T[2, 3] = self._clamp(T[2, 3], self.workspace_z_min, self.workspace_z_max)

        with self._lock:
            cur = dict(self._current)
        result = self._ik_gesture_or_nearest_on_ray(T, cur)
        if result is None:
            self.get_logger().warn("GESTURE IK 失败")
            return

        with self._lock:
            self._gesture_target = dict(result)
            self._target = result
            self._last_cmd = None
        self._motion_done = False
        self.pub_done.publish(Bool(data=False))

    def _gesture_pan_soft_step(self, current: float, delta: float):
        """Slow only outward pan motion near GESTURE limits."""
        lo = self.gesture_pan_min
        hi = self.gesture_pan_max
        margin = self.gesture_pan_soft_margin
        if abs(delta) < 1e-9:
            return current, 1.0
        if current > hi:
            return (current, 0.0) if delta > 0.0 else (max(hi, current + delta), 1.0)
        if current < lo:
            return (current, 0.0) if delta < 0.0 else (min(lo, current + delta), 1.0)
        scale = 1.0
        if delta > 0.0 and current > hi - margin:
            scale = max(0.0, min(1.0, (hi - current) / margin))
        elif delta < 0.0 and current < lo + margin:
            scale = max(0.0, min(1.0, (current - lo) / margin))
        return self._clamp(current + delta * scale, lo, hi), scale

    def _apply_gesture_joint_target(self, msg: JointState):
        """Apply only GESTURE joint targets; joint1 is a cumulative offset."""
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("Invalid gesture JointState lengths")
            return
        received = dict(zip(msg.name, msg.position))
        if not any(
            name in received
            for name in (
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "elbow_flex_offset",
                "wrist_flex",
                "wrist_flex_offset",
            )
        ):
            self.get_logger().warn("Gesture JointState contains no supported joints")
            return

        values = {}
        for name in ("shoulder_lift", "elbow_flex", "wrist_flex"):
            if name not in received:
                continue
            value = float(np.rad2deg(received[name]))
            if not np.isfinite(value):
                self.get_logger().warn(f"Invalid gesture target for {name}")
                return
            lo, hi = JOINT_LIMITS[name]
            values[name] = self._clamp(value, lo, hi)

        pan_offset = None
        if "shoulder_pan" in received:
            pan_offset = float(np.rad2deg(received["shoulder_pan"]))
            if not np.isfinite(pan_offset):
                self.get_logger().warn("Invalid gesture pan offset")
                return
        wrist_offset = None
        if "wrist_flex_offset" in received:
            wrist_offset = float(np.rad2deg(received["wrist_flex_offset"]))
            if not np.isfinite(wrist_offset):
                self.get_logger().warn("Invalid gesture wrist offset")
                return
        elbow_offset = None
        if "elbow_flex_offset" in received:
            elbow_offset = float(np.rad2deg(received["elbow_flex_offset"]))
            if not np.isfinite(elbow_offset):
                self.get_logger().warn("Invalid gesture elbow offset")
                return

        with self._lock:
            if self._gesture_joint_lock is None:
                self._gesture_joint_lock = self._gesture_current_without_pan_spike()
            pan_target = self._gesture_pan_applied_target
            input_step = 0.0
            edge_scale = 1.0
            pan_base = float(
                self._gesture_joint_lock.get("shoulder_pan", HOME_ANGLES["shoulder_pan"])
            )
            if pan_offset is not None:
                desired_pan = pan_base + pan_offset
                pan_target = self._clamp(
                    desired_pan,
                    self.gesture_pan_min,
                    self.gesture_pan_max,
                )
                input_step = pan_target - (
                    self._gesture_pan_applied_target
                    if self._gesture_pan_applied_target is not None
                    else pan_base
                )
                self._gesture_pan_input_offset = pan_offset
                if abs(desired_pan - pan_target) > 1e-6:
                    edge_scale = 0.0
                self._gesture_pan_applied_target = pan_target
                values["shoulder_pan"] = pan_target
            elif self._gesture_pan_applied_target is None:
                self._gesture_pan_applied_target = pan_base
                pan_target = self._gesture_pan_applied_target

            target = dict(self._gesture_joint_lock)
            if elbow_offset is not None:
                elbow_base = float(
                    self._gesture_joint_lock.get("elbow_flex", HOME_ANGLES["elbow_flex"])
                )
                values["elbow_flex"] = self._clamp(
                    elbow_base + elbow_offset,
                    self.gesture_elbow_min,
                    self.gesture_elbow_max,
                )
            if wrist_offset is not None:
                wrist_base = float(
                    self._gesture_joint_lock.get("wrist_flex", HOME_ANGLES["wrist_flex"])
                )
                lo, hi = JOINT_LIMITS["wrist_flex"]
                values["wrist_flex"] = self._clamp(wrist_base + wrist_offset, lo, hi)
            target.update(values)
            target["gripper"] = self._target.get(
                "gripper", self._current.get("gripper", HOME_ANGLES["gripper"])
            )
            self._gesture_target = dict(target)
            self._target = target
            self._last_cmd = None
            cur_pan = self._gesture_current_without_pan_spike().get("shoulder_pan", 0.0)
            cur_elbow = self._current.get("elbow_flex", 0.0)
            elbow_apply = values.get("elbow_flex", target.get("elbow_flex", cur_elbow))

        self.get_logger().info(
            f"GESTURE joint: pan_offset={0.0 if pan_offset is None else pan_offset:+.2f} "
            f"pan_base={pan_base:+.2f} pan_step={input_step:+.2f} "
            f"pan_apply={pan_target:+.2f} "
            f"elbow_offset={0.0 if elbow_offset is None else elbow_offset:+.2f} "
            f"elbow_apply={elbow_apply:+.2f} "
            f"wrist_offset={0.0 if wrist_offset is None else wrist_offset:+.2f} "
            f"current_pan={cur_pan:+.2f} current_elbow={cur_elbow:+.2f} "
            f"edge={edge_scale:.2f}",
            throttle_duration_sec=0.5,
        )
        self._motion_done = False
        self.pub_done.publish(Bool(data=False))

    def _cb_go_home(self, msg: Bool):
        if not msg.data or self._control_mode != "AUTO":
            return
        self.get_logger().info("回零位")
        with self._lock:
            self._target   = dict(HOME_ANGLES)
            self._last_cmd = None
        self._motion_done = False
        self.pub_done.publish(Bool(data=False))

    def _cb_gripper(self, msg: Bool):
        if not self._hw_enabled or self._control_mode != "AUTO":
            return
        self.auto_controller.gripper(msg)

    def _cb_gesture_gripper(self, msg: Bool):
        if not self._hw_enabled or self._control_mode != "GESTURE":
            return
        self.gesture_controller.gripper(msg)

    def _set_gripper_target(self, pct: float):
        pct = self._clamp(pct, JOINT_LIMITS["gripper"][0], JOINT_LIMITS["gripper"][1])
        with self._lock:
            self._target["gripper"] = pct
            if self._control_mode == "GESTURE" and self._gesture_target is not None:
                self._gesture_target["gripper"] = pct
            self._last_cmd = None
        self._motion_done = False

    def _apply_auto_gripper(self, msg: Bool):
        # AUTO 自主抓取夹爪：使用 gripper_* 参数，默认保持旧逻辑 close=75 / open=0。
        pct = self.gripper_close if msg.data else self.gripper_open
        self._set_gripper_target(pct)

    def _apply_gesture_gripper(self, msg: Bool):
        # GESTURE 手势夹爪：只使用 gesture_gripper_*，修手势不影响 AUTO。
        grip_command = bool(msg.data)
        if grip_command == self._last_gesture_grip_command:
            return
        self._last_gesture_grip_command = grip_command
        pct = self.gesture_gripper_close if msg.data else self.gesture_gripper_open
        self._set_gripper_target(pct)

    def _cb_servo_delta(self, msg: Twist):
        if not self._hw_enabled or self._control_mode != "AUTO":
            return
        self.auto_controller.servo_delta(msg)

    def _cb_gesture_servo_delta(self, msg: Twist):
        if not self._hw_enabled or self._control_mode != "GESTURE":
            return
        self.gesture_controller.servo_delta(msg)

    def _apply_auto_servo_delta(self, msg: Twist):
        # AUTO 视觉伺服：保留原始行为，从当前实测姿态加 delta，使用 delta_scale_m 和全局 _ik()。
        dx = msg.linear.x * self.delta_scale
        dy = msg.linear.y * self.delta_scale
        dz = msg.linear.z * self.delta_scale
        if abs(dx) + abs(dy) + abs(dz) < 1e-5:
            return
        with self._lock:
            cur = dict(self._current)
        T = self._fk(cur)
        if T is None:
            return
        T = T.copy()
        T[0, 3] += dx
        T[1, 3] += dy
        T[2, 3] += dz
        result = self._ik(T, cur, ori_w=0.00001)
        if result is not None:
            with self._lock:
                self._target = result
                self._last_cmd = None
            self._motion_done = False

    def _get_gesture_seed(self):
        # Always seed velocity control from the latest measured pose.  The old
        # implementation reused the previous gesture target; after one IK
        # rejection that stale target stayed tens of millimetres ahead, so
        # reverse commands could not bring the arm back.
        with self._lock:
            seed = dict(self._current)
            self._gesture_target = dict(seed)
        return seed

    def _limit_gesture_target_lead(self, T_target: np.ndarray) -> np.ndarray:
        if self.gesture_max_target_lead <= 0.0:
            return T_target
        with self._lock:
            cur = dict(self._current)
        T_cur = self._fk(cur)
        if T_cur is None:
            return T_target
        diff = T_target[:3, 3] - T_cur[:3, 3]
        dist = float(np.linalg.norm(diff))
        if dist <= self.gesture_max_target_lead or dist < 1e-9:
            return T_target
        limited = T_target.copy()
        limited[:3, 3] = T_cur[:3, 3] + diff * (self.gesture_max_target_lead / dist)
        self.get_logger().warn(
            f"GESTURE target lead limited: {dist * 1000:.1f}mm -> "
            f"{self.gesture_max_target_lead * 1000:.1f}mm",
            throttle_duration_sec=1.0,
        )
        return limited

    def _gesture_soft_workspace_axis(self, current, proposed, lower, upper):
        """GESTURE 专用软工作空间限制。

        旧逻辑是直接 clamp(proposed, lower, upper)。如果当前 FK 位姿已经在
        工作空间外，第一条手势指令会让目标瞬间跳到边界，IK 会报 100mm+
        收敛误差。这里改为：
        - 当前在范围内：正常限幅；
        - 当前低于下限：只允许继续向上限方向运动，不允许更低；
        - 当前高于上限：只允许继续向下限方向运动，不允许更高。
        这样不会“瞬移”到边界，也不会让手势继续把机械臂推离安全范围。
        """
        current = float(current)
        proposed = float(proposed)
        lower = float(lower)
        upper = float(upper)
        if self.gesture_hard_workspace_clamp:
            return self._clamp(proposed, lower, upper)
        if lower <= current <= upper:
            return self._clamp(proposed, lower, upper)
        if current < lower:
            if proposed <= current:
                return current
            return min(proposed, lower)
        if current > upper:
            if proposed >= current:
                return current
            return max(proposed, upper)
        return proposed

    def _apply_gesture_workspace_limit(self, T: np.ndarray) -> np.ndarray:
        limited = T.copy()
        limited[0, 3] = self._gesture_soft_workspace_axis(
            T[0, 3], T[0, 3], self.workspace_x_min, self.workspace_x_max
        )
        limited[1, 3] = self._gesture_soft_workspace_axis(
            T[1, 3], T[1, 3], self.workspace_y_min, self.workspace_y_max
        )
        limited[2, 3] = self._gesture_soft_workspace_axis(
            T[2, 3], T[2, 3], self.workspace_z_min, self.workspace_z_max
        )
        return limited

    def _apply_gesture_servo_delta(self, msg: Twist):
        # GESTURE 独立速度缩放和方向符号。调这些参数不会改变 AUTO 视觉伺服。
        dx = msg.linear.x * self.gesture_delta_scale * self.gesture_x_sign
        dy = msg.linear.y * self.gesture_delta_scale * self.gesture_y_sign
        dz = msg.linear.z * self.gesture_delta_scale * self.gesture_z_sign

        seed = self._get_gesture_seed()
        T_seed = self._fk(seed)
        if T_seed is None:
            return
        T = T_seed.copy()

        proposed_x = T_seed[0, 3] + dx
        proposed_y = T_seed[1, 3] + dy
        proposed_z = T_seed[2, 3] + dz
        T[0, 3] = self._gesture_soft_workspace_axis(
            T_seed[0, 3], proposed_x, self.workspace_x_min, self.workspace_x_max
        )
        T[1, 3] = self._gesture_soft_workspace_axis(
            T_seed[1, 3], proposed_y, self.workspace_y_min, self.workspace_y_max
        )
        T[2, 3] = self._gesture_soft_workspace_axis(
            T_seed[2, 3], proposed_z, self.workspace_z_min, self.workspace_z_max
        )
        T = self._limit_gesture_target_lead(T)

        if self.gesture_debug_log:
            self.get_logger().info(
                f"GESTURE target xyz: "
                f"seed=({T_seed[0,3]:.3f},{T_seed[1,3]:.3f},{T_seed[2,3]:.3f}) "
                f"delta=({dx:.4f},{dy:.4f},{dz:.4f}) "
                f"target=({T[0,3]:.3f},{T[1,3]:.3f},{T[2,3]:.3f})",
                throttle_duration_sec=0.5,
            )

        result = self._ik_gesture(T, seed)
        if result is not None:
            with self._lock:
                self._gesture_target = dict(result)
                self._target = result
                self._last_cmd = None
            self._motion_done = False
        else:
            # Rejecting IK must also discard the failed Cartesian target.
            # Otherwise every later command, including the reverse direction,
            # continues from the same unreachable point.
            with self._lock:
                cur = dict(self._current)
                self._gesture_target = dict(cur)
                self._target = dict(cur)
                self._last_cmd = None
            self._clear_command_queue()

    def _hold_gesture_at_current(self, reason: str = "reset"):
        with self._lock:
            cur = self._gesture_current_without_pan_spike()
            self._gesture_target = dict(cur)
            self._gesture_joint_lock = dict(cur)
            self._gesture_pan_input_offset = 0.0
            self._gesture_pan_applied_target = float(
                cur.get("shoulder_pan", HOME_ANGLES["shoulder_pan"])
            )
            if self._control_mode == "GESTURE":
                self._target = dict(cur)
                self._last_cmd = None
        if self._control_mode == "GESTURE":
            self._clear_command_queue()
            self._motion_done = True
            self.pub_done.publish(Bool(data=True))
        self.get_logger().info(
            f"GESTURE hold at current ({reason})",
            throttle_duration_sec=1.0,
        )

    def _cb_gesture_reset(self, msg: Bool):
        if not msg.data:
            return
        # 只清除手势目标。非 GESTURE 模式下不改 _target，避免影响 AUTO/TELEOP。
        self._hold_gesture_at_current("/gesture/reset")
        self._gesture_zero_since = None
        self._gesture_zero_hold_applied = True

    def _cb_hw_enable(self, msg: Bool):
        was = self._hw_enabled
        self._hw_enabled = msg.data
        if was and not msg.data:
            self._set_control_mode("STOW")
            self.get_logger().warn("安全联锁触发 → 强制收臂 🔴")
            with self._lock:
                self._target   = dict(HOME_ANGLES)
                self._last_cmd = None
            self._motion_done = False
            self.pub_done.publish(Bool(data=False))

    def _clear_command_queue(self):
        while True:
            try:
                self._cmd_queue.get_nowait()
            except queue.Empty:
                return

    def _gesture_current_without_pan_spike(self):
        cur = dict(self._current)
        pan = float(cur.get("shoulder_pan", HOME_ANGLES["shoulder_pan"]))
        if pan >= self._gesture_pan_spike_threshold:
            if self._gesture_last_valid_pan is not None:
                cur["shoulder_pan"] = float(self._gesture_last_valid_pan)
            else:
                cur["shoulder_pan"] = float(
                    self._target.get("shoulder_pan", HOME_ANGLES["shoulder_pan"])
                )
        else:
            self._gesture_last_valid_pan = pan
        return cur

    def _set_control_mode(self, mode):
        mode = mode.upper().strip()
        if mode not in {"AUTO", "TELEOP", "GESTURE", "EEG", "HOLD", "STOW"}:
            self.get_logger().warn(f"Ignoring invalid control mode: {mode}")
            return
        if mode == self._control_mode:
            return

        previous = self._control_mode
        with self._lock:
            if mode == "STOW":
                self._target = dict(HOME_ANGLES)
                self._gesture_target = None
                self._gesture_joint_lock = None
                self._gesture_pan_input_offset = 0.0
                self._gesture_pan_applied_target = None
            else:
                # Begin at the measured pose so stale commands cannot jump.
                current = (
                    self._gesture_current_without_pan_spike()
                    if mode == "GESTURE"
                    else dict(self._current)
                )
                self._target = dict(current)
                self._gesture_target = dict(current) if mode == "GESTURE" else None
                self._gesture_joint_lock = (
                    dict(current) if mode == "GESTURE" else None
                )
                self._gesture_pan_input_offset = 0.0
                self._gesture_pan_applied_target = (
                    float(current.get("shoulder_pan", 0.0))
                    if mode == "GESTURE"
                    else None
                )
                if mode == "AUTO" and self.auto_entry_pose_enabled:
                    lo, hi = JOINT_LIMITS[self.auto_entry_joint_name]
                    self._target[self.auto_entry_joint_name] = self._clamp(
                        self.auto_entry_joint_angle_deg,
                        lo,
                        hi,
                    )
            self._last_cmd = None
        self._clear_command_queue()
        self._gesture_zero_since = None
        self._gesture_zero_hold_applied = False
        self._last_gesture_grip_command = None
        self._control_mode = mode
        self._last_teleop_time = (
            time.monotonic() if mode == "TELEOP" else None
        )
        self._motion_done = mode == "HOLD"
        self.get_logger().warn(f"Arm control mode: {previous} -> {mode}")

    def _cb_control_mode(self, msg: String):
        self._set_control_mode(msg.data)

    def _cb_eeg_command(self, msg: String):
        self.eeg_controller.command(msg)

    def _cb_teleop_joint_target(self, msg: JointState):
        if not self._hw_enabled or self._control_mode not in {"TELEOP", "EEG"}:
            return
        self.teleop_controller.joint_target(msg)

    def _check_teleop_watchdog(self):
        if self._control_mode != "TELEOP":
            return
        if self._last_teleop_time is None:
            return
        age = time.monotonic() - self._last_teleop_time
        if age <= self.teleop_timeout:
            return
        self.get_logger().error(
            f"Teleop watchdog timeout ({age * 1000:.0f} ms); holding arm"
        )
        self._set_control_mode("HOLD")

    def destroy_node(self):
        self._hw_running = False
        if self.arm is not None:
            try:
                self.arm.set_action(dict(HOME_ANGLES))
                time.sleep(2.0)
                self.arm.disconnect()
            except Exception as e:
                self.get_logger().warn(f"断开连接异常: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101ArmInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
