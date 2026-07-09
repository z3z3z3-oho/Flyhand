"""
harvest_state_machine.py  v3
采摘任务状态机（主控节点）

v3 改动（ArUco 抓取实验专项）：
  1. 新增 aruco_mode 参数：
       True  → COARSE_MOVE 到位后直接 GRASPING，跳过视觉伺服
       False → 原有流程，COARSE_MOVE → SERVOING → GRASPING
  2. 修复"只抓一次"问题：
       - 检测新鲜度阈值从 1.0s 放宽至 detection_valid_sec（默认 2.0s）
       - RETRACTING 结束后不清除 target_pose，保留旧目标供复用；
         同时用 _target_locked 标志防止在运动中被新检测覆盖
       - 打开夹爪（gripper_open）后再进 SEARCHING，确保下次抓取前夹爪张开
  3. 新增 GRIPPER_CLOSE 中间状态：
       先闭合夹爪并等待 arm/motion_done（或超时），确认夹爪动作完成后再 RETRACTING
  4. 新增指令：
       'next'  → 手动跳到 SEARCHING（当前目标完成/跳过，找下一个）
       'grasp' → 在 COARSE_MOVE 到位后手动触发夹爪（调试用）
  5. 每次回到 SEARCHING 时自动发布夹爪张开指令，确保安全

状态流（aruco_mode=True，地面测试）：
  IDLE → SEARCHING → [APPROACHING] → HOVERING → COARSE_MOVE
       → GRIPPER_CLOSE → RETRACTING → SEARCHING → ...（循环）

状态流（aruco_mode=False，视觉伺服模式）：
  IDLE → SEARCHING → [APPROACHING] → HOVERING → COARSE_MOVE
       → SERVOING → GRIPPER_CLOSE → RETRACTING → SEARCHING → ...
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32, Int32
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger
import numpy as np
import time
from enum import Enum, auto
from dataclasses import dataclass, field


# ───────────────────────────────────────────────
# 状态定义
# ───────────────────────────────────────────────
class State(Enum):
    IDLE          = auto()
    SEARCHING     = auto()
    APPROACHING   = auto()
    HOVERING      = auto()
    COARSE_MOVE   = auto()
    SERVOING      = auto()   # 视觉伺服（aruco_mode 时跳过）
    GRIPPER_CLOSE = auto()   # 夹爪闭合+确认（新增）
    RETRACTING    = auto()
    RETURNING     = auto()
    ABORT         = auto()


@dataclass
class HarvestContext:
    target_pose:        PoseStamped | None = None
    target_id:          int                = -1    # ArUco ID，-1 表示未知
    drone_stable_since: float              = 0.0
    coarse_move_sent:   bool               = False
    servo_aligned:      bool               = False
    grasp_confirmed:    bool               = False
    last_detection_ts:  float              = 0.0
    harvest_count:      int                = 0
    abort_reason:       str                = ""
    battery_pct:        float              = 100.0


@dataclass
class HarvestConfig:
    # 检测
    detection_valid_sec:    float = 2.0    # 目标坐标有效期（秒），放宽以应对低帧率检测
    detection_lost_timeout: float = 5.0    # SEARCHING 中超过此时间未检测到→继续等待

    # 飞行
    hover_stable_duration:  float = 1.0
    hover_vel_thresh:       float = 0.08
    approach_distance:      float = 0.40
    approach_pos_thresh:    float = 0.10

    # 机械臂
    coarse_move_timeout:    float = 10.0   # 粗运动超时（秒）
    servo_timeout:          float = 8.0    # 视觉伺服超时
    pre_grasp_pause:        float = 0.8    # 到位后停顿再夹（秒），让机械臂稳定，抓取更自然
    gripper_close_wait:     float = 1.5    # 夹爪闭合等待时间（无 motion_done 时的兜底）
    gripper_close_timeout:  float = 4.0    # 夹爪闭合超时
    retract_duration:       float = 3.0    # 收臂等待时间
    inter_grasp_delay:      float = 2.0    # 每次抓取完成后等待时间（秒），再启动下一次搜索

    # 安全
    min_battery_pct:        float = 20.0

    # 模式
    loop_rate_hz:           float = 20.0
    ground_test_mode:       bool  = False  # True → 跳过飞行
    aruco_mode:             bool  = True   # True → 跳过视觉伺服，直接夹取


class HarvestStateMachine(Node):

    def __init__(self):
        super().__init__("harvest_state_machine")

        # ── 参数声明 ──────────────────────────────
        self.declare_parameter("ground_test_mode",    False)
        self.declare_parameter("aruco_mode",          True)
        self.declare_parameter("coarse_move_timeout", 10.0)
        self.declare_parameter("approach_distance",   0.40)
        self.declare_parameter("approach_pos_thresh", 0.10)
        self.declare_parameter("detection_valid_sec", 2.0)
        self.declare_parameter("pre_grasp_pause",     0.8)
        self.declare_parameter("gripper_close_wait",  1.5)
        self.declare_parameter("retract_duration",    3.0)
        self.declare_parameter("inter_grasp_delay",   2.0)

        self.cfg = HarvestConfig(
            ground_test_mode   = self.get_parameter("ground_test_mode").value,
            aruco_mode         = self.get_parameter("aruco_mode").value,
            coarse_move_timeout= self.get_parameter("coarse_move_timeout").value,
            approach_distance  = self.get_parameter("approach_distance").value,
            approach_pos_thresh= self.get_parameter("approach_pos_thresh").value,
            detection_valid_sec= self.get_parameter("detection_valid_sec").value,
            pre_grasp_pause    = self.get_parameter("pre_grasp_pause").value,
            gripper_close_wait = self.get_parameter("gripper_close_wait").value,
            retract_duration   = self.get_parameter("retract_duration").value,
            inter_grasp_delay  = self.get_parameter("inter_grasp_delay").value,
        )

        self.ctx   = HarvestContext()
        self.state = State.IDLE

        # ── 内部标志 ──────────────────────────────
        self._drone_vel       = np.zeros(3)
        self._drone_pos       = np.zeros(3)
        self._drone_pose_ts   = 0.0
        self._arm_motion_done = False    # /arm/motion_done 信号
        self._state_enter_time= time.time()

        # 目标锁定标志：COARSE_MOVE 开始后不再接受新目标，直到 RETRACTING 结束
        self._target_locked   = False

        # ── 订阅 ──────────────────────────────────
        self.create_subscription(PoseStamped, "/target/best",     self._cb_target,     10)
        self.create_subscription(Float32,     "/target/confidence",self._cb_conf,       10)
        self.create_subscription(Twist,       "/drone/velocity",  self._cb_drone_vel,  10)
        self.create_subscription(PoseStamped, "/drone/pose",      self._cb_drone_pose, 10)
        self.create_subscription(Bool,        "/arm/motion_done", self._cb_arm_done,   10)
        self.create_subscription(Bool,        "/servo/aligned",   self._cb_aligned,    10)
        self.create_subscription(BatteryState,"/battery_state",   self._cb_battery,    10)
        self.create_subscription(String,      "/task/command",    self._cb_command,    10)

        # ── 发布 ──────────────────────────────────
        self.pub_state      = self.create_publisher(String,      "/task/state",          10)
        self.pub_drone_pose = self.create_publisher(PoseStamped, "/drone/setpoint",      10)
        self.pub_arm_pose   = self.create_publisher(PoseStamped, "/arm/target_pose",     10)
        self.pub_servo_en   = self.create_publisher(Bool,        "/servo/enable",        10)
        self.pub_gripper    = self.create_publisher(Bool,        "/gripper/close",       10)
        self.pub_arm_home   = self.create_publisher(Bool,        "/arm/go_home",         10)
        self.pub_log        = self.create_publisher(String,      "/task/log",            10)
        self.pub_harvest    = self.create_publisher(Int32,       "/task/harvest_count",  10)

        # ── 服务 ──────────────────────────────────
        self.cli_calib_load = self.create_client(Trigger, "/hand_eye/load")

        # ── 定时器 ────────────────────────────────
        self.create_timer(1.0 / self.cfg.loop_rate_hz, self._step)

        mode_str = []
        if self.cfg.ground_test_mode: mode_str.append("GROUND_TEST")
        if self.cfg.aruco_mode:       mode_str.append("ARUCO_MODE")
        self.get_logger().info(f"HarvestStateMachine v3 ready | {' | '.join(mode_str) or 'FLIGHT'}")
        self._log("Ready. Commands: start / hover / next / stop / return")

    # ═══════════════════════════════════════════
    # 主循环
    # ═══════════════════════════════════════════
    def _step(self):
        if self._check_abort():
            return
        {
            State.IDLE:          self._s_idle,
            State.SEARCHING:     self._s_searching,
            State.APPROACHING:   self._s_approaching,
            State.HOVERING:      self._s_hovering,
            State.COARSE_MOVE:   self._s_coarse_move,
            State.SERVOING:      self._s_servoing,
            State.GRIPPER_CLOSE: self._s_gripper_close,
            State.RETRACTING:    self._s_retracting,
            State.RETURNING:     self._s_returning,
            State.ABORT:         self._s_abort,
        }[self.state]()
        self.pub_state.publish(String(data=self.state.name))
        self.pub_harvest.publish(Int32(data=self.ctx.harvest_count))

    # ═══════════════════════════════════════════
    # 各状态实现
    # ═══════════════════════════════════════════

    def _s_idle(self):
        pass

    def _s_searching(self):
        """
        等待有效目标检测。
        目标有效条件：target_pose 不为 None 且 detection_valid_sec 内刷新过。
        """
        now = time.time()
        if (self.ctx.target_pose is not None and
                now - self.ctx.last_detection_ts < self.cfg.detection_valid_sec):
            if self.cfg.ground_test_mode:
                # 地面测试：跳过飞行直接悬停
                self._log(f"Target detected → HOVERING (ground test)")
                self._arm_motion_done = False
                self._target_locked   = False
                self._transition(State.HOVERING)
            else:
                self._log("Target detected → APPROACHING")
                self._transition(State.APPROACHING)

    def _s_approaching(self):
        if self.ctx.target_pose is None:
            self._handle_lost()
            return

        if time.time() - self._drone_pose_ts > 1.0:
            self._log("Waiting for /drone/pose...", "warn")
            return

        hover_pos = self._target_to_world_hover(
            self.ctx.target_pose, self.cfg.approach_distance)

        sp = PoseStamped()
        sp.header.frame_id = "map"
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.pose.position.x = float(hover_pos[0])
        sp.pose.position.y = float(hover_pos[1])
        sp.pose.position.z = float(hover_pos[2])
        sp.pose.orientation.w = 1.0
        self.pub_drone_pose.publish(sp)

        dist = float(np.linalg.norm(self._drone_pos - hover_pos))
        if dist < self.cfg.approach_pos_thresh:
            self._log("At hover point → HOVERING")
            self.ctx.drone_stable_since = 0.0
            self._transition(State.HOVERING)

    def _s_hovering(self):
        vel_mag = float(np.linalg.norm(self._drone_vel))
        if vel_mag < self.cfg.hover_vel_thresh:
            if self.ctx.drone_stable_since == 0.0:
                self.ctx.drone_stable_since = time.time()
            stable = time.time() - self.ctx.drone_stable_since
            if stable >= self.cfg.hover_stable_duration:
                self._log(f"Stable {stable:.1f}s → COARSE_MOVE")
                self._arm_motion_done = False
                self._target_locked   = True   # 锁定当前目标
                self._transition(State.COARSE_MOVE)
        else:
            self.ctx.drone_stable_since = 0.0

    def _s_coarse_move(self):
        """
        发送机械臂目标位姿，等待真实到位信号。
        到位后：
          aruco_mode=True  → 直接进 GRIPPER_CLOSE
          aruco_mode=False → 进 SERVOING（视觉伺服精对准）
        """
        elapsed = time.time() - self._state_enter_time

        # 首次进入：发送目标
        if not self.ctx.coarse_move_sent:
            if self.ctx.target_pose is not None:
                self.pub_arm_pose.publish(self.ctx.target_pose)
                self.ctx.coarse_move_sent = True
                p = self.ctx.target_pose.pose.position
                self._log(
                    f"Arm → ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
                    + (f" [ArUco #{self.ctx.target_id}]" if self.ctx.target_id >= 0 else "")
                )
            else:
                self._log("No target! Returning to SEARCHING", "warn")
                self._target_locked = False
                self._transition(State.SEARCHING)
                return

        # 安全超时
        if elapsed > self.cfg.coarse_move_timeout:
            self._log("Coarse move TIMEOUT → ABORT", "error")
            self._abort("Coarse arm move timeout")
            return

        # 到位判断
        if self._arm_motion_done:
            self._arm_motion_done = False
            if self.cfg.aruco_mode:
                self._log(f"At target ({elapsed:.1f}s) [ArUco mode] → GRIPPER_CLOSE")
                self._transition(State.GRIPPER_CLOSE)
            else:
                self._log(f"At target ({elapsed:.1f}s) → SERVOING")
                self.pub_servo_en.publish(Bool(data=True))
                self._transition(State.SERVOING)

    def _s_servoing(self):
        """视觉伺服精对准（aruco_mode=False 时才进入）"""
        elapsed = time.time() - self._state_enter_time

        if elapsed > self.cfg.servo_timeout:
            self._log("Servo timeout → retry HOVERING", "warn")
            self.pub_servo_en.publish(Bool(data=False))
            self.ctx.coarse_move_sent = False
            self._arm_motion_done     = False
            self._transition(State.HOVERING)
            return

        if self.ctx.servo_aligned:
            self._log(f"Aligned {elapsed:.1f}s → GRIPPER_CLOSE")
            self.pub_servo_en.publish(Bool(data=False))
            self._transition(State.GRIPPER_CLOSE)

    def _s_gripper_close(self):
        """
        闭合夹爪并等待确认。
        分两步：
          1. 到位后先停顿 pre_grasp_pause 秒（让机械臂稳定，抓取更自然）
          2. 再发夹爪闭合指令，等 motion_done 或超时
        完成后 → RETRACTING
        """
        elapsed = time.time() - self._state_enter_time

        # 阶段1：停顿（机械臂到位后稳定，抓取更自然）
        if elapsed < self.cfg.pre_grasp_pause:
            if elapsed < 0.05:
                self._log(f"At target, pausing {self.cfg.pre_grasp_pause:.1f}s before grasp...")
            return

        # 阶段2：发夹爪闭合指令（只在刚进入阶段2时发一次）
        if elapsed < self.cfg.pre_grasp_pause + 0.3:
            self._log("Closing gripper...")
            self._arm_motion_done = False
            self.pub_gripper.publish(Bool(data=True))
            return

        # 等待夹爪到位：motion_done 优先，超时兜底
        gripper_done = (
            self._arm_motion_done or
            elapsed >= self.cfg.gripper_close_timeout
        )
        if gripper_done:
            if self._arm_motion_done:
                self._log(f"Gripper closed (motion_done, {elapsed:.1f}s) → RETRACTING")
            else:
                self._log(f"Gripper closed (timeout {elapsed:.1f}s) → RETRACTING")
            self.ctx.grasp_confirmed = True
            self.ctx.harvest_count  += 1
            self._arm_motion_done    = False
            self._log(f"★ Grasped! Total={self.ctx.harvest_count}")
            self.pub_harvest.publish(Int32(data=self.ctx.harvest_count))
            self._transition(State.RETRACTING)

    def _s_retracting(self):
        """
        两阶段：
          阶段1（0 ~ retract_duration）：收臂回零位，打开夹爪
          阶段2（retract_duration ~ +inter_grasp_delay）：停顿等待，再启动下一次搜索
        inter_grasp_delay 可在 GCS 实时调整，设 0 则无间隔。
        """
        elapsed = time.time() - self._state_enter_time
        total   = self.cfg.retract_duration + self.cfg.inter_grasp_delay

        # 阶段1开始：发收臂和开夹爪指令
        if elapsed < 0.2:
            #self.pub_arm_home.publish(Bool(data=True))
            self.pub_gripper.publish(Bool(data=False))
            self._log("Retracting arm, opening gripper...")

        # 阶段1结束：收臂完成，进入间隔等待
        if self.cfg.retract_duration <= elapsed < total:
            remaining = total - elapsed
            if elapsed - self.cfg.retract_duration < 0.1:   # 只打一次日志
                self._log(
                    f"Arm retracted ✓  waiting {self.cfg.inter_grasp_delay:.1f}s "
                    f"before next grasp..."
                )
            return   # 等待中，不转移状态

        # 阶段2结束：进入搜索
        if elapsed >= total:
            self._target_locked       = False
            self.ctx.coarse_move_sent = False
            self.ctx.servo_aligned    = False
            self.ctx.grasp_confirmed  = False
            self._arm_motion_done     = False
            self._log(f"Ready → SEARCHING for next target "
                      f"(total={elapsed:.1f}s, harvested={self.ctx.harvest_count})")
            self._transition(State.SEARCHING)

    def _s_returning(self):
        self._log("Low battery, returning to base")

    def _s_abort(self):
        elapsed = time.time() - self._state_enter_time
        if elapsed < 0.5:
            self.pub_servo_en.publish(Bool(data=False))
            self.pub_arm_home.publish(Bool(data=True))
            self.pub_gripper.publish(Bool(data=False))
            self._target_locked = False
            self._log(f"ABORT: {self.ctx.abort_reason}", "error")

    # ═══════════════════════════════════════════
    # 安全检查
    # ═══════════════════════════════════════════
    def _check_abort(self) -> bool:
        if self.state == State.ABORT:
            return False
        if (self.ctx.battery_pct < self.cfg.min_battery_pct and
                self.state not in (State.IDLE, State.RETURNING)):
            self._abort(f"Low battery {self.ctx.battery_pct:.0f}%")
            self._transition(State.RETURNING)
            return True
        return False

    # ═══════════════════════════════════════════
    # 话题回调
    # ═══════════════════════════════════════════
    def _cb_target(self, msg: PoseStamped):
        # 目标锁定期间（COARSE_MOVE 开始到 RETRACTING 结束）不更新目标位姿，
        # 防止机械臂运动过程中目标坐标被新检测覆盖导致运动突变
        if not self._target_locked:
            self.ctx.target_pose       = msg
            self.ctx.last_detection_ts = time.time()
        else:
            # 仅更新时间戳，让 SEARCHING 知道检测还活着，但不换坐标
            self.ctx.last_detection_ts = time.time()

    def _cb_conf(self, msg: Float32):
        pass

    def _cb_drone_vel(self, msg: Twist):
        self._drone_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])

    def _cb_drone_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self._drone_pos     = np.array([p.x, p.y, p.z])
        self._drone_pose_ts = time.time()

    def _cb_arm_done(self, msg: Bool):
        if msg.data:
            self._arm_motion_done = True

    def _cb_aligned(self, msg: Bool):
        self.ctx.servo_aligned = msg.data

    def _cb_battery(self, msg: BatteryState):
        if msg.percentage >= 0:
            self.ctx.battery_pct = msg.percentage * 100.0

    def _cb_command(self, msg: String):
        cmd = msg.data.lower().strip()

        if cmd == "start" and self.state in (State.IDLE, State.ABORT):
            # AUTO may be resumed after TELEOP/HOLD stopped the previous run.
            self.ctx.abort_reason = ""
            self._target_locked = False
            self.ctx.coarse_move_sent = False
            self.ctx.servo_aligned = False
            self.ctx.grasp_confirmed = False
            self._arm_motion_done = False
            self._log("Task started!")
            self._transition(State.SEARCHING)

        elif cmd == "hover" and self.state in (State.IDLE, State.SEARCHING, State.APPROACHING):
            # 地面测试：手动进悬停，跳过飞行
            self._log("Manual HOVER (ground test)")
            self._arm_motion_done = False
            self._target_locked   = False
            self._transition(State.HOVERING)

        elif cmd == "next":
            # 跳过当前目标，重新找下一个（任意状态均可）
            self._log("NEXT: skip current target → SEARCHING")
            #self.pub_arm_home.publish(Bool(data=True))
            self.pub_gripper.publish(Bool(data=False))
            self.pub_servo_en.publish(Bool(data=False))
            self._target_locked       = False
            self.ctx.coarse_move_sent = False
            self.ctx.servo_aligned    = False
            self._arm_motion_done     = False
            self.ctx.target_pose      = None   # 清掉旧目标，强制等新检测
            self._transition(State.SEARCHING)

        elif cmd == "grasp":
            # 调试用：在任意状态强制触发夹爪闭合
            if self.state == State.COARSE_MOVE:
                self._log("Manual GRASP triggered")
                self._arm_motion_done = True   # 伪造到位信号，让状态机推进

        elif cmd == "stop":
            self._abort("Manual stop")

        elif cmd == "return":
            self._transition(State.RETURNING)

        elif cmd == "calibrate":
            if self.cli_calib_load.service_is_ready():
                self.cli_calib_load.call_async(Trigger.Request())

        else:
            self._log(f"Unknown command: {cmd}", "warn")

    # ═══════════════════════════════════════════
    # 工具函数
    # ═══════════════════════════════════════════
    def _transition(self, new_state: State):
        old = self.state.name
        self.state = new_state
        self._state_enter_time = time.time()
        self._log(f"{old} → {new_state.name}")

    def _abort(self, reason: str):
        self.ctx.abort_reason = reason
        self._transition(State.ABORT)

    def _handle_lost(self):
        dt = time.time() - self.ctx.last_detection_ts
        if dt > self.cfg.detection_lost_timeout:
            self._log(f"Detection lost {dt:.0f}s → SEARCHING")
            self._transition(State.SEARCHING)

    def _target_to_world_hover(self, target: PoseStamped, standoff: float) -> np.ndarray:
        ARM_BASE_OFFSET = np.array([0.0, 0.0, -0.15])
        tp = target.pose.position
        target_body = np.array([tp.x, tp.y, tp.z]) - ARM_BASE_OFFSET
        target_world = self._drone_pos + target_body
        direction_xy = target_world[:2] - self._drone_pos[:2]
        norm = float(np.linalg.norm(direction_xy))
        unit_xy = direction_xy / norm if norm > 0.01 else np.array([1.0, 0.0])
        hover_pos = target_world.copy()
        hover_pos[0] -= unit_xy[0] * standoff
        hover_pos[1] -= unit_xy[1] * standoff
        return hover_pos

    def _log(self, msg: str, level: str = "info"):
        full = f"[{self.state.name}] {msg}"
        if level == "error":
            self.get_logger().error(full)
        elif level == "warn" or level == "warning":
            self.get_logger().warning(full)
        else:
            self.get_logger().info(full)
        self.pub_log.publish(String(data=full))


def main(args=None):
    rclpy.init(args=args)
    node = HarvestStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
