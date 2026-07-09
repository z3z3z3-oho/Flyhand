#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ardupilot_interface.py
ArduPilot ↔ ROS2 桥接节点

职责：
  1. 通过 MAVLink（串口）与 ArduPilot 双向通信
  2. 桥接飞控数据 → ROS2 话题（位置、速度、模式、电量）
  3. 桥接 ROS2 设定点 → 飞控 GUIDED 模式位置指令
  4. ★ 机械臂安全联锁：
       - 上电/起飞/降落全程保持机械臂收缩
       - 仅在"安全悬停"条件满足后才允许机械臂伸出
       - 飞控通过 AUX PWM 或 MAVLink Relay 发出使能信号

ArduPilot 连接：
  串口：/dev/ttyS3（RK3588 UART3，推荐）或 /dev/ttyUSB0
  波特率：115200（ArduPilot 默认 SERIALX_BAUD=57，即57600或115200）
  协议：MAVLink 2

AUX PWM 联锁方案（二选一）：
  方案A（推荐）：Lua脚本控制 SERVO9/10 → RK3588 GPIO 读 PWM 电平
  方案B（简单）：读 MAVLink SERVO_OUTPUT_RAW 消息里的 AUX 通道值

发布话题：
  /drone/pose        → geometry_msgs/PoseStamped   无人机世界坐标系位姿
  /drone/velocity    → geometry_msgs/Twist          无人机速度
  /drone/state       → std_msgs/String              飞行模式+武装状态
  /battery_state     → sensor_msgs/BatteryState     飞控电量
  /arm/enable_hw     → std_msgs/Bool                机械臂硬件使能（安全联锁输出）

订阅话题：
  /drone/setpoint    → geometry_msgs/PoseStamped   发送给飞控的位置目标
  /arm/go_home       → std_msgs/Bool               收到收臂请求时同步通知飞控（日志）

安装依赖：
  pip install pymavlink
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
import threading
import time
import math

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("[WARN] pymavlink not found. Run: pip install pymavlink")

# ── ArduPilot 飞行模式 ID（Copter）──────────────────────────
COPTER_MODES = {
    0:  "STABILIZE", 1:  "ACRO",    2:  "ALT_HOLD",
    3:  "AUTO",      4:  "GUIDED",  5:  "LOITER",
    6:  "RTL",       7:  "CIRCLE",  9:  "LAND",
    11: "DRIFT",     13: "SPORT",   16: "POSHOLD",
    17: "BRAKE",     18: "THROW",   19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL",
}

# 允许伸出机械臂的飞行模式
ARM_ALLOW_MODES = {"GUIDED", "LOITER", "POSHOLD", "ALT_HOLD"}

# AUX 通道号（对应 ArduPilot Lua 脚本里设置的 SERVO 索引，1-based）
AUX_ARM_ENABLE_CHANNEL = 9   # SERVO9 = AUX1（通常）


class ArdupilotInterface(Node):
    """
    ArduPilot MAVLink ↔ ROS2 桥接节点
    包含机械臂安全联锁逻辑
    """

    def __init__(self):
        super().__init__("ardupilot_interface")

        # ── 参数 ──────────────────────────────────
        self.declare_parameter("serial_port",       "/dev/ttyS3")
        self.declare_parameter("baudrate",          115200)
        self.declare_parameter("system_id",         255)     # GCS system ID
        self.declare_parameter("component_id",      190)
        self.declare_parameter("safe_alt_m",        1.5)     # 允许伸臂的最低高度（米）
        self.declare_parameter("arm_enable_method", "mavlink")  # mavlink | pwm_gpio
        self.declare_parameter("pwm_gpio_pin",      "GPIO3_A0")  # RK3588 GPIO（方案A）
        self.declare_parameter("heartbeat_hz",      1.0)
        self.declare_parameter("setpoint_hz",       10.0)    # GUIDED 模式位置指令频率

        self.serial_port       = self.get_parameter("serial_port").value
        self.baudrate          = self.get_parameter("baudrate").value
        self.safe_alt          = self.get_parameter("safe_alt_m").value
        self.arm_enable_method = self.get_parameter("arm_enable_method").value
        self.heartbeat_hz      = self.get_parameter("heartbeat_hz").value
        self.setpoint_hz       = self.get_parameter("setpoint_hz").value

        # ── 内部状态 ──────────────────────────────
        self._mav             = None        # MAVLink 连接
        self._armed           = False       # 飞控武装状态
        self._flight_mode     = "UNKNOWN"   # 当前飞行模式
        self._alt_rel         = 0.0         # 相对高度（米）
        self._pos_ned         = [0.0, 0.0, 0.0]  # NED 本地坐标
        self._vel_ned         = [0.0, 0.0, 0.0]  # NED 速度
        self._battery_pct     = -1.0
        self._battery_v       = 0.0
        self._aux_pwm_value   = 1000        # AUX 通道当前 PWM 值（us）
        self._last_hb_ts      = 0.0         # 最近一次心跳时间
        self._mav_lock        = threading.Lock()

        # 机械臂硬件使能状态（综合安全联锁的输出）
        self._arm_hw_enabled  = False

        # 待发送的目标位置（GUIDED 模式）
        self._setpoint        : PoseStamped | None = None
        self._setpoint_active = False

        # ── 发布 ──────────────────────────────────
        self.pub_pose    = self.create_publisher(PoseStamped,  "/drone/pose",      10)
        self.pub_vel     = self.create_publisher(Twist,        "/drone/velocity",  10)
        self.pub_state   = self.create_publisher(String,       "/drone/state",     10)
        self.pub_battery = self.create_publisher(BatteryState, "/battery_state",   10)
        self.pub_arm_en  = self.create_publisher(Bool,         "/arm/enable_hw",   10)

        # ── 订阅 ──────────────────────────────────
        self.create_subscription(PoseStamped, "/drone/setpoint", self._cb_setpoint, 10)
        self.create_subscription(Bool,        "/arm/go_home",    self._cb_arm_home, 10)

        # ── 连接 MAVLink ──────────────────────────
        if MAVLINK_AVAILABLE:
            self._connect_mavlink()
            # MAVLink 接收线程
            self._recv_thread = threading.Thread(
                target=self._recv_loop, daemon=True)
            self._recv_thread.start()
        else:
            self.get_logger().error("pymavlink not installed! pip install pymavlink")

        # ── 定时器 ────────────────────────────────
        self.create_timer(1.0 / self.heartbeat_hz, self._send_heartbeat)
        self.create_timer(1.0 / self.setpoint_hz,  self._send_setpoint)
        self.create_timer(0.5,                      self._update_arm_enable)
        self.create_timer(1.0,                      self._publish_state)

        self.get_logger().info(
            f"ArdupilotInterface ready | "
            f"port={self.serial_port}@{self.baudrate} | "
            f"safe_alt={self.safe_alt}m | "
            f"arm_enable={self.arm_enable_method}"
        )

    # ═══════════════════════════════════════════
    # MAVLink 连接
    # ═══════════════════════════════════════════
    def _connect_mavlink(self):
        try:
            self._mav = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baudrate,
                source_system=self.get_parameter("system_id").value,
                source_component=self.get_parameter("component_id").value,
            )
            # 等待第一个心跳包（超时 5s）
            self.get_logger().info(f"Waiting for ArduPilot heartbeat on {self.serial_port}...")
            self._mav.wait_heartbeat(timeout=5)
            self._last_hb_ts = time.time()
            self.get_logger().info(
                f"ArduPilot connected | "
                f"sysid={self._mav.target_system} "
                f"compid={self._mav.target_component}"
            )

            # 请求数据流
            self._request_data_streams()

        except Exception as e:
            self.get_logger().error(f"MAVLink connection failed: {e}")
            self._mav = None

    def _request_data_streams(self):
        """请求 ArduPilot 以指定频率发送各类数据"""
        if self._mav is None:
            return
        stream_rates = {
            mavutil.mavlink.MAV_DATA_STREAM_POSITION:    10,  # 位置 10Hz
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1:       5,  # 姿态  5Hz
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2:       2,  # VFR   2Hz
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS:  5,  # RC/AUX 5Hz（用于读PWM）
        }
        for stream_id, rate in stream_rates.items():
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                stream_id,
                rate,
                1,  # start
            )

    # ═══════════════════════════════════════════
    # MAVLink 接收循环（独立线程）
    # ═══════════════════════════════════════════
    def _recv_loop(self):
        """持续接收并解析 MAVLink 消息"""
        while rclpy.ok():
            if self._mav is None:
                time.sleep(0.5)
                continue
            try:
                msg = self._mav.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT":
                    self._handle_heartbeat(msg)
                elif msg_type == "LOCAL_POSITION_NED":
                    self._handle_local_pos(msg)
                elif msg_type == "VFR_HUD":
                    self._handle_vfr_hud(msg)
                elif msg_type == "SYS_STATUS":
                    self._handle_sys_status(msg)
                elif msg_type == "SERVO_OUTPUT_RAW":
                    self._handle_servo_output(msg)
                elif msg_type == "STATUSTEXT":
                    self.get_logger().info(f"[FC] {msg.text}")

            except Exception as e:
                self.get_logger().warn(f"MAVLink recv error: {e}")
                time.sleep(0.1)

    def _handle_heartbeat(self, msg):
        with self._mav_lock:
            self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode_id = msg.custom_mode
            self._flight_mode = COPTER_MODES.get(mode_id, f"MODE_{mode_id}")
            self._last_hb_ts  = time.time()

    def _handle_local_pos(self, msg):
        """LOCAL_POSITION_NED → /drone/pose + /drone/velocity"""
        with self._mav_lock:
            # NED → ROS ENU 坐标系转换
            # NED: x=North, y=East, z=Down
            # ENU: x=East,  y=North, z=Up
            self._pos_ned = [msg.x, msg.y, msg.z]
            self._vel_ned = [msg.vx, msg.vy, msg.vz]

        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp    = stamp
        pose.header.frame_id = "map"
        pose.pose.position   = Point(
            x=float(msg.y),    # East
            y=float(msg.x),    # North
            z=float(-msg.z),   # Up（NED z 取反）
        )
        pose.pose.orientation.w = 1.0  # 姿态简化（完整版需从ATTITUDE消息获取）
        self.pub_pose.publish(pose)

        vel = Twist()
        vel.linear.x = float(msg.vy)   # East
        vel.linear.y = float(msg.vx)   # North
        vel.linear.z = float(-msg.vz)  # Up
        self.pub_vel.publish(vel)

    def _handle_vfr_hud(self, msg):
        with self._mav_lock:
            self._alt_rel = float(msg.alt)  # 相对起飞点高度

    def _handle_sys_status(self, msg):
        with self._mav_lock:
            if msg.battery_remaining >= 0:
                self._battery_pct = float(msg.battery_remaining) / 100.0
            self._battery_v = float(msg.voltage_battery) / 1000.0

        bat = BatteryState()
        bat.voltage    = self._battery_v
        bat.percentage = float(self._battery_pct)
        bat.present    = True
        self.pub_battery.publish(bat)

    def _handle_servo_output(self, msg):
        """
        读 AUX PWM 通道值（方案B）
        SERVO_OUTPUT_RAW 包含 servo1~16 的输出值（us）
        AUX_ARM_ENABLE_CHANNEL=9 → servo9_raw
        """
        channel_attr = f"servo{AUX_ARM_ENABLE_CHANNEL}_raw"
        if hasattr(msg, channel_attr):
            with self._mav_lock:
                self._aux_pwm_value = getattr(msg, channel_attr)

    # ═══════════════════════════════════════════
    # 机械臂安全联锁
    # ═══════════════════════════════════════════
    def _update_arm_enable(self):
        """
        综合所有安全条件，决定机械臂硬件使能状态。
        任何一个条件不满足 → 立即禁用（收臂）。

        安全条件（全部满足才允许伸臂）：
          1. ArduPilot 已武装（ARMED）
          2. 飞行模式在允许列表内（GUIDED / LOITER / POSHOLD 等）
          3. 相对高度 > safe_alt_m（默认 1.5m，防止地面伸臂）
          4. 飞控心跳正常（1s 内收到过心跳）
          5. AUX 使能信号高（PWM > 1500us，方案B）OR 方案A 模式跳过此项
        """
        with self._mav_lock:
            armed      = self._armed
            mode       = self._flight_mode
            alt        = self._alt_rel
            hb_age     = time.time() - self._last_hb_ts
            aux_pwm    = self._aux_pwm_value

        # 逐条检查
        reasons = []

        if not armed:
            reasons.append("NOT_ARMED")

        if mode not in ARM_ALLOW_MODES:
            reasons.append(f"BAD_MODE({mode})")

        if alt < self.safe_alt:
            reasons.append(f"LOW_ALT({alt:.1f}m<{self.safe_alt}m)")

        if hb_age > 2.0:
            reasons.append(f"HB_TIMEOUT({hb_age:.0f}s)")

        # AUX PWM 检查（仅 mavlink 方案B 时启用）
        if self.arm_enable_method == "mavlink":
            if aux_pwm < 1500:
                reasons.append(f"AUX_LOW({aux_pwm}us)")

        new_enabled = (len(reasons) == 0)

        if new_enabled != self._arm_hw_enabled:
            self._arm_hw_enabled = new_enabled
            state_str = "ENABLED" if new_enabled else f"DISABLED({','.join(reasons)})"
            self.get_logger().info(f"Arm HW enable → {state_str}")

        self.pub_arm_en.publish(Bool(data=self._arm_hw_enabled))

    # ═══════════════════════════════════════════
    # MAVLink 发送
    # ═══════════════════════════════════════════
    def _send_heartbeat(self):
        if self._mav is None:
            return
        try:
            self._mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        except Exception as e:
            self.get_logger().warn(f"Heartbeat send error: {e}")

    def _send_setpoint(self):
        """
        以 setpoint_hz 频率持续发送位置目标（GUIDED 模式必须持续发送）
        """
        if self._mav is None or self._setpoint is None:
            return
        if not self._setpoint_active:
            return

        with self._mav_lock:
            mode = self._flight_mode

        if mode != "GUIDED":
            return

        sp = self._setpoint
        # ENU → NED 坐标转换
        x_ned =  sp.pose.position.y   # North = ROS y
        y_ned =  sp.pose.position.x   # East  = ROS x
        z_ned = -sp.pose.position.z   # Down  = -ROS z

        try:
            self._mav.mav.set_position_target_local_ned_send(
                0,                              # time_boot_ms
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                # type_mask: 仅使用位置（忽略速度/加速度/yaw）
                0b110111111000,
                x_ned, y_ned, z_ned,
                0, 0, 0,    # vx vy vz（忽略）
                0, 0, 0,    # ax ay az（忽略）
                0, 0,       # yaw yaw_rate（忽略）
            )
        except Exception as e:
            self.get_logger().warn(f"Setpoint send error: {e}")

    def _publish_state(self):
        with self._mav_lock:
            armed  = self._armed
            mode   = self._flight_mode
            alt    = self._alt_rel
            bat    = self._battery_pct
            hb_age = time.time() - self._last_hb_ts

        state_str = (
            f"mode={mode} armed={armed} "
            f"alt={alt:.1f}m bat={bat*100:.0f}% "
            f"arm_hw={'ON' if self._arm_hw_enabled else 'OFF'} "
            f"hb_age={hb_age:.1f}s"
        )
        self.pub_state.publish(String(data=state_str))

    # ═══════════════════════════════════════════
    # 话题回调
    # ═══════════════════════════════════════════
    def _cb_setpoint(self, msg: PoseStamped):
        """接收 ROS2 位置目标，缓存待发送给飞控"""
        self._setpoint        = msg
        self._setpoint_active = True

    def _cb_arm_home(self, msg: Bool):
        """收到收臂指令时记录日志（飞控侧不需要额外操作）"""
        if msg.data:
            self.get_logger().info("Arm retract requested by state machine")


def main(args=None):
    rclpy.init(args=args)
    node = ArdupilotInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
