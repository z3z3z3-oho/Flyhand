#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visual_servo.py
末端视觉伺服节点（IBVS - Image-Based Visual Servoing）

当机械臂粗运动完成后，用末端相机做闭环伺服，
消除无人机悬停抖动带来的位置误差。

订阅:
  /camera/end/image_raw   → sensor_msgs/Image（末端相机）
  /servo/enable           → std_msgs/Bool（使能/停止伺服）

发布:
  /servo/arm_delta        → geometry_msgs/Twist（末端增量指令）
  /servo/aligned          → std_msgs/Bool（是否对准）
  /servo/debug            → sensor_msgs/Image（可视化）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

try:
    from rknnlite.api import RKNNLite
    RKNN_AVAILABLE = True
except ImportError:
    RKNN_AVAILABLE = False


class VisualServoNode(Node):
    """
    基于图像的视觉伺服 (IBVS)

    策略：
      - 末端相机中目标松塔的质心 → 对准图像中心
      - 目标检测框大小 → 控制进深方向距离
      - 比例控制 + 死区
    """

    # 目标区域：图像中心附近的允许误差（像素）
    ALIGN_THRESHOLD_PX  = 15   # 中心对准阈值（像素）
    ALIGN_THRESHOLD_Z   = 0.02  # 深度对准阈值（m）
    TARGET_BBOX_WIDTH   = 120   # 期望检测框宽度（像素，对应约10cm距离）

    def __init__(self):
        super().__init__("visual_servo")

        # ── 参数 ──────────────────────────────────
        self.declare_parameter("model_path",   "/opt/models/pine_cone_yolov8n.rknn")
        self.declare_parameter("kp_xy",        0.003)    # 横向比例增益
        self.declare_parameter("kp_z",         0.002)    # 进深比例增益
        self.declare_parameter("max_vel_xy",   0.05)     # 最大横向速度（m/s）
        self.declare_parameter("max_vel_z",    0.03)     # 最大进深速度（m/s）
        self.declare_parameter("conf_thresh",  0.4)

        self.model_path  = self.get_parameter("model_path").value
        self.kp_xy       = self.get_parameter("kp_xy").value
        self.kp_z        = self.get_parameter("kp_z").value
        self.max_vel_xy  = self.get_parameter("max_vel_xy").value
        self.max_vel_z   = self.get_parameter("max_vel_z").value
        self.conf_thresh = self.get_parameter("conf_thresh").value

        # ── 状态 ──────────────────────────────────
        self.enabled       = False
        self.aligned       = False
        self._last_det_ts  = 0.0
        self.bridge        = CvBridge()
        self._img_w        = 640
        self._img_h        = 480

        # ── 加载轻量检测模型（末端相机专用）────────
        self._load_model()

        # ── 订阅 ──────────────────────────────────
        self.sub_image   = self.create_subscription(
            Image, "/camera/end/image_raw", self._image_cb, 10)
        self.sub_enable  = self.create_subscription(
            Bool, "/servo/enable", self._enable_cb, 10)

        # ── 发布 ──────────────────────────────────
        self.pub_delta   = self.create_publisher(Twist, "/servo/arm_delta", 10)
        self.pub_aligned = self.create_publisher(Bool,  "/servo/aligned",   10)
        self.pub_debug   = self.create_publisher(Image, "/servo/debug",     1)

        # ── 超时定时器（检测丢失时发零速）──────────
        self.create_timer(0.1, self._timeout_check)

        self.get_logger().info("VisualServo ready | IBVS controller initialized")

    def _load_model(self):
        """为末端相机加载独立的轻量推理模型"""
        if RKNN_AVAILABLE:
            try:
                self.rknn = RKNNLite()
                # 末端模型用单独的 NPU 核心（core_mask=0b010=Core1），
                # 避免与机身YOLO（Core0+Core2）竞争
                self.rknn.load_rknn(self.model_path)
                self.rknn.init_runtime(core_mask=2)
                self.get_logger().info("Visual servo RKNN model loaded on NPU Core1")
                self._has_model = True
            except Exception as e:
                self.get_logger().warn(f"RKNN load failed: {e}, using color detection fallback")
                self._has_model = False
        else:
            self._has_model = False

    # ─────────────────────────────────────────────
    # 使能回调
    # ─────────────────────────────────────────────
    def _enable_cb(self, msg: Bool):
        self.enabled = msg.data
        if self.enabled:
            self.aligned = False
            self.get_logger().info("Visual servo ENABLED")
        else:
            self.get_logger().info("Visual servo DISABLED")
            self._publish_zero_vel()

    # ─────────────────────────────────────────────
    # 图像回调：检测 + 伺服计算
    # ─────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        if not self.enabled:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        self._img_h, self._img_w = frame.shape[:2]
        cx_img = self._img_w / 2.0
        cy_img = self._img_h / 2.0

        # ── 目标检测（优先RKNN，回退颜色检测）──────
        det = self._detect_target(frame)

        if det is None:
            # 目标丢失：发零速，等待重新检测
            self._publish_zero_vel()
            return

        self._last_det_ts = time.time()
        tx, ty, bbox_w = det  # 目标中心像素坐标 + 检测框宽度

        # ── IBVS 误差计算 ─────────────────────────
        ex = tx - cx_img             # 横向误差（像素）
        ey = ty - cy_img             # 纵向误差（像素）
        ez = bbox_w - self.TARGET_BBOX_WIDTH  # 进深误差（像素，正=太近）

        # ── 对准判断 ─────────────────────────────
        aligned_xy = (abs(ex) < self.ALIGN_THRESHOLD_PX and
                      abs(ey) < self.ALIGN_THRESHOLD_PX)
        aligned_z  = abs(ez) < 20  # bbox宽度误差<20px

        new_aligned = aligned_xy and aligned_z
        if new_aligned != self.aligned:
            self.aligned = new_aligned
            self.pub_aligned.publish(Bool(data=self.aligned))
            if self.aligned:
                self.get_logger().info(
                    f"✅ Aligned! ex={ex:.1f}px ey={ey:.1f}px ez={ez:.1f}px"
                )

        # ── 比例控制 → 速度指令 ───────────────────
        twist = Twist()
        if not self.aligned:
            # 横向：图像x误差 → 机械臂y轴运动
            vy = -self.kp_xy * ex
            # 纵向：图像y误差 → 机械臂z轴运动
            vz = -self.kp_xy * ey
            # 进深：bbox大小误差 → 机械臂x轴运动
            vx =  self.kp_z  * ez

            # 限速
            twist.linear.x = float(np.clip(vx, -self.max_vel_z,  self.max_vel_z))
            twist.linear.y = float(np.clip(vy, -self.max_vel_xy, self.max_vel_xy))
            twist.linear.z = float(np.clip(vz, -self.max_vel_xy, self.max_vel_xy))

        self.pub_delta.publish(twist)

        # ── 可视化 ────────────────────────────────
        viz = self._draw_servo_viz(frame, tx, ty, bbox_w, ex, ey, aligned_xy)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(viz, "bgr8"))

    # ─────────────────────────────────────────────
    # 目标检测（末端相机）
    # ─────────────────────────────────────────────
    def _detect_target(self, frame: np.ndarray):
        """
        返回 (cx, cy, bbox_width) 或 None
        """
        if self._has_model:
            return self._detect_rknn(frame)
        return self._detect_color_fallback(frame)

    def _detect_rknn(self, frame: np.ndarray):
        """RKNN轻量推理（复用yolo_detector中的预处理逻辑）"""
        try:
            h, w = frame.shape[:2]
            scale = min(640/h, 640/w)
            nw, nh = int(w*scale), int(h*scale)
            resized = cv2.resize(frame, (nw, nh))
            padded = np.full((640, 640, 3), 114, dtype=np.uint8)
            pw, ph = (640-nw)//2, (640-nh)//2
            padded[ph:ph+nh, pw:pw+nw] = resized
            rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)

            outputs = self.rknn.inference(inputs=[rgb])
            pred = outputs[0][0].T  # [8400, 4+nc]

            conf = pred[:, 4]
            best = np.argmax(conf)
            if conf[best] < self.conf_thresh:
                return None

            bx, by, bw, bh = pred[best, :4]
            # 反算回原图坐标
            cx = (bx - pw) / scale
            cy = (by - ph) / scale
            bbox_w = bw / scale
            return (cx, cy, bbox_w)
        except Exception as e:
            self.get_logger().warn(f"RKNN servo infer error: {e}")
            return None

    def _detect_color_fallback(self, frame: np.ndarray):
        """
        颜色检测回退（松塔为棕褐色）
        实际使用时应替换为真实颜色范围
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 棕褐色范围（松塔）
        lower = np.array([5,  50, 30])
        upper = np.array([25, 255, 200])
        mask  = cv2.inRange(hsv, lower, upper)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5,5), np.uint8))
        mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10,10), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 200:
            return None

        x, y, w, h = cv2.boundingRect(largest)
        return (x + w/2, y + h/2, float(w))

    # ─────────────────────────────────────────────
    # 工具
    # ─────────────────────────────────────────────
    def _publish_zero_vel(self):
        self.pub_delta.publish(Twist())

    def _timeout_check(self):
        """检测超时：0.5s没有检测结果则停止伺服"""
        if self.enabled and time.time() - self._last_det_ts > 0.5:
            self._publish_zero_vel()

    def _draw_servo_viz(self, frame, tx, ty, bbox_w, ex, ey, aligned):
        viz = frame.copy()
        h, w = viz.shape[:2]
        cx, cy = w//2, h//2

        # 中心十字
        color = (0, 255, 0) if aligned else (0, 165, 255)
        cv2.line(viz, (cx-20, cy), (cx+20, cy), color, 2)
        cv2.line(viz, (cx, cy-20), (cx, cy+20), color, 2)

        # 目标点
        cv2.circle(viz, (int(tx), int(ty)), 8, (0, 0, 255), -1)

        # 误差向量
        cv2.arrowedLine(viz, (cx, cy), (int(tx), int(ty)), (255, 0, 0), 2)

        # 文字
        status = "ALIGNED" if aligned else f"ex={ex:.0f} ey={ey:.0f}"
        cv2.putText(viz, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(viz, f"bbox_w={bbox_w:.0f}px (target={self.TARGET_BBOX_WIDTH})",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        return viz

    def destroy_node(self):
        if RKNN_AVAILABLE and hasattr(self, "rknn"):
            self.rknn.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
