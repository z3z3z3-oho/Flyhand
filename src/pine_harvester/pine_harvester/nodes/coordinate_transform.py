import json
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String


PKG_SHARE = get_package_share_directory("pine_harvester")


def _as_transform_matrix(value):
    """Return a finite 4x4 transform matrix or raise a descriptive error."""
    if isinstance(value, dict):
        if "T_cam2body" not in value:
            raise ValueError("标定文件缺少 T_cam2body 字段")
        value = value["T_cam2body"]
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"手眼矩阵形状必须是 (4, 4)，实际为 {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("手眼矩阵包含 NaN 或无穷大")
    return matrix


def _transform_point(transform, point_cam):
    """Safely transform a 3-D or homogeneous 4-D point."""
    if point_cam is None:
        return None
    try:
        point = np.asarray(point_cam, dtype=np.float64).reshape(-1)
        matrix = _as_transform_matrix(transform)
    except (TypeError, ValueError):
        return None

    if point.size not in (3, 4) or not np.all(np.isfinite(point)):
        return None
    if point.size == 3:
        point = np.append(point, 1.0)

    transformed = matrix @ point
    if transformed.shape != (4,) or not np.all(np.isfinite(transformed)):
        return None
    if not np.isclose(transformed[3], 0.0) and not np.isclose(transformed[3], 1.0):
        transformed = transformed / transformed[3]
    return transformed[:3]


class CoordinateTransformNode(Node):
    def __init__(self):
        super().__init__("coordinate_transform")
        self.declare_parameter(
            "calib_path",
            os.path.join(PKG_SHARE, "calib", "hand_eye_result(1).json"),
        )
        self.declare_parameter("depth_mode", "aruco_ref")
        self.declare_parameter("fallback_depth", 0.4)
        self.declare_parameter("aruco_depth_offset_m", -0.035)
        self.declare_parameter("tennis_diameter_m", 0.067)

        self.calib_path = self.get_parameter("calib_path").value
        self.depth_mode = self.get_parameter("depth_mode").value
        self.fallback_depth = float(self.get_parameter("fallback_depth").value)
        self.aruco_depth_offset_m = float(
            self.get_parameter("aruco_depth_offset_m").value
        )
        self.tennis_diameter_m = float(
            self.get_parameter("tennis_diameter_m").value
        )

        self.camera_matrix = None
        self.dist_coeffs = None
        self.T_cam2body = np.eye(4, dtype=np.float64)
        self._load_hand_eye_result()

        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/target/best", 10
        )
        self.detections_sub = self.create_subscription(
            String, "/yolo/detections", self._detections_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/body/camera_info",
            self._camera_info_callback,
            10,
        )

        self.get_logger().info(
            "Coordinate transform ready | "
            f"depth_mode={self.depth_mode} | "
            f"tennis_diameter={self.tennis_diameter_m * 100:.1f} cm"
        )

    def _load_hand_eye_result(self):
        if not os.path.exists(self.calib_path):
            self.get_logger().warning(
                f"Hand-eye file not found: {self.calib_path}; using identity"
            )
            return
        try:
            with open(self.calib_path, "r", encoding="utf-8") as file:
                self.T_cam2body = _as_transform_matrix(json.load(file))
            self.get_logger().info(
                f"Hand-eye matrix loaded from {self.calib_path}"
            )
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.T_cam2body = np.eye(4, dtype=np.float64)
            self.get_logger().error(
                f"Invalid hand-eye calibration; using identity: {exc}"
            )

    def _camera_info_callback(self, msg):
        try:
            camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
            dist_coeffs = np.asarray(msg.d, dtype=np.float64).reshape(-1)
            if not np.all(np.isfinite(camera_matrix)):
                raise ValueError("相机内参包含 NaN 或无穷大")
            if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
                raise ValueError("相机焦距 fx/fy 必须大于 0")
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"无效的 CameraInfo，已忽略: {exc}")
            return

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.get_logger().info(
            f"Camera intrinsics received: fx={camera_matrix[0, 0]:.1f}, "
            f"fy={camera_matrix[1, 1]:.1f}, cx={camera_matrix[0, 2]:.1f}, "
            f"cy={camera_matrix[1, 2]:.1f}"
        )
        self.destroy_subscription(self.camera_info_sub)

    def transform_point(self, point_cam):
        return _transform_point(self.T_cam2body, point_cam)

    @staticmethod
    def _valid_detection(detection):
        if not isinstance(detection, dict):
            return False
        required = ("x1", "y1", "x2", "y2", "score")
        try:
            values = np.asarray(
                [detection[key] for key in required], dtype=np.float64
            )
        except (KeyError, TypeError, ValueError):
            return False
        return bool(
            np.all(np.isfinite(values))
            and values[2] > values[0]
            and values[3] > values[1]
        )

    def _detections_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            detections = json.loads(msg.data)
            if not isinstance(detections, list) or len(detections) == 0:
                return

            valid_detections = [
                detection
                for detection in detections
                if self._valid_detection(detection)
            ]
            if not valid_detections:
                return

            best = max(valid_detections, key=lambda item: float(item["score"]))
            x1, y1, x2, y2 = (
                float(best["x1"]),
                float(best["y1"]),
                float(best["x2"]),
                float(best["y2"]),
            )
            x_center = (x1 + x2) / 2.0
            y_center = (y1 + y2) / 2.0
            width = x2 - x1
            if width <= 0.0:
                return

            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            depth = (fx * self.tennis_diameter_m) / width
            if not np.isfinite(depth) or depth <= 0.0:
                return

            point_body = self.transform_point(
                [
                    (x_center - cx) * depth / fx,
                    (y_center - cy) * depth / fy,
                    depth,
                ]
            )
            if point_body is None:
                self.get_logger().warning(
                    "Invalid point or hand-eye matrix; target was not published"
                )
                return

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = float(point_body[0])
            pose_msg.pose.position.y = float(point_body[1])
            pose_msg.pose.position.z = float(point_body[2])
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 1.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0
            self.target_pose_pub.publish(pose_msg)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().error(f"坐标变换输入无效，已忽略: {exc}")
        except Exception as exc:
            self.get_logger().error(f"坐标变换失败: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
