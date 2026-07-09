import json
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


PKG_SHARE = get_package_share_directory("pine_harvester")


def _as_transform_matrix(value):
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


def _rotation_matrix_to_quaternion(matrix):
    """Convert a finite 3x3 rotation matrix to ROS x/y/z/w quaternion."""
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("旋转矩阵必须是有限的 3x3 数组")

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale

    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm <= np.finfo(np.float64).eps:
        raise ValueError("无法从旋转矩阵生成有效四元数")
    return quaternion / norm


def _create_aruco_components(aruco_module):
    """Create dictionary/parameters/detector across old and new OpenCV APIs."""
    if hasattr(aruco_module, "getPredefinedDictionary"):
        dictionary = aruco_module.getPredefinedDictionary(
            aruco_module.DICT_4X4_50
        )
    elif hasattr(aruco_module, "Dictionary_get"):
        dictionary = aruco_module.Dictionary_get(aruco_module.DICT_4X4_50)
    else:
        raise RuntimeError("当前 OpenCV 缺少 ArUco 字典 API")

    if hasattr(aruco_module, "DetectorParameters"):
        parameters = aruco_module.DetectorParameters()
    elif hasattr(aruco_module, "DetectorParameters_create"):
        parameters = aruco_module.DetectorParameters_create()
    else:
        raise RuntimeError("当前 OpenCV 缺少 ArUco 检测参数 API")

    detector = None
    if hasattr(aruco_module, "ArucoDetector"):
        detector = aruco_module.ArucoDetector(dictionary, parameters)
    elif not hasattr(aruco_module, "detectMarkers"):
        raise RuntimeError("当前 OpenCV 缺少 ArUco 检测 API")
    return dictionary, parameters, detector


class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("hand_eye_calibration")
        self.declare_parameter(
            "camera_matrix_path",
            os.path.join(PKG_SHARE, "calib", "camera_intrinsics.json"),
        )
        self.declare_parameter(
            "calib_save_path",
            os.path.join(PKG_SHARE, "calib", "hand_eye_result(1).json"),
        )
        self.declare_parameter("aruco_tracking", False)
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("aruco_depth_offset_m", 0.0)
        self.declare_parameter("debug_viz", True)

        self.camera_matrix_path = self.get_parameter("camera_matrix_path").value
        self.calib_save_path = self.get_parameter("calib_save_path").value
        self.aruco_tracking = bool(self.get_parameter("aruco_tracking").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.aruco_depth_offset_m = float(
            self.get_parameter("aruco_depth_offset_m").value
        )
        self.debug_viz = bool(self.get_parameter("debug_viz").value)

        if self.marker_size <= 0.0:
            raise ValueError("marker_size 必须大于 0")

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.T_cam2body = np.eye(4, dtype=np.float64)
        self._viz_failed = False
        self._load_camera_intrinsics()
        self._load_hand_eye_result()

        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "当前 OpenCV 未包含 aruco 模块，请安装 opencv-contrib-python"
            )
        (
            self.aruco_dict,
            self.aruco_params,
            self.aruco_detector,
        ) = _create_aruco_components(cv2.aruco)

        self.aruco_pose_pub = self.create_publisher(
            PoseStamped, "/aruco/pose", 10
        )
        self.image_sub = self.create_subscription(
            Image, "/camera/body/image_raw", self._image_callback, 10
        )
        self.calib_service = self.create_service(
            Trigger, "/hand_eye/calibrate", self._calibrate_callback
        )

        api_name = "ArucoDetector" if self.aruco_detector is not None else "detectMarkers"
        self.get_logger().info(
            f"Hand-eye ready | marker_id={self.marker_id} | "
            f"marker_size={self.marker_size * 100:.1f} cm | "
            f"tracking={'ON' if self.aruco_tracking else 'OFF'} | API={api_name}"
        )

    def _load_camera_intrinsics(self):
        try:
            with open(self.camera_matrix_path, "r", encoding="utf-8") as file:
                data = json.load(file)
            camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
            dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1)
            if camera_matrix.shape != (3, 3):
                raise ValueError(f"camera_matrix 形状错误: {camera_matrix.shape}")
            if not np.all(np.isfinite(camera_matrix)) or not np.all(
                np.isfinite(dist_coeffs)
            ):
                raise ValueError("相机内参包含 NaN 或无穷大")
            if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
                raise ValueError("相机焦距 fx/fy 必须大于 0")
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            self.get_logger().info(
                f"Camera intrinsics loaded: {self.camera_matrix_path}"
            )
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"Camera intrinsics load failed ({exc}); using fallback values"
            )
            self.camera_matrix = np.array(
                [[894.4, 0.0, 626.6], [0.0, 984.3, 199.0], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            self.dist_coeffs = np.array(
                [0.647466, -0.367587, -0.173382, -0.000737, 0.0],
                dtype=np.float64,
            )

    def _load_hand_eye_result(self):
        if not os.path.exists(self.calib_save_path):
            self.get_logger().warning(
                f"No calibration file at {self.calib_save_path}; using identity"
            )
            return
        try:
            with open(self.calib_save_path, "r", encoding="utf-8") as file:
                self.T_cam2body = _as_transform_matrix(json.load(file))
            self.get_logger().info(
                f"T_cam2body loaded from {self.calib_save_path}"
            )
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.T_cam2body = np.eye(4, dtype=np.float64)
            self.get_logger().error(
                f"Invalid hand-eye calibration; using identity: {exc}"
            )

    def _detect_markers(self, gray):
        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

    def _estimate_marker_pose(self, marker_corners):
        half = self.marker_size / 2.0
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )
        image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
        flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=flag,
        )
        if not success:
            return None, None
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)):
            return None, None
        return rvec, tvec

    def _image_callback(self, msg):
        if not self.aruco_tracking:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._detect_markers(gray)

            if ids is not None:
                flat_ids = np.asarray(ids).reshape(-1)
                matches = np.flatnonzero(flat_ids == self.marker_id)
                if matches.size:
                    index = int(matches[0])
                    rvec, tvec = self._estimate_marker_pose(corners[index])
                    if rvec is not None:
                        rotation, _ = cv2.Rodrigues(rvec)
                        camera_to_marker = np.eye(4, dtype=np.float64)
                        camera_to_marker[:3, :3] = rotation
                        camera_to_marker[:3, 3] = tvec
                        body_to_marker = self.T_cam2body @ camera_to_marker
                        if not np.all(np.isfinite(body_to_marker)):
                            raise ValueError("变换后的 ArUco 位姿包含无效数值")

                        quaternion = _rotation_matrix_to_quaternion(
                            body_to_marker[:3, :3]
                        )
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = msg.header.stamp
                        pose_msg.header.frame_id = "base_link"
                        pose_msg.pose.orientation.x = float(quaternion[0])
                        pose_msg.pose.orientation.y = float(quaternion[1])
                        pose_msg.pose.orientation.z = float(quaternion[2])
                        pose_msg.pose.orientation.w = float(quaternion[3])
                        pose_msg.pose.position.x = float(body_to_marker[0, 3])
                        pose_msg.pose.position.y = float(body_to_marker[1, 3])
                        pose_msg.pose.position.z = float(
                            body_to_marker[2, 3] + self.aruco_depth_offset_m
                        )
                        self.aruco_pose_pub.publish(pose_msg)

                        if self.debug_viz and not self._viz_failed:
                            cv2.drawFrameAxes(
                                cv_image,
                                self.camera_matrix,
                                self.dist_coeffs,
                                rvec,
                                tvec.reshape(3, 1),
                                self.marker_size,
                            )

            if self.debug_viz and not self._viz_failed:
                try:
                    cv2.imshow("ArUco Tracking", cv_image)
                    cv2.waitKey(1)
                except cv2.error as exc:
                    self._viz_failed = True
                    self.get_logger().warning(
                        f"Debug window unavailable; display disabled: {exc}"
                    )
        except (cv2.error, TypeError, ValueError) as exc:
            self.get_logger().error(f"ArUco detection failed: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Unexpected ArUco error: {exc}")

    def _calibrate_callback(self, request, response):
        response.success = False
        response.message = "Calibration collection is not implemented in this node"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
