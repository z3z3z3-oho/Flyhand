from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    perception_only = LaunchConfiguration("perception_only")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("perception_only", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("serial_baudrate", default_value="57600"),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="body_camera",
                output="screen",
                parameters=[
                    {
                        "video_device": "/dev/video20",
                        "image_size": [1280, 720],
                        "camera_info_url": "file:///home/coolpi/ost.yaml",
                    }
                ],
                remappings=[
                    ("/image_raw", "/camera/body/image_raw"),
                    ("/camera_info", "/camera/body/camera_info"),
                ],
            ),
            Node(
                package="pine_harvester",
                executable="yolo_detector",
                name="yolo_detector",
                output="screen",
                parameters=[{"debug_viz": True}],
            ),
            Node(
                package="pine_harvester",
                executable="coordinate_transform",
                name="coordinate_transform",
                output="screen",
                parameters=[
                    {
                        "calib_path": "/home/coolpi/hand_eye_result(1).json",
                        "depth_mode": "aruco_ref",
                        "fallback_depth": 0.4,
                        "aruco_depth_offset_m": -0.035,
                    }
                ],
            ),
            Node(
                package="pine_harvester",
                executable="debug_perception_viewer",
                name="debug_viewer",
                output="screen",
            ),
            Node(
                package="pine_harvester",
                executable="so101_arm_interface",
                name="so101_arm_interface",
                output="screen",
                condition=UnlessCondition(perception_only),
                parameters=[
                    {
                        "port": "/dev/ttyACM0",
                        "urdf_path": "/opt/models/so101_new_calib.urdf",
                        "calibration_path": "/home/coolpi/ros2_ws/src/pine_harvester/pine_harvester/config/drone_follower.json",
                    }
                ],
            ),
            Node(
                package="pine_harvester",
                executable="harvest_state_machine",
                name="harvest_state_machine",
                output="screen",
                condition=UnlessCondition(perception_only),
                parameters=[{"ground_test_mode": False, "aruco_mode": False}],
                remappings=[("/target/pose", "/target/best")],
            ),
            Node(
                package="pine_harvester",
                executable="serial_command_bridge",
                name="serial_command_bridge",
                output="screen",
                condition=UnlessCondition(perception_only),
                parameters=[
                    {
                        "port": serial_port,
                        "baudrate": ParameterValue(
                            serial_baudrate, value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="pine_harvester",
                executable="hand_eye_calibration",
                name="hand_eye_calibration",
                output="screen",
                parameters=[
                    {
                        "aruco_tracking": False,
                        "calib_save_path": "/home/coolpi/hand_eye_result(1).json",
                    }
                ],
            ),
        ]
    )
