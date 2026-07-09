import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_description():
    perception_only = LaunchConfiguration('perception_only', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('perception_only', default_value='false'),

        # 1. 相机（✅ 加了 output="screen"）
        Node(
            package="v4l2_camera", executable="v4l2_camera_node", name="body_camera",
            output="screen",
            parameters=[{"video_device": "/dev/video2", "image_size": [1280, 720],"camera_info_url":"file:///home/lxc/ost.yaml"}],
            remappings=[("/image_raw", "/camera/body/image_raw"), ("/camera_info", "/camera/body/camera_info")]
        ),

        # 2. YOLO 检测
        Node(
            package="pine_harvester", executable="yolo_detector", name="yolo_detector",
            output="screen", parameters=[{"debug_viz": True}]
        ),

        # 3. 坐标变换
        Node(
            package="pine_harvester", executable="coordinate_transform", name="coordinate_transform",
            output="screen",
            parameters=[{"calib_path":"/home/lxc/hand_eye_result(1).json",
            		"depth_mode":"aruco_ref",
            		"fallback_depth":0.4,
            		"aruco_depth_offset_m": -0.035 }]
        ),

        # 4. 可视化诊断窗口
        Node(
            package="pine_harvester", executable="debug_perception_viewer", name="debug_viewer",
            output="screen"
        ),

        # 5. 机械臂驱动
        Node(
            package="pine_harvester", executable="so101_arm_interface",
            name="so101_arm_interface", output="screen",
            condition=UnlessCondition(perception_only),
            parameters=[{"port": "/dev/ttyACM0", "urdf_path": "/opt/models/so101_new_calib.urdf"}]
        ),

        # 6. 状态机
        Node(
            package="pine_harvester", executable="harvest_state_machine",
            name="harvest_state_machine", output="screen",
            condition=UnlessCondition(perception_only),
            parameters=[{"ground_test_mode": True, "aruco_test_mode": False}],
            remappings=[("/target/pose","/target/best")]
        ),
        
        # 7. ArUco 识别 / 手眼标定节点
        Node(
            package="pine_harvester", executable="hand_eye_calibration",
            name="hand_eye_calibration", output="screen",
            parameters=[{
                "aruco_tracking": False,  # 🔥 绝对核心：剥夺它发布 target_best 的权力！
                "calib_save_path": "/home/lxc/hand_eye_result(1).json" # 🔥 强行统一让它用正确的标定文件
            }]
        ),
    ])
