from setuptools import setup
import os
from glob import glob

package_name = "pine_harvester"

setup(
    name=package_name,
    version="0.0.0",
    # 【核心修改 1】：添加了 f"{package_name}.utils"，确保识别工具包
    packages=[package_name, f"{package_name}.nodes", f"{package_name}.utils"],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "calib"), glob("calib/*")),
        # 如果你的 assets 文件夹里有 STL 文件，建议也加上这一行：
        (os.path.join("share", package_name, "assets"), glob("assets/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lxc",
    maintainer_email="lxc@todo.todo",
    description="SO101 采摘机器人集成包",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "so101_arm_interface = pine_harvester.nodes.so101_arm_interface:main",
            "hand_eye_calibration = pine_harvester.nodes.hand_eye_calibration:main",
            "harvest_state_machine = pine_harvester.nodes.harvest_state_machine:main",
            # 【核心修正】：点(.)改为冒号(:)
            "yolo_detector = pine_harvester.nodes.yolo_detector:main", 
            "coordinate_transform = pine_harvester.nodes.coordinate_transform:main",
            "visual_servo = pine_harvester.nodes.visual_servo:main",
            "debug_perception_viewer = pine_harvester.nodes.debug_perception_viewer:main",
            "gesture_control_node = pine_harvester.nodes.gesture_control_node:main",
            "serial_forward_node = pine_harvester.nodes.serial_forward_node:main",
        ],
    },
)
