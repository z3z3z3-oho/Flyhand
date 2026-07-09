"""
so101_calib_tool.py
SO101 运动学参数标定工具（独立脚本，不是ROS2节点）

功能：
  通过遥操控制机械臂到已知位置，用标尺测量实际末端位置，
  与FK预测值比对，迭代修正 DH 参数

用法：
  python3 so101_calib_tool.py --port /dev/ttyACM0
"""

import argparse
import json
import numpy as np
import time

try:
    from lerobot.common.robots.so101_follower import SO101Follower
    from lerobot.common.robots.config_so101_follower import SO101FollowerConfig
except ImportError:
    print("请先安装 LeRobot: pip install lerobot[feetech]")
    exit(1)

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper"
]

# 标定测试位姿（关节角度，度）——机械臂在这些位置容易用尺量
CALIB_POSES = [
    # [pan, lift, elbow, wrist_flex, wrist_roll, gripper]
    [  0,  -90,  90,   0,  0, 0],   # 竖直向上
    [  0,    0,   0,   0,  0, 0],   # 水平伸直
    [  0,  -45,  45,   0,  0, 0],   # 45度伸出
    [ 45,  -90,  90,   0,  0, 0],   # 45度旋转向上
    [-45,  -90,  90,   0,  0, 0],   # -45度旋转向上
]


def read_joints(arm) -> np.ndarray:
    obs = arm.get_observation()
    angles = []
    for name in JOINT_NAMES:
        angles.append(float(obs.get(f"{name}.pos", 0.0)))
    return np.array(angles)


def send_joints(arm, angles_deg: list, wait=3.0):
    action = {f"{name}.pos": float(a)
              for name, a in zip(JOINT_NAMES, angles_deg)}
    arm.send_action(action)
    print(f"  → 移动到: {np.round(angles_deg, 1)}°")
    print(f"  等待 {wait}s...")
    time.sleep(wait)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--robot_id", default="calib_arm")
    parser.add_argument("--output", default="/opt/calib/so101_kinematic_params.json")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print("SO101 运动学参数标定工具")
    print(f"{'='*60}\n")

    config = SO101FollowerConfig(port=args.port, id=args.robot_id)
    arm = SO101Follower(config)
    arm.connect()
    print(f"✅ 连接成功: {args.port}\n")

    measured_positions = []
    actual_joints_list = []

    print("📏 说明：对于每个测试位姿，机械臂会自动移动到目标位置。")
    print("   请用卷尺测量末端执行器（夹爪前端）相对于机械臂底座中心的：")
    print("   X（向前）, Y（向左）, Z（向上）距离，单位：厘米\n")
    input("准备好后按 Enter 开始...\n")

    for i, pose in enumerate(CALIB_POSES):
        print(f"\n--- 位姿 {i+1}/{len(CALIB_POSES)} ---")
        send_joints(arm, pose, wait=3.0)

        # 读取实际关节角度
        actual = read_joints(arm)
        actual_joints_list.append(actual.tolist())
        print(f"  实际关节角度: {np.round(actual[:5], 1)}°")

        # 让用户输入测量值
        while True:
            try:
                raw = input("  请输入测量到的末端位置 X Y Z (cm，空格分隔): ")
                xyz = [float(v) for v in raw.strip().split()]
                if len(xyz) == 3:
                    measured_positions.append(xyz)
                    print(f"  记录: ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) cm")
                    break
                else:
                    print("  请输入3个值！")
            except ValueError:
                print("  输入格式错误，请重试")

    # 回到零位
    print("\n返回零位...")
    send_joints(arm, [0, -90, 90, 0, 0, 0], wait=2.0)
    arm.disconnect()

    # ── 输出标定数据 ──────────────────────────
    print(f"\n{'='*60}")
    print("标定数据汇总（请发给开发者用于优化DH参数）：")
    print(f"{'='*60}")
    for i, (joints, meas) in enumerate(zip(actual_joints_list, measured_positions)):
        print(f"\n位姿{i+1}:")
        print(f"  关节角度(°): {np.round(joints[:5], 2).tolist()}")
        print(f"  测量末端(cm): {meas}")

    # 保存到文件
    import os
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    data = {
        "measured_positions_cm": measured_positions,
        "actual_joint_angles_deg": actual_joints_list,
        "calib_poses_deg": CALIB_POSES,
        "note": "手动测量数据，用于修正so101_arm_interface.py中的SO101_LINK_LENGTHS"
    }
    with open(args.output, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n✅ 数据已保存到 {args.output}")
    print("\n下一步：将测量数据代入 so101_arm_interface.py 中的")
    print("SO101_LINK_LENGTHS 字典，更新 L1~L5 的值（单位：米）")


if __name__ == "__main__":
    main()
