#!/bin/bash
# 松果采摘机器人一键启动脚本（带环境保护）

echo "🌲 松果采摘机器人启动程序..."
echo "="*60

# 1. 环境检查与保护
echo "[1/5] 检查环境..."
NUMPY_VERSION=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null)
if [ "$NUMPY_VERSION" != "1.24.4" ]; then
    echo "⚠️  发现 NumPy 版本异常 ($NUMPY_VERSION)，正在强制恢复..."
    pip3 install "numpy==1.24.4" --force-reinstall --no-deps
else
    echo "✅ NumPy 版本正确: 1.24.4"
fi

# 2. 进入工作空间
echo "[2/5] 进入工作空间..."
cd ~/ros2_ws

# 3. 刷新环境
echo "[3/5] 刷新 ROS2 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. 检查设备权限
echo "[4/5] 检查设备权限..."
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 666 /dev/ttyACM0
    echo "✅ 机械臂串口权限已设置"
else
    echo "⚠️  未检测到机械臂串口 /dev/ttyACM0"
fi

if [ -e /dev/video0 ]; then
    sudo chmod 666 /dev/video0
    echo "✅ 相机权限已设置"
fi

# 5. 启动系统
echo "[5/5] 启动采摘系统..."
echo "="*60
echo ""
echo "启动命令："
echo "  地面测试: ros2 launch pine_harvester harvest_system.launch.py aruco_test_mode:=true ground_test_mode:=true arm_port:=/dev/ttyACM0 camera_device:=/dev/video0"
echo ""
echo "按 Ctrl+C 退出"
echo ""

# 启动（默认进入地面测试模式）
ros2 launch pine_harvester harvest_system.launch.py \
    aruco_test_mode:=true \
    ground_test_mode:=true \
    arm_port:=/dev/ttyACM0 \
    camera_device:=/dev/video0
