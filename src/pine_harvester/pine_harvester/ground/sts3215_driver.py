import serial
import struct
import threading
import time
import numpy as np

class STS3215Driver:
    def __init__(self, port="/dev/ttyACM0", baudrate=1000000):
        try:
            # 修复 1：增加超时时间到 0.05s，给硬件留出反应余地
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            self.lock = threading.Lock()
            print(f"✅ 串口已连接: {port} (1M Baud)")
        except Exception as e:
            print(f"❌ 串口打开失败: {e}")
            self.ser = None

    def checksum(self, data):
        return (~sum(data)) & 0xFF

    def read_pos_deg(self, id):
        if not self.ser: return None
        with self.lock:
            try:
                # 修复 2：删掉 reset_input_buffer()，防止误删正在传输的后续电机数据
                payload = [id, 0x04, 0x02, 0x38, 0x02]
                chk = self.checksum(payload)
                self.ser.write(bytes([0xFF, 0xFF] + payload + [chk]))
                
                # 修复 3：给予物理层极短的传输等待
                time.sleep(0.002)
                
                # 修复 4：鲁棒性读取。不再死等 8 字节，而是读取所有可用字节并寻找报头
                waiting = self.ser.in_waiting
                res = self.ser.read(waiting if waiting > 8 else 8)
                
                if len(res) >= 8:
                    # 在字节流中搜索 0xFF 0xFF [ID] 的报头
                    for i in range(len(res) - 7):
                        if res[i] == 0xFF and res[i+1] == 0xFF and res[i+2] == id:
                            val = res[i+5] + (res[i+6] << 8)
                            return (val - 2048) * (360.0 / 4096.0)
            except Exception:
                pass
        return None

    def write_pos_deg(self, id, pos_deg, speed: int = 0):
        """
        写目标位置，同时设置速度上限。
        一次写 6 字节（0x2A~0x2F）：
          0x2A-0x2B  Goal Position
          0x2C-0x2D  Running Time  (0 = 不限时间)
          0x2E-0x2F  Running Speed (0 = 全速，越大越慢)
        speed=300 约等于 1.5s 走完 90°，是推荐的调试值。
        """
        if not self.ser:
            return
        with self.lock:
            try:
                pos_raw = int(float(pos_deg) * (4096.0 / 360.0) + 2048)
                pos_raw = max(0, min(4095, pos_raw))
                pos_l, pos_h   = pos_raw & 0xFF, (pos_raw >> 8) & 0xFF
                time_l, time_h = 0, 0                         # 不限时间
                sp_l,   sp_h   = speed & 0xFF, (speed >> 8) & 0xFF
                data    = [pos_l, pos_h, time_l, time_h, sp_l, sp_h]
                payload = [id, len(data) + 3, 0x03, 0x2A] + data
                chk     = self.checksum(payload)
                self.ser.write(bytes([0xFF, 0xFF] + payload + [chk]))
            except Exception:
                pass

class SO101Follower:
    def __init__(self, config):
        self.driver = STS3215Driver(config.port)
        self.motor_names = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
        self.motor_ids = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
        # 速度寄存器 0x2E：0=全速，越大越慢（推荐调试值 300，正式采摘 150）
        self.body_speed    = getattr(config, 'body_speed',    300)
        self.gripper_speed = getattr(config, 'gripper_speed', 200)

    def connect(self):
        if self.driver.ser is None: raise Exception("Serial port error")

    def get_observation(self):
        obs = {}
        for name in self.motor_names:
            motor_id = self.motor_ids[name]
            pos = self.driver.read_pos_deg(motor_id)
            if pos is not None:
                obs[f"{name}.pos"] = pos
            # 修复 5：每个电机读取间隙增加极微小延迟，防止总线拥堵
            time.sleep(0.001)
        return obs

    def set_action(self, action_dict):
        """
        发送关节目标角度（含速度限制）。
        兼容两种 key 格式："shoulder_pan" 或 "shoulder_pan.pos"
        """
        if not self.driver.ser:
            return
        for raw_name, target_deg in action_dict.items():
            name = raw_name[:-4] if raw_name.endswith(".pos") else raw_name
            if name not in self.motor_ids:
                continue
            try:
                val   = float(target_deg.item() if hasattr(target_deg, 'item') else target_deg)
                speed = self.gripper_speed if name == "gripper" else self.body_speed
                self.driver.write_pos_deg(self.motor_ids[name], val, speed=speed)
            except (TypeError, ValueError):
                pass

    def disconnect(self):
        if self.driver.ser: self.driver.ser.close()
