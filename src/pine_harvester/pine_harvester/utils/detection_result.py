"""
detection_result.py
检测结果数据类（替代自定义消息，开发阶段用Python dataclass）

生产环境建议替换为真正的 ROS2 自定义消息包 pine_harvester_msgs
"""

from dataclasses import dataclass, field
from std_msgs.msg import Header


@dataclass
class DetectionResult:
    """单个检测结果"""
    x1:          float = 0.0    # 边界框左上角 x
    y1:          float = 0.0    # 边界框左上角 y
    x2:          float = 0.0    # 边界框右下角 x
    y2:          float = 0.0    # 边界框右下角 y
    cx:          float = 0.0    # 中心点 x
    cy:          float = 0.0    # 中心点 y
    confidence:  float = 0.0    # 置信度
    class_id:    int   = 0      # 类别 ID
    class_name:  str   = ""     # 类别名称


class DetectionArray:
    """
    检测结果数组（模拟ROS2消息）
    生产环境请替换为自定义 msg 文件
    """
    def __init__(self):
        self.header:   Header             = Header()
        self.items:    list[DetectionResult] = []
        self.infer_ms: float              = 0.0
