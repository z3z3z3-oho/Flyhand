import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2

class PerceptionDebugger(Node):
    def __init__(self):
        super().__init__('perception_debugger')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.current_coord = "Waiting for Target..."
        
        # 订阅 YOLO 渲染后的图像
        self.create_subscription(Image, '/camera/body/image_detections', self.img_callback, 10)
        # 订阅坐标转换后的 3D 坐标
        self.create_subscription(PoseStamped, '/target/best', self.coord_callback, 10)
        
        self.get_logger().info("🔍 诊断窗口已就绪，正在等待感知数据...")

    def coord_callback(self, msg):
        p = msg.pose.position
        self.current_coord = f"XYZ: [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]"

    def img_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # 在画面左上角绘制大号文字
        cv2.rectangle(self.latest_frame, (5, 5), (450, 80), (0,0,0), -1) # 黑底
        cv2.putText(self.latest_frame, "DEBUG INFO:", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(self.latest_frame, self.current_coord, (15, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        cv2.imshow("Harvest System Debugger", self.latest_frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(PerceptionDebugger())
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
