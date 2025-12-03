import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # 1. 구독할 카메라 토픽 (문제지 요구사항)
        self.subscription = self.create_subscription(
            Image,
            '/camera_top/image',
            self.listener_callback,
            10)
        
        self.br = CvBridge()
        
        # 2. 이미지가 저장될 위치 설정 (실행하는 위치 기준 dataset/images)
        self.save_dir = 'dataset/images'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.get_logger().info(f"Ready to save images to: {os.path.abspath(self.save_dir)}")
        self.get_logger().info("Press 's' to save, 'q' to quit.")

    def listener_callback(self, data):
        try:
            # ROS Image -> OpenCV Image 변환
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Conversion error: {str(e)}')
            return

        # 3. 화면 띄우기
        cv2.imshow("Data Collector View", current_frame)
        
        # 4. 키보드 입력 처리
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # 파일명: img_날짜_시간.jpg
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{self.save_dir}/img_{timestamp}.jpg"
            cv2.imwrite(filename, current_frame)
            self.get_logger().info(f"Saved: {filename}")
        
        elif key == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
