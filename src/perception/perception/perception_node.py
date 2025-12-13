import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import message_filters
import cv2
from ultralytics import YOLO
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # 1. 모델 경로 설정
        package_share_directory = get_package_share_directory('perception')
        model_path = os.path.join(package_share_directory, 'weights', 'best(1).pt')

        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.model = YOLO("yolov8n.pt")

        # 2. Publishers
        self.pub_image = self.create_publisher(Image, '/camera/detections/image', 10)
        self.pub_label = self.create_publisher(String, '/detections/labels', 10)
        self.pub_dist = self.create_publisher(Float32, '/detections/distance', 10)
        self.pub_speech = self.create_publisher(String, '/robot_dog/speech', 10)

        # 3. Subscribers
        self.br = CvBridge()
        
        rgb_sub = message_filters.Subscriber(self, Image, '/camera_top/image')
        depth_sub = message_filters.Subscriber(self, Image, '/camera_top/depth')

        # 시간 동기화 (slop=1.0으로 설정하여 끊김 방지)
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 1.0)
        ts.registerCallback(self.listener_callback)
        
        self.get_logger().info("Perception Node Started! Waiting for images...")

    def listener_callback(self, rgb_msg, depth_msg):
        try:
            frame = self.br.imgmsg_to_cv2(rgb_msg, "bgr8")
            # 메모리 정렬
            frame = np.ascontiguousarray(frame)
            depth_frame = self.br.imgmsg_to_cv2(depth_msg, "32FC1") 
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        img_h, img_w, _ = frame.shape

        # YOLO 추론
        results = self.model(frame, verbose=False)
        
        final_label = "None"
        final_dist = 0.0
        speech_cmd = "None"
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                coords = box.xyxy[0].cpu().numpy()
                x1, y1 = int(coords[0]), int(coords[1])
                x2, y2 = int(coords[2]), int(coords[3])
                pt1 = (x1, y1)
                pt2 = (x2, y2)

                cls = int(box.cls[0])           
                label_name = self.model.names[cls] 

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                cx_safe = np.clip(cx, 0, img_w - 1)
                cy_safe = np.clip(cy, 0, img_h - 1)
                
                # 거리 측정
                raw_dist = depth_frame[cy_safe, cx_safe]

                if np.isnan(raw_dist) or np.isinf(raw_dist):
                    real_dist = 0.0
                    dist_str = "??"
                else:
                    real_dist = float(raw_dist)
                    dist_str = f"{real_dist:.2f}m"

                # ★★★ [수정된 판단 로직] ★★★
                # C. 식용 여부 판단 (Pizza 특수 규칙 적용)
                if label_name == "bad_pizza":
                    # 나쁜 피자는 먹는다 (초록색)
                    is_edible = True
                elif label_name == "good_pizza":
                    # 좋은 피자는 안 먹는다 (빨간색)
                    is_edible = False
                else:
                    # 나머지는 이름에 'good'이나 'fresh'가 있으면 식용
                    is_edible = ("good" in label_name) or ("fresh" in label_name)

                # 시각화 색상 결정
                if is_edible:
                    box_color = (0, 255, 0)  # 초록색 (Green)
                else:
                    box_color = (0, 0, 255)  # 빨간색 (Red)

                label_text = f"{label_name} {dist_str}"

                try:
                    cv2.rectangle(frame, pt1, pt2, box_color, 3)
                    cv2.putText(frame, label_text, (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                except Exception as e:
                    self.get_logger().error(f"Drawing Error: {e}")

                # A. 거리 조건: 3m 이내
                is_close = (real_dist > 0.1) and (real_dist <= 3.0)
                
                # B. 위치 조건: 중앙
                left_limit = img_w * 0.2
                right_limit = img_w * 0.8
                is_centered = left_limit < cx < right_limit
                
                # ★ BARK 조건 체크 ★ (식용이고 + 가깝고 + 중앙이면)
                if is_edible and is_close and is_centered:
                    speech_cmd = "bark"
                    final_label = label_name
                    final_dist = float(real_dist)
                    
                    cv2.putText(frame, "BARK!!!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                                1.5, (0, 255, 255), 3)

        self.pub_image.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
        
        msg_label = String()
        msg_label.data = final_label
        self.pub_label.publish(msg_label)

        msg_dist = Float32()
        msg_dist.data = final_dist
        self.pub_dist.publish(msg_dist)

        msg_speech = String()
        msg_speech.data = speech_cmd
        self.pub_speech.publish(msg_speech)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()