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

        # 1. 모델 경로 설정 (setup.py에서 복사된 위치를 자동으로 찾음)
        package_share_directory = get_package_share_directory('perception')
        model_path = os.path.join(package_share_directory, 'weights', 'best(1).pt')

        

        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            # 모델 로드 실패 시 안전장치 (기본 모델 사용)
            self.model = YOLO("yolov8n.pt")

        # 2. Publishers
        self.pub_image = self.create_publisher(Image, '/camera/detections/image', 10)
        self.pub_label = self.create_publisher(String, '/detections/labels', 10)
        self.pub_dist = self.create_publisher(Float32, '/detections/distance', 10)
        self.pub_speech = self.create_publisher(String, '/robot_dog/speech', 10)

        # 3. Subscribers (Time Synchronizer)
        self.br = CvBridge()
        
        # 토픽 이름 확인 필수! (Depth와 Image가 짝이 맞아야 함)
        rgb_sub = message_filters.Subscriber(self, Image, '/camera_top/image')
        depth_sub = message_filters.Subscriber(self, Image, '/camera_top/depth')

        # 두 이미지의 시간을 맞춰주는 동기화 도구 (slop=0.1초 오차 허용)
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.listener_callback)
        
        self.get_logger().info("Perception Node Started! Waiting for images...")

    def listener_callback(self, rgb_msg, depth_msg):
        # 1. 이미지 변환
        try:
            frame = self.br.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_frame = self.br.imgmsg_to_cv2(depth_msg, "32FC1") # 미터 단위 (Float32)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        img_h, img_w, _ = frame.shape

        # 2. YOLO 추론
        results = self.model(frame, verbose=False)
        
        # 기본값 설정
        final_label = "None"
        final_dist = 0.0
        speech_cmd = "None"
        
        # 감지된 물체 처리
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # 좌표 및 정보 추출
                coords = box.xyxy[0].cpu().numpy()
                
                # 2. 정수형(int)으로 명확하게 변환
                x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
                pt1 = (x1, y1)
                pt2 = (x2, y2)
                # 3. 로그로 좌표 확인 (터미널에 좌표가 찍히는지 보세요!)
                self.get_logger().info(f"Drawing Box at: {pt1} ~ {pt2}")                
                cls=int(box.cls[0])           
                label_name = self.model.names[cls] # 예: fresh_apple, rotten_banana

                # 중심점 계산
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # 3. 거리 측정 (Depth 이미지에서 중심점의 깊이값 읽기)
                # 좌표가 이미지 밖으로 나가지 않게 보호
                cx_safe = np.clip(cx, 0, img_w - 1)
                cy_safe = np.clip(cy, 0, img_h - 1)
                
                # ★★★ [수정된 부분 시작] ★★★
                # Depth 값 읽기
                raw_dist = depth_frame[cy_safe, cx_safe]

                # 거리값이 이상하면(NaN, Inf) 무시하지 말고 0.0으로 처리!
                if np.isnan(raw_dist) or np.isinf(raw_dist):
                    real_dist = 0.0
                    dist_str = "??"
                else:
                    real_dist = float(raw_dist)
                    dist_str = f"{real_dist:.2f}m"
                # ★★★ [수정된 부분 끝] ★★★
                # 4. 판단 로직 (Rule)
                # A. 거리 조건: 3m 이내
                is_close = real_dist <= 3.0
                
                # B. 위치 조건: 가로의 가운데 3/5 영역 (0.2 ~ 0.8)
                left_limit = img_w * 0.2
                right_limit = img_w * 0.8
                is_centered = left_limit < cx < right_limit
                
                # C. 종류 조건: 'fresh'가 들어간 이름만 식용 (fresh_apple 등)
                is_edible = "good" in label_name

                # 시각화 (박스 그리기)
                if is_edible:
                    box_color = (0, 255, 0)  # 초록색 (Green)
                else:
                    box_color = (0, 0, 255)  # 빨간색 (Red)

                # 표시할 텍스트 설정 (라벨 이름 + 거리)
                label_text = f"{label_name} {dist_str}"

                # 시각화 (박스 및 텍스트 그리기)
                try:
                    # 결정된 색상(box_color)으로 박스 그리기
                    cv2.rectangle(frame, pt1, pt2, box_color, 3)
                    # 결정된 색상과 내용으로 글씨 쓰기
                    cv2.putText(frame, label_text, (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                except Exception as e:
                    self.get_logger().error(f"Drawing Error: {e}")

                # ★ BARK 조건 체크 ★
                if is_edible and is_close and is_centered:
                    speech_cmd = "bark"
                    final_label = label_name
                    final_dist = float(real_dist)
                 
      
        # 5. 결과 발행 (Publish)
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
