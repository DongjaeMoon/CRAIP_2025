#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray
from cv_bridge import CvBridge
import message_filters

import cv2
from ultralytics import YOLO
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


def robust_depth_m(depth_img: np.ndarray, cx: int, cy: int, r: int = 3) -> float:
    h, w = depth_img.shape[:2]
    x1 = max(cx - r, 0); x2 = min(cx + r + 1, w)
    y1 = max(cy - r, 0); y2 = min(cy + r + 1, h)

    roi = depth_img[y1:y2, x1:x2].astype(np.float32).reshape(-1)
    roi = roi[np.isfinite(roi)]
    roi = roi[roi > 0.0]
    if roi.size == 0:
        return 0.0
    return float(np.median(roi))


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # =========================
        # Model
        # =========================
        pkg_dir = get_package_share_directory('perception')
        model_path = os.path.join(pkg_dir, 'weights', 'best(1).pt')

        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.model = YOLO("yolov8n.pt")

        self.get_logger().info(f"Model classes: {self.model.names}")

        # =========================
        # Publishers
        # =========================
        self.pub_image      = self.create_publisher(Image,  '/camera/detections/image', 10)
        self.pub_labels     = self.create_publisher(String, '/detections/labels', 10)                 # csv
        self.pub_distances  = self.create_publisher(Float32MultiArray, '/detections/distances', 10)   # all
        self.pub_dist_near  = self.create_publisher(Float32, '/detections/distance', 10)              # nearest 1
        self.pub_speech     = self.create_publisher(String, '/robot_dog/speech', 10)
        # [NEW] 중심점 좌표(cx) 발행 추가
        self.pub_centers    = self.create_publisher(Float32MultiArray, '/detections/centers', 10)

        # =========================
        # Subscribers
        # =========================
        self.br = CvBridge()
        rgb_sub   = message_filters.Subscriber(self, Image, '/camera_top/image')
        depth_sub = message_filters.Subscriber(self, Image, '/camera_top/depth')

        ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.listener_callback)

        # =========================
        # Labels to export
        # =========================
        self.TARGET_LABELS = {"box", "sign", "nurse", "red_cone", "green_cone", "blue_cone"}
        self.FOOD_LABELS   = {
            "good_pizza", "bad_pizza",
            "good_banana", "bad_banana",
            "good_apple", "bad_apple",
        }

        self.get_logger().info("Perception Node Started.")

    def listener_callback(self, rgb_msg, depth_msg):
        centers_all = []
        try:
            frame = self.br.imgmsg_to_cv2(rgb_msg, "bgr8")
            # depth 인코딩이 32FC1이 아닐 수도 있어서 passthrough로 받고, 아래에서 처리
            depth = self.br.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        h, w, _ = frame.shape
        results = self.model(frame, verbose=False)

        speech_cmd = "None"
        candidates = []   # (sort_dist, label, dist_m)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                # (optional) 피자만 이름 바꿔치기 유지 (필요 없으면 삭제 가능)
                if label == "bad_pizza":
                    label = "good_pizza"
                elif label == "good_pizza":
                    label = "bad_pizza"

                cx = int((x1 + x2) // 2)
                cy = int((y1 + y2) // 2)
                cx = int(np.clip(cx, 0, w - 1))
                cy = int(np.clip(cy, 0, h - 1))

                # =========================
                # depth -> meter
                # =========================
                dist = 0.0

                # depth dtype에 따라 처리
                # - float 계열이면 보통 meter(32FC1)
                # - uint16이면 보통 mm(16UC1)
                if depth.dtype == np.uint16:
                    dist_mm = robust_depth_m(depth.astype(np.float32), cx, cy, r=3)
                    dist = float(dist_mm) / 1000.0
                else:
                    dist = robust_depth_m(depth.astype(np.float32), cx, cy, r=3)

                if dist <= 0.0 or (not np.isfinite(dist)):
                    dist = 0.0
                    dist_str = "??"
                else:
                    dist_str = f"{dist:.2f}m"

                # visualize
                is_edible = ("good" in label.lower())
                color = (0, 255, 0) if is_edible else (0, 0, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                cv2.putText(frame, f"{label} {dist_str}",
                            (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # collect candidates (타겟 + 음식)
                if (label in self.TARGET_LABELS) or (label in self.FOOD_LABELS):
                    sort_dist = dist if dist > 0.0 else float("inf")
                    candidates.append((sort_dist, label, float(dist), float(cx)))

                # bark condition (음식만)
                if label in self.FOOD_LABELS and is_edible:
                    # 3m 이내 + 가운데 3/5 (좌/우 1/5 제외)
                    is_close = (0.0 < dist <= 3.0)
                    is_centered = (w * 0.2) < cx < (w * 0.8)
                    if is_close and is_centered:
                        speech_cmd = "bark"

        # sort by distance
        candidates.sort(key=lambda x: x[0])

        # build outputs
        if candidates:
            labels_csv = ",".join([c[1] for c in candidates])
            dists_all = [float(c[2]) for c in candidates]
            # [NEW] cx 좌표 리스트 추출
            centers_all = [float(c[3]) for c in candidates]
            nearest_dist = float(candidates[0][2])
        else:
            labels_csv = "None"
            dists_all = []
            centers_all = []
            nearest_dist = 0.0

        # publish image
        self.pub_image.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))

        # publish labels csv
        m_labels = String()
        m_labels.data = labels_csv
        self.pub_labels.publish(m_labels)

        # publish distances array
        m_dists = Float32MultiArray()
        m_dists.data = dists_all
        self.pub_distances.publish(m_dists)

        # [NEW] publish centers array
        m_centers = Float32MultiArray()
        m_centers.data = centers_all
        self.pub_centers.publish(m_centers)

        # publish nearest float (호환용)
        m_near = Float32()
        m_near.data = nearest_dist
        self.pub_dist_near.publish(m_near)

        # publish speech
        m_sp = String()
        m_sp.data = speech_cmd
        self.pub_speech.publish(m_sp)


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
