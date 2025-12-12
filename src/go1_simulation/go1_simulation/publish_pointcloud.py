#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
import struct
from rclpy.time import Time
import time
from std_msgs.msg import Bool


class CameraHandler:
    """
    This class handles point cloud generation for a single camera.
    It subscribes to the camera info, image, and depth topics, and publishes the point cloud to the '/points' topic.
    For the transformation from camera frame to map frame (fixed world origin), set proper 'msg.header.frame_id'.
    """
    
    def __init__(self, node, camera_name):
        f"""
        Initialize camera handler for a specific camera.
        Colored point cloud is published to the '/"camera_name"/points' topic.
        
        Args:
            node: Parent ROS2 node
            camera_name: Name of the camera (e.g., 'face', 'top')
        """
        self.node = node
        self.camera_name = camera_name
        # Camera fram is differnt from camera optical frame because of image transformation convention.
        self.camera_frame = f'camera_optical_{camera_name}'
        self.bridge = CvBridge()
        self.camera_info = None
        
        # Cache for meshgrid to avoid recomputing every frame
        self.meshgrid_cache = None
        
        # Subscribe to camera info
        self.camera_info_sub = self.node.create_subscription(
            CameraInfo,
            f'camera_{camera_name}/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Synchronized subscribers for image and depth
        # Because RGB and depth images are might be captured at different times, we use approximate time synchronizer.
        self.image_sub = Subscriber(self.node, Image, f'camera_{camera_name}/image')
        self.depth_sub = Subscriber(self.node, Image, f'camera_{camera_name}/depth')
        
        # Synchronize image and depth messages
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
            # slop: Maximum time difference between the RGB and depth images timestamp in seconds.
            # RGB t=1.00 + Depth t=1.02 → Difference = 0.02s ✓ (< 0.1s, MATCH!)
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Publisher for point cloud
        self.pointcloud_pub = self.node.create_publisher(
            PointCloud2,
            f'camera_{camera_name}/points',
            10
        )
        # [수정 2] Face 카메라일 경우에만 Blind 감지 Publisher 생성
        self.blind_pub = None
        if self.camera_name == 'face':
            self.blind_pub = self.node.create_publisher(Bool, '/is_blind', 1)

        # Print information about the node
        self.node.get_logger().info(
            f'Camera handler initialized for {camera_name} camera '
            f'(frame: {self.camera_frame})'
        )
    

    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        # Store camera intrinsics for later use (Update only once)
        if self.camera_info is None:
            self.camera_info = msg
            self.node.get_logger().info(f'Camera info received for {self.camera_name}')
    

    def sync_callback(self, image_msg, depth_msg):
        """Process synchronized image and depth messages"""
        if self.camera_info is None:
            self.node.get_logger().warn(
                f'Waiting for {self.camera_name} camera info...', 
                throttle_duration_sec=1.0
            )
            return
        
        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Convert RGB image to numpy array
            rgb_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
            
            # Create point cloud
            pointcloud_msg = self.create_pointcloud(depth_image, rgb_image, depth_msg.header)
            
            # Publish point cloud
            self.pointcloud_pub.publish(pointcloud_msg)
            
        except Exception as e:
            self.node.get_logger().error(
                f'Error processing images for {self.camera_name}: {str(e)}'
            )
    

    def create_pointcloud(self, depth_image, rgb_image, header):
        """
        Create PointCloud2 message from RGB and depth images.
        
        Converts 2D RGB and depth images into 3D colored points using camera intrinsics,
        
        Args:
            depth_image: Depth image as numpy array (meters)
            rgb_image: RGB image as numpy array
            header: Original message header (timestamp will be preserved)
            
        Returns:
            PointCloud2 message with XYZRGB points in target frame
        """
        
        # Get camera intrinsics
        fx = self.camera_info.k[0]  # focal length x
        fy = self.camera_info.k[4]  # focal length y
        cx = self.camera_info.k[2]  # principal point x
        cy = self.camera_info.k[5]  # principal point y
        
        height, width = depth_image.shape
        
        # Convert depth to meters if needed (In our case, depth is in meters already)
        # In our case, depth image is in the camera optical frame (You can visualize it in Rviz: RGB arrow<->XYZ coordinate).
        # x-axis: Represents the forward (viewing) direction, pointing from the camera's center out through the lens.
        # z-axis: Represents the up direction, pointing from the camera's center toward the top (e.g., toward a physical button).
        x = depth_image.astype(np.float32)
        
        # Filter out invalid depth values (zero, negative, inf, nan)
        valid_mask = (x > 0) & (x < np.inf) & np.isfinite(x)
        
        # Check if there are any valid points
        if not np.any(valid_mask):
            self.node.get_logger().warn(
                f'No valid depth values in {self.camera_name} image', 
                throttle_duration_sec=1.0
            )
            # [수정 3] Face 카메라인데 유효한 값이 하나도 없다 -> 벽에 박음 (True 발행)
            if self.blind_pub is not None:
                blind_msg = Bool()
                blind_msg.data = True
                self.blind_pub.publish(blind_msg)

            # Return empty point cloud
            msg = PointCloud2()
            msg.header = header
            msg.header.frame_id = self.camera_frame
            msg.height = 1
            msg.width = 0
            msg.is_dense = False
            msg.is_bigendian = False
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step = 0
            msg.data = bytes()
            return msg
        
        # [수정 4] 유효한 값이 있다 -> 앞이 보임 (False 발행)
        if self.blind_pub is not None:
            blind_msg = Bool()
            blind_msg.data = False
            self.blind_pub.publish(blind_msg)
        
        # If there are valid depth values, create point cloud
        # Create pixel-wise coordinate arrays (W x H) - cached for performance
        if self.meshgrid_cache is None or self.meshgrid_cache[0].shape != (height, width):
            u = np.arange(width)
            v = np.arange(height)
            u, v = np.meshgrid(u, v)
            self.meshgrid_cache = (u, v)
        else:
            u, v = self.meshgrid_cache

        # u: shape (H, W) <- right-side up
        # array([[  0,   1,   2, ..., W-1],
        #        [  0,   1,   2, ..., W-1],
        #        ...,
        #        [  0,   1,   2, ..., W-1]]
        # v: shape (H, W) <- bottom-side up
        # array([[  0,   0,   0, ...,   0],
        #        [  1,   1,   1, ...,   1],
        #        ...,
        #        [H-1, H-1, H-1, ..., H-1]]
        
        # Extract only valid depth values and corresponding coordinates
        x_valid = x[valid_mask]
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]
        
        # Compute 3D coordinates in camera frame (only for valid points)
        # x-axis: Represents the forward (viewing) direction, pointing from the camera's center out through the lens.
        # z-axis: Represents the up direction, pointing from the camera's center toward the top (e.g., toward a physical button).        
        x = x_valid
        y = - (u_valid - cx) * x_valid / fx
        z = - (v_valid - cy) * x_valid / fy     

        # Get RGB values for valid points 
        rgb_valid = rgb_image[valid_mask]  # Shape: (N, 3)
        r = rgb_valid[:, 0]
        g = rgb_valid[:, 1]
        b = rgb_valid[:, 2]
        
        # Pack RGB into single float 
        rgb_int = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
        rgb = rgb_int.view(np.float32)
        
        # Create point cloud data
        points = np.zeros(len(x), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.float32)
        ])
        
        points['x'] = x
        points['y'] = y
        points['z'] = z
        points['rgb'] = rgb
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = self.camera_frame
        msg.height = 1
        msg.width = len(points)
        # is_dense: If True, the point cloud is dense (i.e., all points are valid).
        msg.is_dense = False
        # is_bigendian: If True, the point cloud is in big endian format.
        msg.is_bigendian = False
        
        # Define fields
        # offset: Offset in bytes from the beginning of the point data. (0-3: x, 4-7: y, 8-11: z, 12-15: rgb)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # point_step: Size of one point in bytes (16 bytes for x, y, z, rgb with 4 bytes each)
        msg.point_step = 16
        # row_step: Size of one row in bytes (point_step * width)
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        
        return msg


class PointCloudPublisher(Node):
    """Main node that manages point cloud publishers for face and top cameras"""
    
    def __init__(self):
        super().__init__('pointcloud_publisher')
        
        # Create handlers for both face and top cameras
        self.face_camera_handler = CameraHandler(self, 'face')
        self.top_camera_handler = CameraHandler(self, 'top')
        
        self.get_logger().info('PointCloud publisher node initialized for face and top cameras')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()