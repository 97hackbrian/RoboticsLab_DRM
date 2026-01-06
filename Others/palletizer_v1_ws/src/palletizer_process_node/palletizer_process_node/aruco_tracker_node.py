#!/usr/bin/env python3
"""
ArUco Tracker Node for Palletizer

Detects and tracks ArUco markers using the left camera.
Based on tracking_aruco.py from robot_control_nodes.

Uses DICT_4X4_50 ArUco dictionary (same as the box marker).

Author: hackbrian
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import cv2
import numpy as np


class ArucoTrackerNode(Node):
    """
    ArUco marker detection and angular tracking node.
    
    Subscribes to camera images and publishes angular velocity
    commands to keep the ArUco marker centered in the frame.
    """

    def __init__(self):
        super().__init__('aruco_tracker_node')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/left/image_raw')
        self.declare_parameter('angular_kp', 2.0)
        self.declare_parameter('max_angular_vel', 0.6)
        self.declare_parameter('show_image', True)
        
        camera_topic = self.get_parameter('camera_topic').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.show_image = self.get_parameter('show_image').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector (DICT_4X4_50 - same as box marker)
        # Using older OpenCV API for compatibility with ROS2 Humble
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Image properties (initialized on first frame)
        self.center_img = None
        self.img_width = None
        
        # Detection status
        self.aruco_detected = False
        self.last_detection_time = self.get_clock().now()
        
        # --- Publishers ---
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/palletizer/aruco_cmd_vel', 10
        )
        self.detected_pub = self.create_publisher(
            Bool, '/palletizer/aruco_detected', 10
        )
        
        # --- Subscribers ---
        self.img_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )
        
        # Detection timeout timer (check for lost ArUco)
        self.timeout_timer = self.create_timer(0.1, self.check_detection_timeout)
        
        self.get_logger().info(f'ArUco Tracker initialized on {camera_topic}')

    def image_callback(self, msg: Image):
        """Process camera images for ArUco detection."""
        try:
            # Convert ROS Image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        # Initialize image center on first frame
        if self.center_img is None:
            height, width = cv_img.shape[:2]
            self.center_img = np.array([width / 2.0, height / 2.0])
            self.img_width = width
            self.get_logger().info(f'Image center: {self.center_img}')
        
        # Detect ArUco markers (older OpenCV API)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_img, self.aruco_dict, parameters=self.aruco_params
        )
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        
        detected = Bool()
        
        if ids is not None and len(ids) > 0:
            # ArUco detected
            self.aruco_detected = True
            self.last_detection_time = self.get_clock().now()
            detected.data = True
            
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)
            
            # Get first marker's center
            marker_corners = corners[0][0]
            marker_center = np.mean(marker_corners, axis=0)
            
            # Calculate normalized error (-1 to 1)
            error = (marker_center[0] - self.center_img[0]) / self.img_width
            
            # Proportional control
            angular_vel = -self.angular_kp * error
            angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
            
            cmd.twist.angular.z = angular_vel
            
            # Draw tracking info
            cv2.circle(cv_img, tuple(marker_center.astype(int)), 10, (0, 255, 0), -1)
            cv2.putText(cv_img, f'Error: {error:.2f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(cv_img, f'ID: {ids[0][0]}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # No ArUco detected
            detected.data = False
            cmd.twist.angular.z = 0.0
        
        # Publish commands
        self.cmd_pub.publish(cmd)
        self.detected_pub.publish(detected)
        
        # Show image if enabled
        if self.show_image:
            cv2.imshow('ArUco Tracker', cv_img)
            cv2.waitKey(1)

    def check_detection_timeout(self):
        """Check if ArUco was lost (no detection for 0.5s)."""
        if self.aruco_detected:
            elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if elapsed > 0.5:
                self.aruco_detected = False
                detected = Bool()
                detected.data = False
                self.detected_pub.publish(detected)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
