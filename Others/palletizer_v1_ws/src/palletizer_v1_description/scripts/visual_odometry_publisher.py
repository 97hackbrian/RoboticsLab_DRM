#!/usr/bin/env python3
"""
Visual Odometry Publisher Node
Converts camera_link TF to Odometry message for visual odometry tracking
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
from tf2_ros import TransformException
import math


def quaternion_multiply(q1, q2):
    """Multiply two quaternions"""
    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
    
    result = Quaternion()
    result.w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    result.x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    result.y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    result.z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return result


class VisualOdometryPublisher(Node):
    def __init__(self):
        super().__init__('visual_odometry_publisher')
        
        # Parameters
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 30.0)
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Odometry Publisher
        self.odom_pub = self.create_publisher(Odometry, '/camera/odom', 10)
        
        # Rotation offset: -90 degrees around Z axis
        # q = [x, y, z, w] for rotation around Z by -90°
        self.z_rotation_90 = Quaternion()
        self.z_rotation_90.x = 0.0
        self.z_rotation_90.y = 0.0
        self.z_rotation_90.z = -0.7071067811865476  # sin(-45°)
        self.z_rotation_90.w = 0.7071067811865476   # cos(45°)
        
        # Timer to publish odometry
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odometry)
        
        self.get_logger().info(f'Visual Odometry Publisher started')
        self.get_logger().info(f'Publishing {self.camera_frame} pose as odometry at {publish_rate} Hz')
        self.get_logger().info(f'Applying -90° rotation in Z axis')
        
    def publish_odometry(self):
        try:
            # Lookup transform from odom to camera_link
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.camera_frame,
                rclpy.time.Time()
            )
            
            # Create Odometry message
            odom = Odometry()
            odom.header.stamp = transform.header.stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.camera_frame
            
            # Set position from transform
            odom.pose.pose.position.x = transform.transform.translation.x
            odom.pose.pose.position.y = transform.transform.translation.y
            odom.pose.pose.position.z = transform.transform.translation.z
            
            # Apply 90° Z rotation to orientation
            # q_final = q_original * q_z90
            odom.pose.pose.orientation = quaternion_multiply(
                transform.transform.rotation,
                self.z_rotation_90
            )
            
            # Publish
            self.odom_pub.publish(odom)
            
        except TransformException as ex:
            # Only log occasionally to avoid spam
            if not hasattr(self, '_last_error_time') or \
               (self.get_clock().now() - self._last_error_time).nanoseconds > 5e9:
                self.get_logger().warn(f'Could not get transform: {ex}')
                self._last_error_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
