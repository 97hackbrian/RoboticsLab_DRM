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
        self.declare_parameter('odom_frame', 'odom_vo')
        self.declare_parameter('child_frame_id', 'base_link_vo')
        self.declare_parameter('publish_rate', 30.0)
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Odometry Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom_vo', 10)
        
        # Rotation offset: -90 degrees around Z axis (to align camera frame with robot base)
        # q = [x, y, z, w] for rotation around Z by -90°
        self.z_rotation_90 = Quaternion()
        self.z_rotation_90.x = 0.0
        self.z_rotation_90.y = 0.0
        self.z_rotation_90.z = -0.7071067811865476  # sin(-45°)
        self.z_rotation_90.w = 0.7071067811865476   # cos(45°)
        
        # Timer to publish odometry
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odometry)
        
        self.get_logger().info(f'Visual Odometry Publisher started')
        self.get_logger().info(f'Publishing {self.odom_frame} -> {self.child_frame_id}')
        self.get_logger().info(f'Using source: {self.camera_frame}')
        
    def publish_odometry(self):
        try:
            # We want to publish odom_vo -> base_link_vo
            # We assume odom_wheel -> camera_link is available from the main tree + sensor
            # And we treat camera_link as the "proxy" for location, but rotated.
            
            # Lookup transform from odom_wheel (the world) to camera_link
            # Note: We use the *simulated* world frame (odom_wheel) to *simulate* the VO result.
            # In a real robot, 'odom_wheel' here would be the output of the VO algorithm relative to its start.
            source_frame = 'odom_wheel' 
            
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                source_frame,
                self.camera_frame,
                rclpy.time.Time()
            )
            
            # --- Prepare Data ---
            # 1. Position: Same as camera
            pos_x = transform.transform.translation.x
            pos_y = transform.transform.translation.y
            pos_z = transform.transform.translation.z
            
            # 2. Orientation: Camera orientation rotated by -90 Z to match robot front
            q_rot = quaternion_multiply(
                transform.transform.rotation,
                self.z_rotation_90
            )

            current_time = self.get_clock().now().to_msg()

            # --- 1. Publish TF: odom_vo -> base_link_vo ---
            t_vo = TransformStamped()
            t_vo.header.stamp = current_time
            t_vo.header.frame_id = self.odom_frame
            t_vo.child_frame_id = self.child_frame_id
            
            t_vo.transform.translation.x = pos_x
            t_vo.transform.translation.y = pos_y
            t_vo.transform.translation.z = pos_z
            t_vo.transform.rotation = q_rot
            
            self.tf_broadcaster.sendTransform(t_vo)
            
            # --- 2. Publish Odometry Message ---
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.child_frame_id
            
            odom.pose.pose.position.x = pos_x
            odom.pose.pose.position.y = pos_y
            odom.pose.pose.position.z = pos_z
            odom.pose.pose.orientation = q_rot
            
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
