#!/usr/bin/env python3
"""
Odometry TF Corrector Node

This node corrects the odometry published by RTAB-Map stereo_odometry to account for
the rotation applied in base_footprint_joint (rpy="1.5708 0 3.14159").

The problem: RTAB-Map publishes odom->base_footprint TF based on camera movement,
but doesn't account for the fixed rotation between base_footprint and base_link.
This causes the robot to appear to rise in RViz when it moves forward in Gazebo.

Solution: This node subscribes to /odom, applies the inverse rotation of the
base_footprint_joint, and publishes the corrected odom->base_footprint TF.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation


class OdomTfCorrectorNode(Node):
    def __init__(self):
        super().__init__('odom_tf_corrector')
        
        # The rotation from base_footprint_joint: rpy="1.5708 0 3.14159"
        # We need the INVERSE of this rotation to correct the odometry
        # Original: Roll=90° (1.5708), Pitch=0, Yaw=180° (3.14159)
        self.base_footprint_rpy = np.array([1.5708, 0.0, 3.14159])
        
        # Create the inverse rotation matrix
        self.base_footprint_rotation = Rotation.from_euler('xyz', self.base_footprint_rpy)
        self.inverse_rotation = self.base_footprint_rotation.inv()
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry from RTAB-Map
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for corrected odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom_corrected',
            10
        )
        
        self.get_logger().info('Odometry TF Corrector initialized')
        self.get_logger().info(f'Applying inverse rotation for base_footprint_joint: rpy={self.base_footprint_rpy}')
    
    def odom_callback(self, msg: Odometry):
        """Process incoming odometry and publish corrected TF."""
        
        # Extract position from odometry
        pos = msg.pose.pose.position
        original_position = np.array([pos.x, pos.y, pos.z])
        
        # Extract orientation from odometry
        quat = msg.pose.pose.orientation
        original_orientation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        
        # Apply the inverse rotation to the position
        # The position needs to be rotated by the inverse of base_footprint_joint rotation
        corrected_position = self.inverse_rotation.apply(original_position)
        
        # Apply the inverse rotation to the orientation
        # New orientation = inverse_rotation * original_orientation
        corrected_orientation = self.inverse_rotation * original_orientation
        corrected_quat = corrected_orientation.as_quat()  # [x, y, z, w]
        
        # Create and publish TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = corrected_position[0]
        t.transform.translation.y = corrected_position[1]
        t.transform.translation.z = corrected_position[2]
        
        t.transform.rotation.x = corrected_quat[0]
        t.transform.rotation.y = corrected_quat[1]
        t.transform.rotation.z = corrected_quat[2]
        t.transform.rotation.w = corrected_quat[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Also publish corrected odometry message
        corrected_odom = Odometry()
        corrected_odom.header = msg.header
        corrected_odom.header.frame_id = 'odom'
        corrected_odom.child_frame_id = 'base_footprint'
        
        corrected_odom.pose.pose.position.x = corrected_position[0]
        corrected_odom.pose.pose.position.y = corrected_position[1]
        corrected_odom.pose.pose.position.z = corrected_position[2]
        
        corrected_odom.pose.pose.orientation.x = corrected_quat[0]
        corrected_odom.pose.pose.orientation.y = corrected_quat[1]
        corrected_odom.pose.pose.orientation.z = corrected_quat[2]
        corrected_odom.pose.pose.orientation.w = corrected_quat[3]
        
        # Copy covariance (not transformed, but should be acceptable for visualization)
        corrected_odom.pose.covariance = msg.pose.covariance
        corrected_odom.twist = msg.twist  # Twist is in child frame, keep as-is for now
        
        self.odom_pub.publish(corrected_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfCorrectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
