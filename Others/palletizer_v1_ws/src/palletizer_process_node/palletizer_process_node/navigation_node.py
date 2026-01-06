#!/usr/bin/env python3
"""
Navigation Node for Palletizer

Simple point-to-point navigation using /odom_vo for position tracking.
Uses proportional control to navigate toward goal positions.

Author: hackbrian
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import numpy as np


class NavigationNode(Node):
    """
    Point-to-point navigation node using visual odometry.
    
    Subscribes to /odom_vo and publishes velocity commands
    to navigate toward a goal position.
    """

    def __init__(self):
        super().__init__('navigation_node')
        
        # Parameters
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('max_angular_vel', 0.8)
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Status
        self.goal_reached = False
        
        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/palletizer/nav_cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/palletizer/goal_reached', 10)
        
        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_vo', self.odom_callback, 10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/palletizer/goal', self.goal_callback, 10
        )
        
        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'Navigation node initialized. Goal: ({self.goal_x}, {self.goal_y})')

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        """Update current position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def goal_callback(self, msg: PoseStamped):
        """Update goal position."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_reached = False
        self.get_logger().info(f'New goal received: ({self.goal_x}, {self.goal_y})')

    def control_loop(self):
        """Proportional control to navigate toward goal."""
        cmd = Twist()
        
        # Calculate error to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info('Goal reached!')
            
            # Publish goal reached status
            status = Bool()
            status.data = True
            self.goal_reached_pub.publish(status)
            
            # Stop robot
            self.cmd_pub.publish(cmd)
            return
        
        # Publish goal not reached
        status = Bool()
        status.data = False
        self.goal_reached_pub.publish(status)
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.current_yaw
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Proportional control
        cmd.angular.z = np.clip(
            self.angular_kp * angle_error,
            -self.max_angular_vel,
            self.max_angular_vel
        )
        
        # Only move forward if roughly facing the goal
        if abs(angle_error) < math.pi / 4:  # Within 45 degrees
            cmd.linear.x = np.clip(
                self.linear_kp * distance,
                0.0,
                self.max_linear_vel
            )
        else:
            # Rotate in place first
            cmd.linear.x = 0.0
        
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
