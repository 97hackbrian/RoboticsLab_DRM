#!/usr/bin/env python3
"""
Potential Field Navigation Node

Implements reactive navigation using potential field method:
- Attractive force: pulls robot toward goal
- Repulsive force: pushes robot away from obstacles
- Net force: guides robot motion
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

try:
    from tf_transformations import euler_from_quaternion
except ImportError:
    # Fallback if tf_transformations not installed
    def euler_from_quaternion(q):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        x, y, z, w = q
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


class PotentialFieldNode(Node):
    """
    ROS2 node implementing Potential Field Navigation.
    
    Subscriptions:
        /scan (LaserScan): Lidar data for obstacle detection
        /odom (Odometry): Robot position and orientation
        /goal_pose (PoseStamped): Dynamic goal from RViz2
    
    Publications:
        /cmd_vel (Twist): Velocity commands
    """

    def __init__(self):
        super().__init__('potential_field_node')

        # Declare parameters
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('k_attractive', 1.0)
        self.declare_parameter('k_repulsive', 0.3)
        self.declare_parameter('repulsive_range', 0.5)
        self.declare_parameter('obstacle_threshold', 0.15)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 2.84)

        # Get parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.k_attractive = self.get_parameter('k_attractive').value
        self.k_repulsive = self.get_parameter('k_repulsive').value
        self.repulsive_range = self.get_parameter('repulsive_range').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # State variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.laser_data = None
        self.goal_reached = False
        self.has_goal = True  # Start with YAML goal

        # QoS for sensor data
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Potential Field Navigation started. Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def scan_callback(self, msg: LaserScan):
        """Store laser scan data for obstacle detection."""
        self.laser_data = msg

    def odom_callback(self, msg: Odometry):
        """Update robot position and orientation from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def goal_callback(self, msg: PoseStamped):
        """Update goal from RViz2 or external source."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_reached = False
        self.has_goal = True
        
        self.get_logger().info(
            f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def calculate_attractive_force(self):
        """
        Calculate attractive force toward goal.
        
        F_att = k_att * (goal - robot)
        Direction: toward goal
        """
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.01:  # Avoid division by zero
            return 0.0, 0.0

        # F_att = k_att * distance
        magnitude = self.k_attractive * distance

        # Direction: toward goal (normalized)
        fx = magnitude * (dx / distance)
        fy = magnitude * (dy / distance)

        return fx, fy

    def calculate_repulsive_force(self):
        """
        Calculate repulsive force from obstacles.
        
        F_rep = k_rep * (1/d - 1/ρ) / d² for each obstacle
        Direction: away from obstacle
        """
        if self.laser_data is None:
            return 0.0, 0.0

        fx_total = 0.0
        fy_total = 0.0
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        closest_obstacle = float('inf')

        # Iterate through all laser readings
        for i, distance in enumerate(self.laser_data.ranges):
            # Skip invalid readings
            if distance < self.laser_data.range_min or \
               distance > self.laser_data.range_max or \
               math.isinf(distance) or math.isnan(distance):
                continue

            # Track closest obstacle for speed control
            if distance < closest_obstacle:
                closest_obstacle = distance

            # Only consider obstacles within repulsive range
            if distance > self.repulsive_range:
                continue

            # Convert to Cartesian (robot frame)
            angle = angle_min + i * angle_increment
            obs_x = distance * math.cos(angle)
            obs_y = distance * math.sin(angle)

            obs_dist = math.sqrt(obs_x**2 + obs_y**2)

            if obs_dist < 0.01:  # Avoid division by zero
                continue

            # F_rep = k_rep * (1/obs_dist - 1/ρ) / obs_dist²
            magnitude = self.k_repulsive * \
                       (1.0 / obs_dist - 1.0 / self.repulsive_range) / \
                       (obs_dist ** 2)
            magnitude = max(0, magnitude)  # Only repulsive

            # Force direction: away from obstacle
            fx_total += magnitude * (-obs_x / obs_dist)
            fy_total += magnitude * (-obs_y / obs_dist)

        return fx_total, fy_total, closest_obstacle

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        """Main control loop: combine forces and compute velocity commands."""
        cmd = Twist()

        # Check if we have a goal
        if not self.has_goal:
            self.cmd_pub.publish(cmd)
            return

        # Check if goal reached
        distance_to_goal = math.sqrt(
            (self.goal_x - self.robot_x)**2 + 
            (self.goal_y - self.robot_y)**2)

        if distance_to_goal < self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info('Goal reached!')
                self.goal_reached = True
            self.cmd_pub.publish(cmd)
            return

        # Calculate forces
        fx_att, fy_att = self.calculate_attractive_force()
        
        rep_result = self.calculate_repulsive_force()
        fx_rep, fy_rep = rep_result[0], rep_result[1]
        closest_obstacle = rep_result[2] if len(rep_result) > 2 else float('inf')

        # Combine forces (superposition principle)
        fx_total = fx_att + fx_rep
        fy_total = fy_att + fy_rep

        # Calculate desired direction from combined forces
        force_magnitude = math.sqrt(fx_total**2 + fy_total**2)

        if force_magnitude < 0.01:  # No net force
            desired_angle = self.robot_yaw
        else:
            desired_angle = math.atan2(fy_total, fx_total)

        # Calculate steering angle error
        angle_error = self.normalize_angle(desired_angle - self.robot_yaw)

        # Angular velocity (proportional control)
        angular_vel = 2.0 * angle_error
        angular_vel = max(-self.max_angular_speed, 
                         min(self.max_angular_speed, angular_vel))

        # Linear velocity with obstacle proximity scaling
        if closest_obstacle < self.obstacle_threshold:
            speed_factor = closest_obstacle / self.obstacle_threshold
            speed_factor = max(0.1, speed_factor)  # Minimum 10%
        else:
            speed_factor = 1.0

        linear_vel = self.max_linear_speed * speed_factor

        # Reduce speed when turning sharply
        turn_factor = 1.0 - (abs(angle_error) / math.pi) * 0.7
        linear_vel *= turn_factor

        linear_vel = max(self.min_linear_speed, linear_vel)

        # Set velocity command
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        self.cmd_pub.publish(cmd)

        # Debug output (throttled)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0

        if self._log_counter % 20 == 0:  # Every 2 seconds
            self.get_logger().info(
                f'Pos: ({self.robot_x:.2f}, {self.robot_y:.2f}) | '
                f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | '
                f'Dist: {distance_to_goal:.2f}m | '
                f'V: {linear_vel:.2f} ω: {angular_vel:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
