"""
Environment Manager for DQN Navigation.

Provides ROS2 interface to Stage simulator:
- Subscribes to /scan, /odom
- Publishes to /cmd_vel
- Manages state representation, actions, and rewards
"""

import numpy as np
import math
from typing import Tuple, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty
import time

import tf_transformations


class EnvironmentManager:
    """
    Manages the environment interface between DQN agent and Stage simulator.
    
    State Space (24 dimensions):
    - 20 laser scan readings (normalized)
    - Distance to goal (normalized)
    - Angle to goal (normalized to [-1, 1])
    - Current linear velocity
    - Current angular velocity
    
    Action Space (5 discrete actions):
    - 0: Forward
    - 1: Turn left
    - 2: Turn right
    - 3: Hard left (rotate in place)
    - 4: Hard right (rotate in place)
    """
    
    # Action definitions (linear_vel, angular_vel)
    ACTIONS = {
        0: (0.4, 0.0),    # Forward
        1: (0.2, 0.5),    # Left
        2: (0.2, -0.5),   # Right
        3: (0.0, 1.0),    # Hard left (rotate in place)
        4: (0.0, -1.0),   # Hard right (rotate in place)
        5: (-0.2, 0.0),   # Backup (reverse)
    }
    
    def __init__(
        self,
        node: Node,
        goal_radius: float = 0.3,
        collision_distance: float = 0.25,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 1.5,
        num_laser_samples: int = 20,
        max_laser_range: float = 10.0,
        map_size: float = 10.0
    ):
        """
        Initialize the environment manager.
        
        Args:
            node: ROS2 node for subscriptions/publications
            goal_radius: Distance threshold to consider goal reached
            collision_distance: Distance threshold for collision detection
            max_linear_vel: Maximum linear velocity
            max_angular_vel: Maximum angular velocity
            num_laser_samples: Number of laser readings to use in state
            max_laser_range: Maximum laser range for normalization
            map_size: Approximate map size for distance normalization
        """
        self.node = node
        self.goal_radius = goal_radius
        self.collision_distance = collision_distance
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.num_laser_samples = num_laser_samples
        self.max_laser_range = max_laser_range
        self.map_size = map_size
        
        # State variables
        self.laser_data: Optional[np.ndarray] = None
        self.robot_position = np.array([0.0, 0.0])
        self.robot_yaw = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Goal position
        self.goal_position = np.array([0.0, 0.0])
        self.previous_distance_to_goal = None
        
        # Episode tracking
        self.step_count = 0
        self.max_steps = 500
        
        # QoS profile for sensor data
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Subscribers
        self.laser_sub = node.create_subscription(
            LaserScan, '/scan', self._laser_callback, qos)
        self.odom_sub = node.create_subscription(
            Odometry, '/odom', self._odom_callback, qos)
        
        # Publisher
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Pose publisher for reset (try different topics that Stage might support)
        self.pose_pub = node.create_publisher(PoseStamped, '/cmd_pose', 10)
        self.initial_pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Reset position (center of map)
        self.reset_position = np.array([0.0, 0.0])
        
        # State size for DQN
        self.state_size = num_laser_samples + 4  # laser + dist + angle + vels
        self.action_size = len(self.ACTIONS)
        
        self.node.get_logger().info(
            f"Environment initialized: state_size={self.state_size}, "
            f"action_size={self.action_size}"
        )
    
    def _laser_callback(self, msg: LaserScan) -> None:
        """Process incoming laser scan data."""
        ranges = np.array(msg.ranges)
        # Replace inf and nan with max range
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), 
                         self.max_laser_range, ranges)
        # Clip to max range
        ranges = np.clip(ranges, 0, self.max_laser_range)
        self.laser_data = ranges
    
    def _odom_callback(self, msg: Odometry) -> None:
        """Process incoming odometry data."""
        # Extract position
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.robot_yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        # Extract velocities
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z
    
    def _process_laser_scan(self) -> np.ndarray:
        """
        Process laser scan into fixed-size representation.
        
        Returns:
            Array of num_laser_samples normalized readings
        """
        if self.laser_data is None:
            return np.ones(self.num_laser_samples)
        
        # Uniformly sample from laser readings
        num_readings = len(self.laser_data)
        indices = np.linspace(0, num_readings - 1, self.num_laser_samples, dtype=int)
        sampled = self.laser_data[indices]
        
        # Normalize to [0, 1]
        normalized = sampled / self.max_laser_range
        return normalized
    
    def get_state(self) -> np.ndarray:
        """
        Construct the state vector for DQN.
        
        Returns:
            State vector of size state_size
        """
        # Laser readings (20 values, normalized)
        laser = self._process_laser_scan()
        
        # Distance to goal (normalized by map size)
        distance = np.linalg.norm(self.goal_position - self.robot_position)
        distance_normalized = min(distance / self.map_size, 1.0)
        
        # Angle to goal (normalized to [-1, 1])
        dx = self.goal_position[0] - self.robot_position[0]
        dy = self.goal_position[1] - self.robot_position[1]
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - self.robot_yaw
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        angle_normalized = angle_diff / math.pi
        
        # Velocities (normalized)
        linear_vel_normalized = self.current_linear_vel / self.max_linear_vel
        angular_vel_normalized = self.current_angular_vel / self.max_angular_vel
        
        state = np.concatenate([
            laser,
            [distance_normalized, angle_normalized, 
             linear_vel_normalized, angular_vel_normalized]
        ])
        
        return state.astype(np.float32)
    
    def step(self, action: int) -> Tuple[np.ndarray, float, bool]:
        """
        Execute an action and return the result.
        
        Args:
            action: Action index to execute
            
        Returns:
            Tuple of (next_state, reward, done)
        """
        self.step_count += 1
        
        # Get velocities for this action
        linear_vel, angular_vel = self.ACTIONS[action]
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        # Wait for state update (multiple spins for stability)
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        
        # Get new state
        next_state = self.get_state()
        
        # Calculate reward and check termination
        reward, done = self._calculate_reward()
        
        return next_state, reward, done
    
    def _calculate_reward(self) -> Tuple[float, bool]:
        """
        Calculate reward based on current state.
        
        Returns:
            Tuple of (reward, done)
        """
        distance_to_goal = np.linalg.norm(self.goal_position - self.robot_position)
        
        # Check for goal reached
        if distance_to_goal < self.goal_radius:
            self.node.get_logger().info("Goal reached!")
            return 100.0, True
        
        # Check for collision (min laser reading)
        if self.laser_data is not None:
            min_laser = np.min(self.laser_data)
            if min_laser < self.collision_distance:
                self.node.get_logger().info(f"Collision! min_laser={min_laser:.2f}")
                return -100.0, True
        
        # Check for max steps
        if self.step_count >= self.max_steps:
            self.node.get_logger().info("Max steps reached")
            return -50.0, True
        
        # Obstacle proximity penalty (encourage staying away from walls)
        if self.laser_data is not None:
            min_laser = np.min(self.laser_data)
            if min_laser < 0.5:  # Getting too close to obstacles
                proximity_penalty = (0.5 - min_laser) * 2.0  # Up to -1.0
                # Don't end episode, but penalize
            else:
                proximity_penalty = 0.0
        else:
            proximity_penalty = 0.0
        
        # Distance-based reward
        reward = 0.0
        if self.previous_distance_to_goal is not None:
            distance_change = self.previous_distance_to_goal - distance_to_goal
            reward += distance_change * 10.0  # Reward for getting closer
        
        self.previous_distance_to_goal = distance_to_goal
        
        # Small time penalty to encourage efficiency
        reward -= 0.1
        
        # Bonus for facing the goal
        dx = self.goal_position[0] - self.robot_position[0]
        dy = self.goal_position[1] - self.robot_position[1]
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = abs(angle_to_goal - self.robot_yaw)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        heading_reward = (1.0 - angle_diff / math.pi) * 0.1
        reward += heading_reward
        
        # Subtract proximity penalty
        reward -= proximity_penalty
        
        return reward, False
    
    def reset(self, goal_x: Optional[float] = None, 
              goal_y: Optional[float] = None) -> np.ndarray:
        """
        Reset the environment for a new episode.
        
        Since Stage doesn't support teleportation, we perform an aggressive
        "soft reset" by backing up and rotating until the robot is safe.
        Also attempts to publish pose reset.
        
        Args:
            goal_x: X coordinate for goal (random if None)
            goal_y: Y coordinate for goal (random if None)
            
        Returns:
            Initial state
        """
        self.step_count = 0
        self.previous_distance_to_goal = None
        
        # Stop the robot first
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        
        # Try to publish pose reset (some Stage versions support this)
        self._try_pose_reset()
        
        # AGGRESSIVE ESCAPE LOOP - keep trying until robot is safe
        max_escape_attempts = 10
        escape_attempt = 0
        safe_distance = 0.4  # Minimum safe distance from obstacles
        
        while escape_attempt < max_escape_attempts:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.laser_data is None:
                break
                
            min_laser = np.min(self.laser_data)
            
            if min_laser >= safe_distance:
                self.node.get_logger().info(
                    f"Robot is safe (min_laser={min_laser:.2f}m)")
                break
            
            self.node.get_logger().info(
                f"Escape attempt {escape_attempt+1}/{max_escape_attempts}, "
                f"min_laser={min_laser:.2f}m")
            
            # Backup phase
            self.node.get_logger().info("Backing up...")
            for _ in range(20):  # ~2 seconds backup
                cmd = Twist()
                cmd.linear.x = -0.4  # Faster backup
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.1)
            
            # Check if backup helped
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.laser_data is not None and np.min(self.laser_data) >= safe_distance:
                break
            
            # Rotation phase - rotate 90-180 degrees
            rotation_direction = 1.0 if np.random.random() > 0.5 else -1.0
            rotation_steps = np.random.randint(15, 30)  # Variable rotation
            
            self.node.get_logger().info(f"Rotating {rotation_steps * 0.1:.1f}s...")
            for _ in range(rotation_steps):
                cmd = Twist()
                cmd.angular.z = rotation_direction * 1.2  # Fast rotation
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self.node, timeout_sec=0.1)
            
            escape_attempt += 1
        
        # Final stop
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        
        # Set goal position
        if goal_x is not None and goal_y is not None:
            self.goal_position = np.array([goal_x, goal_y])
        else:
            # Random goal within map bounds (with margin)
            margin = 2.0
            self.goal_position = np.random.uniform(
                -self.map_size/2 + margin, 
                self.map_size/2 - margin, 
                size=2
            )
        
        # Wait for sensor data to stabilize
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        robot_x, robot_y = self.robot_position[0], self.robot_position[1]
        dist = np.linalg.norm(self.goal_position - self.robot_position)
        
        self.node.get_logger().info(
            f"Episode reset complete. Robot: ({robot_x:.2f}, {robot_y:.2f}), "
            f"Goal: ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f}), "
            f"Dist: {dist:.2f}m"
        )
        
        return self.get_state()
    
    def _try_pose_reset(self) -> None:
        """
        Attempt to reset robot position via pose topics.
        Stage may or may not support these.
        """
        try:
            # Try PoseStamped to /cmd_pose
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "odom"
            pose_msg.header.stamp = self.node.get_clock().now().to_msg()
            pose_msg.pose.position.x = self.reset_position[0]
            pose_msg.pose.position.y = self.reset_position[1]
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)
            
            # Try PoseWithCovarianceStamped to /initialpose
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.frame_id = "map"
            initial_pose_msg.header.stamp = self.node.get_clock().now().to_msg()
            initial_pose_msg.pose.pose.position.x = self.reset_position[0]
            initial_pose_msg.pose.pose.position.y = self.reset_position[1]
            initial_pose_msg.pose.pose.orientation.w = 1.0
            self.initial_pose_pub.publish(initial_pose_msg)
            
            # Give time for the message to be processed
            rclpy.spin_once(self.node, timeout_sec=0.2)
            
        except Exception as e:
            self.node.get_logger().debug(f"Pose reset attempt failed: {e}")
    
    def stop(self) -> None:
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def get_goal_info(self) -> Tuple[float, float, float]:
        """
        Get current goal information.
        
        Returns:
            Tuple of (goal_x, goal_y, distance_to_goal)
        """
        distance = np.linalg.norm(self.goal_position - self.robot_position)
        return self.goal_position[0], self.goal_position[1], distance
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """
        Get current robot pose.
        
        Returns:
            Tuple of (x, y, yaw)
        """
        return self.robot_position[0], self.robot_position[1], self.robot_yaw
