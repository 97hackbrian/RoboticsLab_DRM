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
        
        # Laser data storage - copy data to avoid reference issues
        self.laser_ranges = None
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0
        self.laser_range_min = 0.0
        self.laser_range_max = 0.0
        self.laser_data_time = None
        self.last_min_range = float('inf')  # Track min range for debugging
        
        self.goal_reached = False
        self.has_goal = True  # Start with YAML goal
        self._log_counter = 0
        self._scan_counter = 0  # Count scan messages received
        self._waiting_for_new_goal = False  # Wait state after reaching goal
        self._debug_scans = 0  # Counter for debug logging after goal change
        
        # ============ LOCAL MINIMA ESCAPE SYSTEM ============
        # Position history for stuck detection
        self._position_history = []  # List of (x, y, timestamp) tuples
        self._history_max_size = 30  # Keep last 3 seconds at 10Hz
        self._stuck_threshold = 0.15  # If moved less than 15cm in 3 seconds = stuck
        
        # Escape mode state
        self._escape_mode = False
        self._escape_start_time = None
        self._escape_duration = 2.0  # Escape for 2 seconds
        self._escape_direction = 1.0  # 1.0 = left, -1.0 = right
        
        # Cooldown to avoid re-triggering escape immediately
        self._last_escape_end_time = None
        self._escape_cooldown = 3.0  # Wait 3 seconds before allowing another escape

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
        """Store laser scan data for obstacle detection - copy data explicitly."""
        # Copy data to avoid any reference issues
        self.laser_ranges = list(msg.ranges)  # Make a copy!
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment
        self.laser_range_min = msg.range_min
        self.laser_range_max = msg.range_max
        self.laser_data_time = self.get_clock().now()
        self._scan_counter += 1
        
        # Calculate min range for debugging
        valid_ranges = [r for r in self.laser_ranges 
                       if r > self.laser_range_min and r < self.laser_range_max 
                       and not math.isinf(r) and not math.isnan(r)]
        self.last_min_range = min(valid_ranges) if valid_ranges else float('inf')
        
        # Log lidar coverage info every 100 scans
        if self._scan_counter % 100 == 1:
            angle_max = msg.angle_min + msg.angle_increment * len(msg.ranges)
            fov_degrees = math.degrees(angle_max - msg.angle_min)
            self.get_logger().info(
                f'[LIDAR] {len(msg.ranges)} readings, '
                f'angle: {math.degrees(msg.angle_min):.1f}° to {math.degrees(angle_max):.1f}° '
                f'(FOV: {fov_degrees:.1f}°)')
        
        # Log when debug mode is active (after new goal)
        if self._debug_scans > 0:
            self.get_logger().info(
                f'[SCAN] #{self._scan_counter}, min_range={self.last_min_range:.2f}m')
            self._debug_scans -= 1

    def odom_callback(self, msg: Odometry):
        """Update robot position and orientation from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def reset_state(self):
        """
        COMPLETE PURGE of navigation state for a fresh start.
        Called when goal is reached or new goal arrives.
        Forces fresh lidar data to be collected.
        """
        self.goal_reached = False
        self._log_counter = 0
        self._waiting_for_new_goal = False
        
        # PURGE ALL LASER DATA - force waiting for fresh data
        self.laser_ranges = None
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0
        self.laser_range_min = 0.0
        self.laser_range_max = 0.0
        self.laser_data_time = None
        self.last_min_range = float('inf')
        
        # PURGE ESCAPE SYSTEM STATE
        self._position_history.clear()
        self._escape_mode = False
        self._escape_start_time = None
        self._last_escape_end_time = None
        
        # Stop the robot immediately
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        
        self.get_logger().info('PURGE COMPLETE: All state and laser data cleared.')

    def goal_callback(self, msg: PoseStamped):
        """Update goal from RViz2 or external source."""
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        
        self.get_logger().info(
            f'New goal received: ({new_x:.2f}, {new_y:.2f})')
        
        # Reset state completely before accepting new goal
        self.reset_state()
        
        # Enable debug logging for next 20 scans to verify lidar is working
        self._debug_scans = 20
        
        self.goal_x = new_x
        self.goal_y = new_y
        self.has_goal = True

    # ============ LOCAL MINIMA ESCAPE METHODS ============
    
    def update_position_history(self):
        """Add current position to history buffer."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self._position_history.append((self.robot_x, self.robot_y, current_time))
        
        # Remove old entries (keep only last N)
        while len(self._position_history) > self._history_max_size:
            self._position_history.pop(0)
    
    def is_stuck(self):
        """
        Detect if robot is stuck in a local minimum.
        Returns True if robot has moved less than threshold in last N seconds.
        """
        if len(self._position_history) < self._history_max_size:
            return False  # Not enough history yet
        
        # Check cooldown
        if self._last_escape_end_time is not None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_escape_end_time < self._escape_cooldown:
                return False  # Still in cooldown
        
        # Calculate distance traveled between oldest and newest positions
        oldest = self._position_history[0]
        newest = self._position_history[-1]
        
        dx = newest[0] - oldest[0]
        dy = newest[1] - oldest[1]
        distance_traveled = math.sqrt(dx**2 + dy**2)
        
        # Check if we're close enough to goal that we shouldn't trigger escape
        distance_to_goal = math.sqrt(
            (self.goal_x - self.robot_x)**2 + 
            (self.goal_y - self.robot_y)**2)
        
        if distance_to_goal < 0.3:  # Close to goal, don't escape
            return False
        
        return distance_traveled < self._stuck_threshold
    
    def find_clearest_direction(self):
        """
        Analyze laser scan to find which side has more space.
        Returns 1.0 for left (CCW), -1.0 for right (CW).
        """
        if self.laser_ranges is None or len(self.laser_ranges) == 0:
            return 1.0  # Default to left
        
        n = len(self.laser_ranges)
        quarter = n // 4
        
        # Sum distances on left side (indices 0 to n/4 = front-left to left)
        left_space = sum(
            r for r in self.laser_ranges[:quarter] 
            if r > self.laser_range_min and r < self.laser_range_max and not math.isinf(r)
        )
        
        # Sum distances on right side (indices 3n/4 to n = right to front-right)
        right_space = sum(
            r for r in self.laser_ranges[3*quarter:] 
            if r > self.laser_range_min and r < self.laser_range_max and not math.isinf(r)
        )
        
        # Return direction toward the clearer side
        return 1.0 if left_space >= right_space else -1.0
    
    def start_escape(self):
        """Initialize escape mode."""
        self._escape_mode = True
        self._escape_start_time = self.get_clock().now().nanoseconds / 1e9
        self._escape_direction = self.find_clearest_direction()
        self._position_history.clear()  # Clear history during escape
        
        direction_name = "LEFT" if self._escape_direction > 0 else "RIGHT"
        self.get_logger().warn(
            f'🚨 LOCAL MINIMUM DETECTED! Starting escape maneuver turning {direction_name}')
    
    def execute_escape(self):
        """
        Execute escape maneuver: back up while turning toward clearer side.
        Returns Twist command for escape, or None if escape is complete.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self._escape_start_time
        
        if elapsed > self._escape_duration:
            # Escape complete
            self._escape_mode = False
            self._last_escape_end_time = current_time
            self._position_history.clear()  # Reset history after escape
            self.get_logger().info('✅ Escape maneuver complete. Resuming navigation.')
            return None
        
        # Create escape command
        cmd = Twist()
        
        # Phase 1 (first half): Back up while turning
        if elapsed < self._escape_duration / 2:
            cmd.linear.x = -0.15  # Back up
            cmd.angular.z = self._escape_direction * 0.8  # Turn toward clear side
        # Phase 2 (second half): Move forward while turning more
        else:
            cmd.linear.x = 0.1  # Move forward slowly
            cmd.angular.z = self._escape_direction * 1.0  # Continue turning
        
        return cmd

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
        Direction: away from obstacle + tangential component for frontal obstacles
        
        Returns:
            tuple: (fx_total, fy_total, closest_obstacle, obstacle_in_front, obstacles_in_range)
        """
        # Always return 5 values for consistency
        if self.laser_ranges is None:
            return 0.0, 0.0, float('inf'), False, 0

        fx_total = 0.0
        fy_total = 0.0
        angle_min = self.laser_angle_min
        angle_increment = self.laser_angle_increment

        closest_obstacle = float('inf')
        closest_front_obstacle = float('inf')
        obstacle_in_front = False
        obstacles_in_range = 0

        # Iterate through all laser readings (using copied ranges)
        for i, distance in enumerate(self.laser_ranges):
            # Skip invalid readings
            if distance < self.laser_range_min or \
               distance > self.laser_range_max or \
               math.isinf(distance) or math.isnan(distance):
                continue

            # Track closest obstacle for speed control
            if distance < closest_obstacle:
                closest_obstacle = distance

            # Check if obstacle is in front (within ±45 degrees = ±0.785 rad)
            angle = angle_min + i * angle_increment
            if abs(angle) < 0.785 and distance < self.repulsive_range:
                obstacle_in_front = True
                if distance < closest_front_obstacle:
                    closest_front_obstacle = distance

            # Only consider obstacles within repulsive range
            if distance > self.repulsive_range:
                continue

            # Convert to Cartesian (robot frame)
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

            # Force direction: away from obstacle (radial component)
            fx_radial = magnitude * (-obs_x / obs_dist)
            fy_radial = magnitude * (-obs_y / obs_dist)

            # Add TANGENTIAL component for obstacles in front
            # This makes the robot turn sideways instead of just backing up
            if abs(angle) < 0.785:  # Frontal obstacle (within ±45°)
                # Tangential direction: perpendicular to obstacle direction
                # Choose direction based on which side has more space
                # For now, always turn left (positive Y in robot frame)
                tangent_factor = 2.0  # Strength multiplier for tangential force
                
                # If obstacle is slightly to the right (angle < 0), turn left
                # If obstacle is slightly to the left (angle > 0), turn right
                turn_direction = -1.0 if angle > 0 else 1.0
                
                fx_tangent = magnitude * tangent_factor * (-obs_y / obs_dist) * turn_direction
                fy_tangent = magnitude * tangent_factor * (obs_x / obs_dist) * turn_direction
                
                fx_total += fx_radial + fx_tangent
                fy_total += fy_radial + fy_tangent
            else:
                fx_total += fx_radial
                fy_total += fy_radial
            
            obstacles_in_range += 1

        return fx_total, fy_total, closest_obstacle, obstacle_in_front, obstacles_in_range

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

        # Check if laser data is available and fresh
        if self.laser_ranges is None:
            self.get_logger().warn('No laser data available yet!')
            self.cmd_pub.publish(cmd)
            return
        
        # Check for stale laser data (more than 1 second old)
        if self.laser_data_time is not None:
            now = self.get_clock().now()
            age_ns = (now - self.laser_data_time).nanoseconds
            age_sec = age_ns / 1e9
            if age_sec > 1.0:
                self.get_logger().warn(f'Laser data is stale! Age: {age_sec:.2f}s')

        # ============ LOCAL MINIMA ESCAPE HANDLING ============
        # If in escape mode, execute escape maneuver
        if self._escape_mode:
            escape_cmd = self.execute_escape()
            if escape_cmd is not None:
                self.cmd_pub.publish(escape_cmd)
                return
            # If escape_cmd is None, escape is complete, continue with normal navigation
        
        # Update position history for stuck detection (only when not escaping)
        self.update_position_history()
        
        # Check if stuck in local minimum
        if self.is_stuck():
            self.start_escape()
            return  # Will handle escape in next iteration

        # Check if goal reached
        distance_to_goal = math.sqrt(
            (self.goal_x - self.robot_x)**2 + 
            (self.goal_y - self.robot_y)**2)

        if distance_to_goal < self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info(
                    f'Goal reached at ({self.robot_x:.2f}, {self.robot_y:.2f})! '
                    f'PURGING data and waiting for new goal.')
                
                # PURGE everything by calling reset_state
                self.reset_state()
                self.goal_reached = True
                self._waiting_for_new_goal = True
                self.has_goal = False  # Disable navigation until new goal
            
            self.cmd_pub.publish(cmd)  # Publish zero velocity
            return

        # Calculate forces
        fx_att, fy_att = self.calculate_attractive_force()
        fx_rep, fy_rep, closest_obstacle, obstacle_in_front, obstacles_in_range = self.calculate_repulsive_force()

        # AGGRESSIVE ESCAPE MANEUVER for close obstacles
        emergency_distance = 0.20  # Increased from 0.12m to 0.20m
        critical_distance = 0.15   # Very close - need strong escape
        
        if closest_obstacle < emergency_distance:
            # Track how long we've been in emergency mode
            if not hasattr(self, '_emergency_counter'):
                self._emergency_counter = 0
            self._emergency_counter += 1
            
            # Determine escape direction (consistently turn RIGHT to escape)
            escape_direction = -0.5  # Always turn right (negative = clockwise)
            
            if closest_obstacle < critical_distance:
                # CRITICAL: Very close, back up fast and turn hard
                self.get_logger().warn(
                    f'CRITICAL ESCAPE: Obstacle at {closest_obstacle:.2f}m! '
                    f'Emergency #{self._emergency_counter}')
                cmd.linear.x = -0.15  # Back up faster
                cmd.angular.z = escape_direction * 1.5  # Turn hard right
            else:
                # Close but not critical: back up and turn
                self.get_logger().warn(
                    f'ESCAPE: Obstacle at {closest_obstacle:.2f}m! Backing up.')
                cmd.linear.x = -0.10  # Back up moderately
                cmd.angular.z = escape_direction  # Turn right
            
            self.cmd_pub.publish(cmd)
            return
        else:
            # Reset emergency counter when we're clear
            if hasattr(self, '_emergency_counter'):
                self._emergency_counter = 0

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

        # Angular velocity (proportional control with higher gain)
        angular_vel = 2.5 * angle_error
        angular_vel = max(-self.max_angular_speed, 
                         min(self.max_angular_speed, angular_vel))

        # TURN FIRST: If angle error is large, rotate in place before moving
        if abs(angle_error) > math.pi / 3:  # > 60 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = angular_vel
            self.cmd_pub.publish(cmd)
            return

        # Linear velocity with obstacle proximity scaling
        if closest_obstacle < self.obstacle_threshold:
            speed_factor = closest_obstacle / self.obstacle_threshold
            speed_factor = max(0.1, speed_factor)  # Minimum 10%
        else:
            speed_factor = 1.0

        linear_vel = self.max_linear_speed * speed_factor

        # Reduce speed when turning
        turn_factor = 1.0 - (abs(angle_error) / math.pi) * 0.7
        linear_vel *= turn_factor

        linear_vel = max(self.min_linear_speed, linear_vel)

        # Set velocity command
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        self.cmd_pub.publish(cmd)

        # INTENSIVE DEBUG: Log EVERY iteration to catch issues
        self._log_counter += 1
        f_att_mag = math.sqrt(fx_att**2 + fy_att**2)
        f_rep_mag = math.sqrt(fx_rep**2 + fy_rep**2)
        front_indicator = "FRONT!" if obstacle_in_front else ""
        
        # Log every iteration (temporarily) - shows obstacles detected
        self.get_logger().info(
            f'[NAV] Scan#{self._scan_counter} | '
            f'Obs_in_range:{obstacles_in_range} | '
            f'F_att:{f_att_mag:.2f} F_rep:{f_rep_mag:.2f} {front_indicator} | '
            f'Closest:{closest_obstacle:.2f}m | '
            f'V:{linear_vel:.3f} W:{angular_vel:.3f}')


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
