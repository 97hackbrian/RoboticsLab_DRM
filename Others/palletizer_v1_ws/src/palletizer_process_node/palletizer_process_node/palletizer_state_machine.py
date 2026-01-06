#!/usr/bin/env python3
"""
Palletizer State Machine Node

Main coordinator for the palletizing process. Manages state transitions between:
- IDLE: Waiting for start command
- NAVIGATION: Moving toward goal position
- ARUCO_TRACKING: Tracking ArUco marker angularly
- BOX_APPROACH: Approaching box until Sharp sensor < 5cm
- WALL_APPROACH: Approaching wall with box loaded

Author: hackbrian
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, TwistStamped
from enum import Enum


class PalletizerState(Enum):
    IDLE = "IDLE"
    NAVIGATION = "NAVIGATION"
    ARUCO_TRACKING = "ARUCO_TRACKING"
    BOX_APPROACH = "BOX_APPROACH"
    WALL_APPROACH = "WALL_APPROACH"
    REVERSE = "REVERSE"


class PalletizerStateMachine(Node):
    """
    State machine coordinator for the palletizing process.
    
    Subscribes to status topics from other nodes and publishes
    combined velocity commands based on current state.
    """

    def __init__(self):
        super().__init__('palletizer_state_machine')
        
        # Parameters
        self.declare_parameter('auto_start', True)
        self.declare_parameter('navigation_speed', 0.3)
        self.declare_parameter('approach_speed', 0.15)
        
        self.auto_start = self.get_parameter('auto_start').value
        self.navigation_speed = self.get_parameter('navigation_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        
        # Current state
        self.current_state = PalletizerState.IDLE
        self.box_captured = False
        
        # Reverse state timer
        self.reverse_start_time = None
        self.declare_parameter('reverse_duration', 2.0)  # seconds
        self.declare_parameter('reverse_speed', 0.2)
        self.declare_parameter('aruco_cooldown', 5.0)  # seconds to ignore ArUco after deposit
        self.reverse_duration = self.get_parameter('reverse_duration').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.aruco_cooldown = self.get_parameter('aruco_cooldown').value
        
        # Cooldown timer to ignore ArUco after depositing box
        self.aruco_cooldown_start = None
        
        # Velocity commands from child nodes
        self.nav_cmd = Twist()
        self.aruco_cmd = TwistStamped()
        
        # Status flags
        self.aruco_detected = False
        self.goal_reached = False
        self.box_close = False
        self.wall_close = False
        
        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/palletizer/state', 10)
        
        # --- Subscribers ---
        # Navigation velocity commands
        self.nav_cmd_sub = self.create_subscription(
            Twist, '/palletizer/nav_cmd_vel', self.nav_cmd_callback, 10
        )
        
        # ArUco tracking velocity commands
        self.aruco_cmd_sub = self.create_subscription(
            TwistStamped, '/palletizer/aruco_cmd_vel', self.aruco_cmd_callback, 10
        )
        
        # Status topics from child nodes
        self.aruco_detected_sub = self.create_subscription(
            Bool, '/palletizer/aruco_detected', self.aruco_detected_callback, 10
        )
        
        self.goal_reached_sub = self.create_subscription(
            Bool, '/palletizer/goal_reached', self.goal_reached_callback, 10
        )
        
        self.box_close_sub = self.create_subscription(
            Bool, '/palletizer/box_close', self.box_close_callback, 10
        )
        
        self.wall_close_sub = self.create_subscription(
            Bool, '/palletizer/wall_close', self.wall_close_callback, 10
        )
        
        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Palletizer State Machine initialized')
        
        # Auto-start navigation if configured
        if self.auto_start:
            self.get_logger().info('Auto-starting navigation mode')
            self.transition_to(PalletizerState.NAVIGATION)

    def transition_to(self, new_state: PalletizerState):
        """Transition to a new state with logging."""
        if new_state != self.current_state:
            self.get_logger().info(f'State transition: {self.current_state.value} -> {new_state.value}')
            self.current_state = new_state
            
            # Publish state change
            msg = String()
            msg.data = new_state.value
            self.state_pub.publish(msg)

    def nav_cmd_callback(self, msg: Twist):
        """Store navigation velocity command."""
        self.nav_cmd = msg

    def aruco_cmd_callback(self, msg: TwistStamped):
        """Store ArUco tracking velocity command."""
        self.aruco_cmd = msg

    def aruco_detected_callback(self, msg: Bool):
        """Update ArUco detection status."""
        self.aruco_detected = msg.data

    def goal_reached_callback(self, msg: Bool):
        """Update goal reached status."""
        self.goal_reached = msg.data

    def box_close_callback(self, msg: Bool):
        """Update box proximity status."""
        self.box_close = msg.data

    def wall_close_callback(self, msg: Bool):
        """Update wall proximity status."""
        self.wall_close = msg.data

    def control_loop(self):
        """
        Main control loop - handles state transitions and velocity output.
        """
        cmd = Twist()
        
        if self.current_state == PalletizerState.IDLE:
            # Robot stopped - publish explicit zero velocity
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # Log only once when entering IDLE
            pass
            
        elif self.current_state == PalletizerState.NAVIGATION:
            # Check if ArUco cooldown is active
            aruco_allowed = True
            if self.aruco_cooldown_start is not None:
                cooldown_elapsed = (self.get_clock().now() - self.aruco_cooldown_start).nanoseconds / 1e9
                if cooldown_elapsed < self.aruco_cooldown:
                    aruco_allowed = False
                else:
                    self.aruco_cooldown_start = None  # Cooldown expired
                    self.get_logger().info('ArUco cooldown expired, detection enabled')
            
            # Check for ArUco detection - pause navigation and track
            if self.aruco_detected and not self.box_captured and aruco_allowed:
                self.transition_to(PalletizerState.ARUCO_TRACKING)
            elif self.goal_reached:
                self.get_logger().info('Goal reached! Returning to IDLE')
                self.transition_to(PalletizerState.IDLE)
            else:
                # Forward navigation command
                cmd = self.nav_cmd
                
        elif self.current_state == PalletizerState.ARUCO_TRACKING:
            # Track ArUco angularly
            if not self.aruco_detected:
                # Lost ArUco, go back to navigation
                self.get_logger().warn('Lost ArUco marker, resuming navigation')
                self.transition_to(PalletizerState.NAVIGATION)
            else:
                # Apply angular tracking command
                cmd.angular.z = self.aruco_cmd.twist.angular.z
                
                # Check if ArUco is centered (angular velocity is small)
                if abs(self.aruco_cmd.twist.angular.z) < 0.05:
                    self.get_logger().info('ArUco centered, starting box approach')
                    self.transition_to(PalletizerState.BOX_APPROACH)
                    
        elif self.current_state == PalletizerState.BOX_APPROACH:
            # Move forward slowly, track ArUco angularly
            if self.box_close:
                self.get_logger().info('Box captured! Starting wall approach')
                self.box_captured = True
                self.transition_to(PalletizerState.WALL_APPROACH)
            else:
                # Move forward slowly while tracking ArUco
                cmd.linear.x = self.approach_speed
                cmd.angular.z = self.aruco_cmd.twist.angular.z * 0.5  # Reduced angular
                
        elif self.current_state == PalletizerState.WALL_APPROACH:
            # Move forward until wall is close
            if self.wall_close:
                self.get_logger().info('Wall reached! Starting reverse')
                self.box_captured = False
                self.reverse_start_time = self.get_clock().now()
                self.transition_to(PalletizerState.REVERSE)
            else:
                cmd.linear.x = self.approach_speed
        
        elif self.current_state == PalletizerState.REVERSE:
            # Reverse for configured duration
            elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if elapsed >= self.reverse_duration:
                self.get_logger().info('Reverse complete! Resuming navigation with ArUco cooldown')
                self.aruco_cooldown_start = self.get_clock().now()  # Start cooldown
                self.transition_to(PalletizerState.NAVIGATION)
            else:
                cmd.linear.x = -self.reverse_speed  # Negative = backward
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
        
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PalletizerStateMachine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
