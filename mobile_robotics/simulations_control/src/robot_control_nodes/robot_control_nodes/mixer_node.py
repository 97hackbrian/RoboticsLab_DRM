import rclpy
from rclpy.node import Node

# Import message_filters for synchronization
import message_filters

# TwistStamped for input (has timestamp), Twist for output
from geometry_msgs.msg import Twist, TwistStamped

import numpy as np


class SyncedCmdVelMixer(Node):
    """
    Node for mixing synchronized linear and angular velocity commands.
    Subscribes to /cmd_vel_linear and /cmd_vel_angular (TwistStamped)
    and publishes mixed commands to /cmd_vel (Twist) with safety limits.
    """

    def __init__(self):
        super().__init__('synced_mixer')

        # Safety velocity limits (configurable parameters)
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.1)  # seconds
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        sync_queue_size = self.get_parameter('sync_queue_size').value
        sync_slop = self.get_parameter('sync_slop').value

        # 1. Define subscribers using message_filters
        # DO NOT use self.create_subscription, use message_filters.Subscriber
        self.sub_linear = message_filters.Subscriber(
            self, TwistStamped, '/cmd_vel_linear'
        )
        self.sub_angular = message_filters.Subscriber(
            self, TwistStamped, '/cmd_vel_angular'
        )

        # 2. Configure approximate time synchronization
        # queue_size: How many messages to keep in buffer
        # slop: Time window in seconds (e.g., 0.1s = 100ms) to consider messages as "simultaneous"
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_linear, self.sub_angular],
            queue_size=sync_queue_size,
            slop=sync_slop
        )

        # 3. Register callback that executes ONLY when both messages arrive
        self.ts.registerCallback(self.sync_callback)

        # 4. Final publisher for the robot (standard Twist message)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Statistics for monitoring
        self.msg_count = 0
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0

        self.get_logger().info('=' * 60)
        self.get_logger().info('Synced Mixer Node Started')
        self.get_logger().info(f'  Max Linear Velocity:  {self.max_linear_vel:.2f} m/s')
        self.get_logger().info(f'  Max Angular Velocity: {self.max_angular_vel:.2f} rad/s')
        self.get_logger().info(f'  Sync Queue Size:      {sync_queue_size}')
        self.get_logger().info(f'  Sync Slop (window):   {sync_slop:.3f} s')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for synchronized message pairs...')

    def sync_callback(self, linear_msg, angular_msg):
        """
        Callback that executes when a pair of (linear, angular) messages
        with close timestamps (within slop window) arrive.
        
        Args:
            linear_msg: TwistStamped message with linear velocity
            angular_msg: TwistStamped message with angular velocity
        """
        # Create output Twist message
        cmd = Twist()

        # Extract velocities from the stamped messages
        linear_vel = linear_msg.twist.linear.x
        angular_vel = angular_msg.twist.angular.z

        # Apply safety limits using numpy clip
        cmd.linear.x = float(np.clip(
            linear_vel,
            -self.max_linear_vel,
            self.max_linear_vel
        ))
        cmd.angular.z = float(np.clip(
            angular_vel,
            -self.max_angular_vel,
            self.max_angular_vel
        ))

        # Additional safety checks
        # Ensure no NaN or Inf values
        if np.isnan(cmd.linear.x) or np.isinf(cmd.linear.x):
            self.get_logger().warn('Invalid linear velocity detected (NaN/Inf), setting to 0')
            cmd.linear.x = 0.0
            
        if np.isnan(cmd.angular.z) or np.isinf(cmd.angular.z):
            self.get_logger().warn('Invalid angular velocity detected (NaN/Inf), setting to 0')
            cmd.angular.z = 0.0

        # Publish the mixed command
        self.publisher_.publish(cmd)

        # Update statistics
        self.msg_count += 1
        self.last_linear_vel = cmd.linear.x
        self.last_angular_vel = cmd.angular.z

        # Log velocity changes (throttled logging every 20 messages)
        if self.msg_count % 20 == 0:
            self.get_logger().info(
                f'[{self.msg_count:04d}] Mixed Cmd -> '
                f'Linear: {cmd.linear.x:+.3f} m/s, '
                f'Angular: {cmd.angular.z:+.3f} rad/s'
            )

        # Warning if safety limits were applied
        if abs(linear_vel) > self.max_linear_vel or abs(angular_vel) > self.max_angular_vel:
            if hasattr(self, '_last_limit_warning_time'):
                if (self.get_clock().now().nanoseconds - self._last_limit_warning_time) > 2e9:  # 2 seconds
                    self.get_logger().warn(
                        f'SAFETY LIMIT APPLIED! '
                        f'Input: Lin={linear_vel:.2f}, Ang={angular_vel:.2f} | '
                        f'Output: Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}'
                    )
                    self._last_limit_warning_time = self.get_clock().now().nanoseconds
            else:
                self._last_limit_warning_time = self.get_clock().now().nanoseconds

    def get_status_string(self):
        """
        Return current status as a formatted string.
        """
        return (
            f'Messages processed: {self.msg_count}, '
            f'Last cmd: Lin={self.last_linear_vel:.3f} m/s, '
            f'Ang={self.last_angular_vel:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SyncedCmdVelMixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
        node.get_logger().info(node.get_status_string())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
