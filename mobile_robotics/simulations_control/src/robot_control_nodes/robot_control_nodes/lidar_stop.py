import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np


class LidarStop(Node):
    """
    Node for monitoring LiDAR data and controlling linear velocity to avoid collisions.
    Uses ApproximateTimeSynchronizer for LiDAR data synchronization.
    """

    def __init__(self):
        super().__init__('lidar_stop')

        # Subscriber to LiDAR scan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher for linear velocity commands (TwistStamped)
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_linear', 10)

        # Safety parameters
        self.safe_distance = 0.5  # meters - stop if obstacle within this distance
        self.max_linear_velocity = 0.5  # m/s
        self.front_angle_range = 30.0  # degrees - analyze ±30° in front of robot
        
        # Smoothing parameters
        self.slowdown_distance = 1.0  # meters - start slowing down at this distance
        
        self.get_logger().info('LiDAR Stop node started. Safe distance: {:.2f}m'.format(self.safe_distance))

    def lidar_callback(self, msg):
        """
        Callback for processing LiDAR scan data and controlling linear velocity.
        """
        # Convert angle range to radians
        front_angle_rad = np.radians(self.front_angle_range)
        
        # Calculate indices for front sector
        # LiDAR typically scans from -π to π, with 0 being straight ahead
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # Calculate the indices for the front sector (±front_angle_range degrees)
        total_points = len(msg.ranges)
        
        # Find indices corresponding to front sector
        front_indices = []
        for i in range(total_points):
            angle = angle_min + i * angle_increment
            if abs(angle) <= front_angle_rad:
                front_indices.append(i)
        
        # Get distances in front sector, filtering out invalid readings
        front_distances = []
        for i in front_indices:
            distance = msg.ranges[i]
            # Filter out inf, nan, and values outside valid range
            if (not np.isinf(distance) and 
                not np.isnan(distance) and 
                msg.range_min <= distance <= msg.range_max):
                front_distances.append(distance)
        
        # Determine linear velocity based on minimum distance
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        if len(front_distances) > 0:
            min_distance = min(front_distances)
            
            if min_distance <= self.safe_distance:
                # Stop immediately - obstacle too close
                cmd.twist.linear.x = 0.0
                
                # Log warning (throttled)
                if hasattr(self, '_last_warning_time'):
                    if (self.get_clock().now().nanoseconds - self._last_warning_time) > 2e9:  # 2 seconds
                        self.get_logger().warn(
                            f'Obstacle detected at {min_distance:.2f}m - STOPPING'
                        )
                        self._last_warning_time = self.get_clock().now().nanoseconds
                else:
                    self._last_warning_time = self.get_clock().now().nanoseconds
                    
            elif min_distance <= self.slowdown_distance:
                # Gradually reduce speed as we approach the safe distance
                # Linear interpolation between safe_distance (0 vel) and slowdown_distance (max vel)
                velocity_ratio = (min_distance - self.safe_distance) / (self.slowdown_distance - self.safe_distance)
                cmd.twist.linear.x = self.max_linear_velocity * velocity_ratio
                
                # Log info (throttled)
                if hasattr(self, '_last_info_time'):
                    if (self.get_clock().now().nanoseconds - self._last_info_time) > 1e9:  # 1 second
                        self.get_logger().info(
                            f'Obstacle at {min_distance:.2f}m - Slowing down: {cmd.twist.linear.x:.2f} m/s'
                        )
                        self._last_info_time = self.get_clock().now().nanoseconds
                else:
                    self._last_info_time = self.get_clock().now().nanoseconds
                    
            else:
                # Safe to move at max velocity
                cmd.twist.linear.x = self.max_linear_velocity
        else:
            # No valid readings - proceed with caution at reduced speed
            cmd.twist.linear.x = self.max_linear_velocity * 0.5
            self.get_logger().warn('No valid LiDAR readings in front sector', throttle_duration_sec=2.0)
        
        # Publish velocity command
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarStop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
