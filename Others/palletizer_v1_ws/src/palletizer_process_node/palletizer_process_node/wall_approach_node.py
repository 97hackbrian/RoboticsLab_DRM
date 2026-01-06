#!/usr/bin/env python3
"""
Wall Approach Node for Palletizer

Uses 3D LiDAR to detect the closest wall in front of the robot.
Signals when the robot is close enough to the wall.

Author: hackbrian
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import numpy as np
import struct


class WallApproachNode(Node):
    """
    Wall proximity detection using 3D LiDAR.
    
    Analyzes /lidar/points to find the closest obstacle
    in the front cone and signals when within threshold.
    """

    def __init__(self):
        super().__init__('wall_approach_node')
        
        # Parameters
        self.declare_parameter('wall_distance', 0.30)  # 30cm
        self.declare_parameter('front_cone_angle', 0.52)  # ~30 degrees half-angle
        self.declare_parameter('min_height', 0.25)  # Minimum point height relative to robot
        self.declare_parameter('max_height', 1.5)   # Maximum point height relative to robot
        self.declare_parameter('lidar_topic', '/lidar/points')
        
        self.wall_distance = self.get_parameter('wall_distance').value
        self.front_cone_angle = self.get_parameter('front_cone_angle').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        lidar_topic = self.get_parameter('lidar_topic').value
        
        # Current minimum distance
        self.min_front_distance = float('inf')
        self.wall_close = False
        
        # Debug counters
        self.debug_total_pts = 0
        self.debug_height_pts = 0
        self.debug_front_pts = 0
        
        # --- Publishers ---
        self.wall_close_pub = self.create_publisher(
            Bool, '/palletizer/wall_close', 10
        )
        
        # --- Subscribers ---
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self.lidar_callback, 10
        )
        
        # Status update timer (10 Hz)
        self.timer = self.create_timer(0.1, self.status_update)
        
        self.get_logger().info(
            f'Wall Approach node initialized. Wall distance: {self.wall_distance*100:.1f}cm'
        )

    def parse_pointcloud2(self, msg: PointCloud2):
        """
        Parse PointCloud2 message and extract XYZ points.
        
        Returns numpy array of shape (N, 3) with X, Y, Z coordinates.
        """
        # Get field offsets
        field_names = [field.name for field in msg.fields]
        
        # Find x, y, z field indices
        try:
            x_idx = field_names.index('x')
            y_idx = field_names.index('y')
            z_idx = field_names.index('z')
        except ValueError:
            self.get_logger().warn('PointCloud2 missing x, y, or z fields')
            return np.array([])
        
        x_offset = msg.fields[x_idx].offset
        y_offset = msg.fields[y_idx].offset
        z_offset = msg.fields[z_idx].offset
        
        point_step = msg.point_step
        data = msg.data
        
        # Parse points
        points = []
        for i in range(0, len(data), point_step):
            try:
                x = struct.unpack_from('f', data, i + x_offset)[0]
                y = struct.unpack_from('f', data, i + y_offset)[0]
                z = struct.unpack_from('f', data, i + z_offset)[0]
                
                # Skip NaN and infinite values
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or
                        np.isinf(x) or np.isinf(y) or np.isinf(z)):
                    points.append([x, y, z])
            except struct.error:
                continue
        
        return np.array(points)

    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR point cloud to find closest front obstacle."""
        points = self.parse_pointcloud2(msg)
        
        self.debug_total_pts = len(points)
        
        if len(points) == 0:
            self.debug_height_pts = 0
            self.debug_front_pts = 0
            return
        
        # Log raw point cloud statistics once (first frame only)
        if not hasattr(self, '_logged_stats'):
            self._logged_stats = True
            self.get_logger().info(
                f'[DEBUG] Raw point cloud stats: '
                f'X=[{points[:,0].min():.2f}, {points[:,0].max():.2f}] '
                f'Y=[{points[:,1].min():.2f}, {points[:,1].max():.2f}] '
                f'Z=[{points[:,2].min():.2f}, {points[:,2].max():.2f}]'
            )
        
        # CORRECTED: LiDAR X axis points to robot's RIGHT
        # LiDAR -Y was still detecting RIGHT, so try +Y
        # Robot forward = LiDAR +Y axis
        forward_coord = points[:, 1]  # Robot forward = +lidar_Y
        forward_mask = forward_coord > 0.3
        forward_points = points[forward_mask]
        forward_coords = forward_coord[forward_mask]
        
        self.debug_height_pts = len(forward_points)  # Reuse variable
        
        if len(forward_points) == 0:
            self.debug_front_pts = 0
            self.min_front_distance = float('inf')
            return
        
        # Filter by angle - only points in front cone
        # Lateral = X in lidar frame (robot's right/left)
        angles = np.arctan2(np.abs(forward_points[:, 0]), forward_coords)
        front_mask = angles < self.front_cone_angle
        front_points_filtered = forward_points[front_mask]
        front_distances = forward_coords[front_mask]
        
        self.debug_front_pts = len(front_points_filtered)
        
        if len(front_points_filtered) == 0:
            self.min_front_distance = float('inf')
            return
        
        # Distance is the forward coordinate (-Y)
        self.min_front_distance = float(np.min(front_distances))
        
        # Check if wall is close
        was_close = self.wall_close
        self.wall_close = bool(self.min_front_distance < self.wall_distance)
        
        if self.wall_close and not was_close:
            self.get_logger().info(
                f'Wall detected! Distance: {self.min_front_distance*100:.1f}cm'
            )

    def status_update(self):
        """Publish wall proximity status with verbose logging."""
        msg = Bool()
        msg.data = bool(self.wall_close)
        self.wall_close_pub.publish(msg)
        
        # Verbose logging every update
        if self.min_front_distance != float('inf'):
            self.get_logger().info(
                f'[WALL] Dist: {self.min_front_distance*100:.1f}cm | '
                f'Thresh: {self.wall_distance*100:.1f}cm | '
                f'Close: {self.wall_close} | '
                f'FrontPts: {self.debug_front_pts}'
            )
        else:
            self.get_logger().warn(
                f'[WALL] No front points! Total:{self.debug_total_pts} '
                f'HeightFiltered:{self.debug_height_pts} '
                f'FrontCone:{self.debug_front_pts}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WallApproachNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
