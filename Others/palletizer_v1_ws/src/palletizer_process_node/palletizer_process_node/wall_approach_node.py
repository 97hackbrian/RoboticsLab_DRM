#!/usr/bin/env python3
"""
Wall Approach Node for Palletizer

Uses 3D LiDAR to detect the closest wall in front of the robot.
Uses TF2 to transform points from lidar_link to base_footprint frame.
Publishes debug point cloud for visualization.

Author: hackbrian
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header
import numpy as np
import struct
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation


class WallApproachNode(Node):
    """
    Wall proximity detection using 3D LiDAR.
    
    Transforms points to base_footprint frame and finds closest
    obstacle in the front cone.
    """

    def __init__(self):
        super().__init__('wall_approach_node')
        
        # Parameters
        self.declare_parameter('wall_distance', 0.30)  # 30cm
        self.declare_parameter('front_cone_angle', 0.52)  # ~30 degrees half-angle
        self.declare_parameter('min_height', 0.1)   # Minimum point height (Z in base_footprint)
        self.declare_parameter('max_height', 1.5)   # Maximum point height
        self.declare_parameter('lidar_topic', '/lidar/points')
        self.declare_parameter('target_frame', 'base_footprint')
        
        self.wall_distance = self.get_parameter('wall_distance').value
        self.front_cone_angle = self.get_parameter('front_cone_angle').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        lidar_topic = self.get_parameter('lidar_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        
        # Current minimum distance
        self.min_front_distance = float('inf')
        self.wall_close = False
        
        # Debug counters
        self.debug_total_pts = 0
        self.debug_height_pts = 0
        self.debug_front_pts = 0
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Cached transform matrix (lidar_link -> base_footprint)
        self.transform_matrix = None
        self.transform_valid = False
        
        # --- Publishers ---
        self.wall_close_pub = self.create_publisher(
            Bool, '/palletizer/wall_close', 10
        )
        # Debug: publish filtered front points
        self.debug_cloud_pub = self.create_publisher(
            PointCloud2, '/palletizer/wall_debug_cloud', 10
        )
        
        # --- Subscribers ---
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self.lidar_callback, 10
        )
        
        # Status update timer (10 Hz)
        self.timer = self.create_timer(0.1, self.status_update)
        
        # TF lookup timer (1 Hz)
        self.tf_timer = self.create_timer(1.0, self.update_transform)
        
        self.get_logger().info(
            f'Wall Approach node initialized. Wall distance: {self.wall_distance*100:.1f}cm'
        )
        self.get_logger().info(
            f'Will transform points to frame: {self.target_frame}'
        )

    def update_transform(self):
        """Update cached transform from lidar_link to base_footprint."""
        try:
            # Lookup transform: lidar_link -> base_footprint
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,  # target frame
                'lidar_link',       # source frame
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Build 4x4 transformation matrix
            t = transform.transform.translation
            r = transform.transform.rotation
            
            # Rotation quaternion to matrix
            rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
            rot_matrix = rot.as_matrix()
            
            # Build 4x4 transform matrix
            self.transform_matrix = np.eye(4)
            self.transform_matrix[:3, :3] = rot_matrix
            self.transform_matrix[:3, 3] = [t.x, t.y, t.z]
            self.transform_valid = True
            
            if not hasattr(self, '_tf_logged'):
                self._tf_logged = True
                self.get_logger().info(
                    f'TF acquired: lidar_link -> {self.target_frame}'
                )
                
        except TransformException as ex:
            self.transform_valid = False
            self.get_logger().warn(f'Could not get transform: {ex}')

    def parse_pointcloud2(self, msg: PointCloud2):
        """Parse PointCloud2 message and extract XYZ points."""
        field_names = [field.name for field in msg.fields]
        
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
        
        points = []
        for i in range(0, len(data), point_step):
            try:
                x = struct.unpack_from('f', data, i + x_offset)[0]
                y = struct.unpack_from('f', data, i + y_offset)[0]
                z = struct.unpack_from('f', data, i + z_offset)[0]
                
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or
                        np.isinf(x) or np.isinf(y) or np.isinf(z)):
                    points.append([x, y, z])
            except struct.error:
                continue
        
        return np.array(points)

    def transform_points(self, points):
        """Transform points from lidar_link to base_footprint using cached matrix."""
        if not self.transform_valid or self.transform_matrix is None:
            return None
        
        # Add homogeneous coordinate
        ones = np.ones((points.shape[0], 1))
        points_h = np.hstack([points, ones])
        
        # Apply transform
        transformed = (self.transform_matrix @ points_h.T).T
        
        return transformed[:, :3]

    def create_debug_cloud(self, points, frame_id):
        """Create a PointCloud2 message from numpy points for debugging."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack points into bytes
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))
        msg.data = b''.join(buffer)
        
        return msg

    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR point cloud to find closest front obstacle."""
        points_lidar = self.parse_pointcloud2(msg)
        
        self.debug_total_pts = len(points_lidar)
        
        if len(points_lidar) == 0:
            self.debug_height_pts = 0
            self.debug_front_pts = 0
            return
        
        # Transform to base_footprint frame
        if not self.transform_valid:
            return
        
        points = self.transform_points(points_lidar)
        if points is None:
            return
        
        # Log stats once
        if not hasattr(self, '_logged_stats'):
            self._logged_stats = True
            self.get_logger().info(
                f'[DEBUG] Transformed points (base_footprint): '
                f'X=[{points[:,0].min():.2f}, {points[:,0].max():.2f}] '
                f'Y=[{points[:,1].min():.2f}, {points[:,1].max():.2f}] '
                f'Z=[{points[:,2].min():.2f}, {points[:,2].max():.2f}]'
            )
        
        # In base_footprint frame:
        # X = forward
        # Y = left
        # Z = up
        
        # Filter by height (Z in base_footprint)
        height_mask = (points[:, 2] > self.min_height) & (points[:, 2] < self.max_height)
        points_filtered = points[height_mask]
        
        self.debug_height_pts = len(points_filtered)
        
        if len(points_filtered) == 0:
            self.debug_front_pts = 0
            self.min_front_distance = float('inf')
            return
        
        # Filter by X > 0.3 (forward points only)
        forward_mask = points_filtered[:, 0] > 0.3
        forward_points = points_filtered[forward_mask]
        
        if len(forward_points) == 0:
            self.debug_front_pts = 0
            self.min_front_distance = float('inf')
            return
        
        # Filter by angle - only points in front cone
        # angle = atan2(|Y|, X)
        angles = np.arctan2(np.abs(forward_points[:, 1]), forward_points[:, 0])
        front_mask = angles < self.front_cone_angle
        front_points = forward_points[front_mask]
        
        self.debug_front_pts = len(front_points)
        
        # Publish debug cloud
        if len(front_points) > 0:
            debug_msg = self.create_debug_cloud(front_points, self.target_frame)
            self.debug_cloud_pub.publish(debug_msg)
        
        if len(front_points) == 0:
            self.min_front_distance = float('inf')
            return
        
        # Distance is X coordinate (forward in base_footprint)
        distances = front_points[:, 0]
        self.min_front_distance = float(np.min(distances))
        
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
