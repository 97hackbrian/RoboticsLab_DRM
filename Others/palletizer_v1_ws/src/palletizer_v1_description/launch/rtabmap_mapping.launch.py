#!/usr/bin/env python3
"""
RTAB-Map 3D Mapping Launch File

This launch file starts RTAB-Map for 3D mapping using:
- LiDAR 3D PointCloud from /lidar/points
- Pseudo Visual Odometry from /odom_vo

The mapping uses the lidar_link frame for point cloud and
odom_vo_frame -> base_footprint for odometry.

REQUIRED PACKAGES (install if not present):
  sudo apt install ros-humble-rtabmap-slam ros-humble-rtabmap-viz ros-humble-pointcloud-to-laserscan
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def check_packages(context):
    """Check if required packages are available"""
    required_packages = ['rtabmap_slam', 'rtabmap_viz', 'pointcloud_to_laserscan']
    missing = []
    
    for pkg in required_packages:
        try:
            get_package_share_directory(pkg)
        except Exception:
            missing.append(pkg)
    
    if missing:
        print("\n" + "="*70)
        print("WARNING: Missing required packages!")
        print("="*70)
        print(f"Missing: {', '.join(missing)}")
        print("\nPlease install with:")
        print("  sudo apt install ros-humble-rtabmap-slam ros-humble-rtabmap-viz ros-humble-pointcloud-to-laserscan")
        print("="*70 + "\n")
    
    return []


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    localization = LaunchConfiguration('localization', default='false')
    
    # RTAB-Map parameters for 3D LiDAR SLAM
    rtabmap_parameters = {
        # General
        'use_sim_time': use_sim_time,
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom_vo_frame',
        'map_frame_id': 'map',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'queue_size': 10,
        
        # RTAB-Map core parameters
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        
        # ICP parameters (using lidar)
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '1.0',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '30',
        'Icp/Epsilon': '0.001',
        
        # Registration parameters
        'Reg/Strategy': '1',  # 0=Vis, 1=ICP, 2=VisICP
        'Reg/Force3DoF': 'false',  # 6DoF for 3D mapping
        
        # Grid map parameters
        'Grid/Sensor': '0',  # 0=laser scan
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '20.0',
        'Grid/RangeMin': '0.5',
        'Grid/ClusterRadius': '0.1',
        'Grid/GroundIsObstacle': 'false',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/NormalsSegmentation': 'true',
        'Grid/3D': 'true',
        'Grid/RayTracing': 'true',
        
        # Loop closure detection
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        
        # Memory management
        'Rtabmap/TimeThr': '0',
        'Rtabmap/DetectionRate': '1',
        
        # Output
        'GridGlobal/MinSize': '20.0',
    }
    
    return LaunchDescription([
        # Check for required packages
        OpaqueFunction(function=check_packages),
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode (use existing map)'
        ),
        
        LogInfo(msg='\n========================================'),
        LogInfo(msg='RTAB-Map 3D Mapping Starting...'),
        LogInfo(msg='LiDAR Topic: /lidar/points'),
        LogInfo(msg='Odometry Topic: /odom_vo'),
        LogInfo(msg='Frame: base_footprint'),
        LogInfo(msg='Odom Frame: odom_vo_frame'),
        LogInfo(msg='========================================\n'),
        
        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters],
            arguments=['-d'],  # Delete database on start (remove for persistent mapping)
            remappings=[
                ('scan_cloud', '/lidar/points'),
                ('odom', '/odom_vo'),
            ]
        ),
        
        # RTAB-Map Visualization Node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom_vo_frame',
                'subscribe_scan_cloud': True,
                'subscribe_odom': True,
            }],
            remappings=[
                ('scan_cloud', '/lidar/points'),
                ('odom', '/odom_vo'),
            ]
        ),
        
        # Point Cloud to LaserScan converter (for 2D occupancy grid)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'lidar_link',
                'transform_tolerance': 0.1,
                'min_height': -0.1,
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,  # ~0.5 degree
                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/lidar/points'),
                ('scan', '/scan'),
            ]
        ),
    ])
