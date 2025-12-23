#!/usr/bin/env python3
"""
Launch file for robot_control_nodes package.
Launches all three control nodes:
- tracking_aruco: ArUco marker tracking
- lidar_stop: LiDAR obstacle avoidance
- mixer_node: Velocity mixer with safety limits
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for robot control nodes.
    """
    
    # Declare launch arguments for mixer node parameters
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.4',
        description='Maximum linear velocity in m/s'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Maximum angular velocity in rad/s'
    )
    
    sync_queue_size_arg = DeclareLaunchArgument(
        'sync_queue_size',
        default_value='10',
        description='ApproximateTimeSynchronizer queue size'
    )
    
    sync_slop_arg = DeclareLaunchArgument(
        'sync_slop',
        default_value='0.1',
        description='ApproximateTimeSynchronizer time slop (seconds)'
    )
    
    # Get launch configurations
    max_linear_vel = LaunchConfiguration('max_linear_vel')
    max_angular_vel = LaunchConfiguration('max_angular_vel')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    sync_slop = LaunchConfiguration('sync_slop')
    
    # Node 1: ArUco Tracking
    tracking_aruco_node = Node(
        package='robot_control_nodes',
        executable='tracking_aruco',
        name='tracking_aruco',
        output='screen',
        parameters=[],
        remappings=[
            # Add remappings here if needed
            # ('/camera/image_raw', '/custom_camera_topic'),
        ]
    )
    
    # Node 2: LiDAR Stop
    lidar_stop_node = Node(
        package='robot_control_nodes',
        executable='lidar_stop',
        name='lidar_stop',
        output='screen',
        parameters=[],
        remappings=[
            # Add remappings here if needed
            # ('/scan', '/custom_lidar_topic'),
        ]
    )
    
    # Node 3: Velocity Mixer
    mixer_node = Node(
        package='robot_control_nodes',
        executable='mixer_node',
        name='mixer_node',
        output='screen',
        parameters=[{
            'max_linear_vel': max_linear_vel,
            'max_angular_vel': max_angular_vel,
            'sync_queue_size': sync_queue_size,
            'sync_slop': sync_slop,
        }],
        remappings=[
            # Add remappings here if needed
            # ('/cmd_vel', '/custom_cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        max_linear_vel_arg,
        max_angular_vel_arg,
        sync_queue_size_arg,
        sync_slop_arg,
        
        # Nodes
        tracking_aruco_node,
        lidar_stop_node,
        mixer_node,
    ])
