#!/usr/bin/env python3
"""
Palletizer Process Launch File

Launches all nodes for the palletizing process:
- State Machine (coordinator)
- Navigation Node
- ArUco Tracker Node
- Box Approach Node
- Wall Approach Node

Author: hackbrian
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('palletizer_process_node')
    config_file = os.path.join(pkg_dir, 'config', 'palletizer_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    show_aruco_image = LaunchConfiguration('show_aruco_image', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'show_aruco_image',
            default_value='true',
            description='Show ArUco detection visualization window'
        ),
        
        # State Machine Node (Main Coordinator)
        Node(
            package='palletizer_process_node',
            executable='palletizer_state_machine',
            name='palletizer_state_machine',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Navigation Node
        Node(
            package='palletizer_process_node',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # ArUco Tracker Node
        Node(
            package='palletizer_process_node',
            executable='aruco_tracker_node',
            name='aruco_tracker_node',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time},
                {'show_image': show_aruco_image}
            ]
        ),
        
        # Box Approach Node
        Node(
            package='palletizer_process_node',
            executable='box_approach_node',
            name='box_approach_node',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Wall Approach Node
        Node(
            package='palletizer_process_node',
            executable='wall_approach_node',
            name='wall_approach_node',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])
