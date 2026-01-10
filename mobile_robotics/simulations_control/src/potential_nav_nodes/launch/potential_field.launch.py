#!/usr/bin/env python3
"""
Launch file for Potential Field Navigation.

Launches:
1. Gazebo simulation with TurtleBot3 (class.launch.py)
2. Potential Field Navigation node with parameters
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_potential_nav = get_package_share_directory('potential_nav_nodes')
    pkg_gazebo_sim = get_package_share_directory('gazebo_robot_simulation')

    # Config file path
    config_file = os.path.join(pkg_potential_nav, 'config', 'potential_field_params.yaml')

    # Include Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, 'launch', 'class.launch.py')
        )
    )

    # Potential Field Navigation node
    potential_field_node = Node(
        package='potential_nav_nodes',
        executable='potential_field_node',
        name='potential_field_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        gazebo_launch,
        potential_field_node,
    ])
