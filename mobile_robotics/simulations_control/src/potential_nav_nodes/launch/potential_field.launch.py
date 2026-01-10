#!/usr/bin/env python3
"""
Launch file for Potential Field Navigation.

Launches:
1. Gazebo simulation with TurtleBot3 (turtlebot3_house.launch.py)
2. ROS-Gazebo bridge for odom, scan, cmd_vel
3. Potential Field Navigation node with parameters
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
    pkg_gazebo_sim = get_package_share_directory('turtlebot3_gazebo')

    # Config file path
    config_file = os.path.join(pkg_potential_nav, 'config', 'potential_field_params.yaml')

    # Include Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # ROS-Gazebo Bridge for topics
    # Format: "gz_topic@ros_msg_type[gz_to_ros or ros_to_gz]"
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Odometry: Gazebo -> ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LaserScan: Gazebo -> ROS
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Velocity commands: ROS -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # TF: Gazebo -> ROS
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/cmd_vel', '/cmd_vel'),
        ]
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
        bridge_node,
        potential_field_node,
    ])
