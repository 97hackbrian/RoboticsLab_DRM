"""
Launch file for DQN Navigation package.

Usage:
    Training mode:
        ros2 launch finalworkv2_pkg dqn_navigation.launch.py
        ros2 launch finalworkv2_pkg dqn_navigation.launch.py mode:=train
    
    Testing mode:
        ros2 launch finalworkv2_pkg dqn_navigation.launch.py mode:=test goal_x:=2.0 goal_y:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('finalworkv2_pkg')
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='train',
        description='Mode: train or test'
    )
    
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='0.0',
        description='Goal X position (test mode)'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='0.0',
        description='Goal Y position (test mode)'
    )
    
    max_episodes_arg = DeclareLaunchArgument(
        'max_episodes',
        default_value='500',
        description='Maximum training episodes'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/dqn_model.pkl',
        description='Path to save/load model'
    )
    
    # Config file path
    config_file = os.path.join(pkg_share, 'config', 'dqn_params.yaml')
    
    # Training node
    train_node = Node(
        package='finalworkv2_pkg',
        executable='train_node',
        name='train_node',
        output='screen',
        parameters=[
            config_file,
            {
                'mode': LaunchConfiguration('mode'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'max_episodes': LaunchConfiguration('max_episodes'),
                'model_path': LaunchConfiguration('model_path'),
            }
        ]
    )
    
    return LaunchDescription([
        mode_arg,
        goal_x_arg,
        goal_y_arg,
        max_episodes_arg,
        model_path_arg,
        train_node,
    ])
