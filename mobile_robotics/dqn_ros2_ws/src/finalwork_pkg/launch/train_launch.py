#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Launch file para DQN Navigation Training

Inicia:
1. Stage ROS2 con cave.world
2. Training Node

Uso:
    ros2 launch finalwork_pkg train_launch.py
    ros2 launch finalwork_pkg train_launch.py num_episodes:=500
    ros2 launch finalwork_pkg train_launch.py world:=hallway

Autor: Proyecto DQN Navigation - finalwork_pkg
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Genera la descripción del launch para entrenamiento.
    """
    
    # Directorio del paquete stage_ros2
    stage_ros2_dir = get_package_share_directory('stage_ros2')
    
    # ========================================================================
    # ARGUMENTOS DE LAUNCH
    # ========================================================================
    
    # Archivo world para Stage
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='cave'),
        description='World file name (without .world extension)'
    )
    
    # Parámetros de Stage
    use_stamped_velocity_arg = DeclareLaunchArgument(
        'use_stamped_velocity',
        default_value='false',
        description='Use TwistStamped instead of Twist'
    )
    
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='Publish all TFs to global /tf topic'
    )
    
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='Use prefixes for single robot'
    )
    
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames'
    )
    
    # Parámetros de entrenamiento
    num_episodes_arg = DeclareLaunchArgument(
        'num_episodes',
        default_value='1000',
        description='Number of training episodes'
    )
    
    save_interval_arg = DeclareLaunchArgument(
        'save_interval',
        default_value='100',  # Sincronizado con train_node.py DEFAULT_SAVE_INTERVAL
        description='Save model every N episodes'
    )
    
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='dqn_model.pkl',
        description='Name of the model file'
    )
    
    # ========================================================================
    # CONFIGURACIÓN DEL WORLD FILE
    # ========================================================================
    
    def stage_world_configuration(context):
        """Construye la ruta completa al archivo .world"""
        world_name = context.launch_configurations['world']
        file_path = os.path.join(stage_ros2_dir, 'world', world_name + '.world')
        return [SetLaunchConfiguration('world_file', file_path)]
    
    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)
    
    # ========================================================================
    # NODOS
    # ========================================================================
    
    # 1. STAGE SIMULATOR
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        parameters=[{
            'one_tf_tree': LaunchConfiguration('one_tf_tree'),
            'enforce_prefixes': LaunchConfiguration('enforce_prefixes'),
            'use_stamped_velocity': LaunchConfiguration('use_stamped_velocity'),
            'use_static_transformations': LaunchConfiguration('use_static_transformations'),
            'world_file': LaunchConfiguration('world_file')
        }],
        output='screen'
    )
    
    # 2. TRAINING NODE (esperar 3 segundos para que Stage inicie)
    training_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='finalwork_pkg',
                executable='train_node',
                name='dqn_training',
                parameters=[{
                    'num_episodes': LaunchConfiguration('num_episodes'),
                    'save_interval': LaunchConfiguration('save_interval'),
                    'model_name': LaunchConfiguration('model_name'),
                }],
                output='screen'
            )
        ]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Arguments - Stage
        stage_world_arg,
        use_stamped_velocity_arg,
        one_tf_tree_arg,
        enforce_prefixes_arg,
        use_static_transformations_arg,
        stage_world_configuration_arg,
        
        # Arguments - Training
        num_episodes_arg,
        save_interval_arg,
        model_name_arg,
        
        # Nodes
        stage_node,
        training_node,
    ])
