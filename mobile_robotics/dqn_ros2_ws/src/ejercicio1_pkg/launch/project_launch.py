#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Launch file para DQN Navigation Training con Stage

Este launch file inicia:
1. Stage simulator con el mundo cave.world
2. Nodo reset_stage (wrapper de odometría con servicio de reset)
3. Nodo de entrenamiento DQN

Uso:
    ros2 launch ejercicio1_pkg project_launch.py
    ros2 launch ejercicio1_pkg project_launch.py world:=hallway

Autor: Proyecto Académico DQN Navigation
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
    Genera la descripción del launch.
    """
    
    # Directorio del paquete stage_ros2
    stage_ros2_dir = get_package_share_directory('stage_ros2')
    
    # ========================================================================
    # ARGUMENTOS DE LAUNCH
    # ========================================================================
    
    # Archivo world para Stage (sin extensión .world)
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='cave'),
        description='World file name (without .world extension)'
    )
    
    # Usar timestamps en velocidad (Stage)
    use_stamped_velocity_arg = DeclareLaunchArgument(
        'use_stamped_velocity',
        default_value='false',
        description='Use TwistStamped instead of Twist'
    )
    
    # Usar un solo árbol TF
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='Publish all TFs to global /tf topic'
    )
    
    # Usar prefijos para robots
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='Use prefixes for single robot'
    )
    
    # Usar transformaciones estáticas
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames'
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
    
    # 2. RESET STAGE NODE (wrapper de odometría)
    # Esperamos 2 segundos para que Stage inicie
    reset_stage_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ejercicio1_pkg',
                executable='reset_stage',
                name='odom_reset_wrapper',
                output='screen'
            )
        ]
    )
    
    # 3. TRAINING NODE
    # Esperamos 5 segundos para que Stage y reset_stage inicien
    training_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ejercicio1_pkg',
                executable='train_node',
                name='dqn_training',
                output='screen'
            )
        ]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Arguments
        stage_world_arg,
        use_stamped_velocity_arg,
        one_tf_tree_arg,
        enforce_prefixes_arg,
        use_static_transformations_arg,
        stage_world_configuration_arg,
        
        # Nodes
        stage_node,
        reset_stage_node,
        training_node,
    ])
