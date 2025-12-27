from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Levantar controladores de los servos
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_st1l_position_controller', '-c', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_st2l_position_controller', '-c', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_st1r_position_controller', '-c', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_st2r_position_controller', '-c', '/controller_manager'],
            output='screen'
        ),

        # Nodo de control de ángulos de servos
        Node(
            package='herc_robot_control',
            executable='servo_controller',
            output='screen'
        )
    ])
