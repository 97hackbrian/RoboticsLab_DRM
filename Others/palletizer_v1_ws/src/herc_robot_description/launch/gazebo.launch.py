from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def print_env(context, *args, **kwargs):
    print("IGN_GAZEBO_RESOURCE_PATH =", os.environ.get("IGN_GAZEBO_RESOURCE_PATH"))
    return []

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_robot = get_package_share_directory('herc_robot_description')

    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'herc_robot_description.urdf')
    mars_world = os.path.join(pkg_my_robot, 'worlds', 'mars.world')

    # Forzar path correcto para Gazebo
    ign_resource_path = os.pathsep.join([
        os.path.dirname(pkg_my_robot),  # esto apunta a .../share
    ])
    
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_GUI_CONFIG_FILE', ''),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        OpaqueFunction(function=print_env),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {mars_world}'}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'herc_robot_description',
                '-x', '11.0',
                '-y', '21.0',
                '-z', '8.0',
                '-Y', '0.0'
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                
                '/joint_st1l/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_st2l/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_st1r/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_st2r/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            ],
            output='screen'
        ),
    ])

