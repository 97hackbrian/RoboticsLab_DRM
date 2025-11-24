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
    pkg_my_robot = get_package_share_directory('industrial_mobile_robot')

    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'industrial_mobile_robot.urdf')
    world = os.path.join(pkg_my_robot, 'worlds', 'small_warehouse.world')

    # Forzar path correcto para Gazebo
    ign_resource_path = os.pathsep.join([
        os.path.dirname(pkg_my_robot),  # esto apunta a .../share
    ])
    
    # Forzar path correcto para Gazebo
    ign_resource_path = os.pathsep.join([
        os.path.dirname(pkg_my_robot),             # .../share (original)
        os.path.join(pkg_my_robot, 'models'),      # .../share/industrial_mobile_robot/models
        os.path.join(pkg_my_robot, 'meshes'),      # .../share/industrial_mobile_robot/meshes
    ])

    
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_GUI_CONFIG_FILE', ''),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        OpaqueFunction(function=print_env),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world}'}.items(),
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
                '-name', 'industrial_mobile_robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '6.0',
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
                
                '/joint_b1/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_b2/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_b3/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_b4/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_b5/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_b6/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                
                '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            ],
            output='screen'
        ),
    ])


