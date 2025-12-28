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
    pkg_palletizer = get_package_share_directory('palletizer_v1_description')

    urdf_file = os.path.join(pkg_palletizer, 'urdf', 'palletizer_v1_description.urdf')
    world = os.path.join(pkg_palletizer, 'worlds', 'small_warehouse.world')

    # Configurar paths para que Gazebo encuentre los modelos y meshes
    ign_resource_path = os.pathsep.join([
        os.path.dirname(pkg_palletizer),             # .../share (para encontrar el paquete)
        os.path.join(pkg_palletizer, 'models'),      # .../share/palletizer_v1_description/models
        os.path.join(pkg_palletizer, 'meshes'),      # .../share/palletizer_v1_description/meshes
    ])

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_GUI_CONFIG_FILE', ''),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        OpaqueFunction(function=print_env),

        # Lanzar Gazebo con el mundo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world}'}.items(),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}],
            output='screen'
        ),

        # Spawn del robot en Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'palletizer_v1',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.5',
                '-Y', '0.0'
            ],
            output='screen'
        ),

        # Bridge para comunicación ROS2 <-> Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
                # LiDAR 3D PointCloud (de Gazebo a ROS2)
                '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            ],
            output='screen'
        ),
    ])
