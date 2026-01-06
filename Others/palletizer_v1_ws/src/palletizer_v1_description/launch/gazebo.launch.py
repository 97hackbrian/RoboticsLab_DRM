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
    caja_urdf_file = os.path.join(pkg_palletizer, 'urdf', 'caja1.urdf')
    world = os.path.join(pkg_palletizer, 'worlds', 'small_warehouse.world')
    bridge_config = os.path.join(pkg_palletizer, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(pkg_palletizer, 'rviz2', 'conf.rviz')

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
            parameters=[
                {'robot_description': open(urdf_file).read()},
                {'use_sim_time': True}
            ],
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
                '-z', '0.53',
                '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn de la caja 1 con ArUco marker
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', open(caja_urdf_file).read(),
                '-name', 'caja1',
                '-x', '1.5',
                '-y', '0.0',
                '-z', '0.1',
                '-Y', '3.1416'
            ],
            output='screen'
        ),

        # Spawn de la caja 2 con ArUco marker
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', open(caja_urdf_file).read(),
                '-name', 'caja2',
                '-x', '1.33',
                '-y', '1.6',
                '-z', '0.1',
                '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn de la caja 3 con ArUco marker
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', open(caja_urdf_file).read(),
                '-name', 'caja3',
                '-x', '-1.5',
                '-y', '0.5',
                '-z', '0.1',
                '-Y', '2.999'
            ],
            output='screen'
        ),

        # Bridge ROS2 <-> Gazebo usando archivo de configuración YAML
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen'
        ),

        # Pseudo Visual Odometry Publisher using Ground Truth (C++ node)
        # Simulates drift-free visual odometry like Intel T265
        # Listens to /tf_ground_truth and extracts palletizer_v1 pose
        # Publishes TF: odom_vo_frame -> base_footprint
        Node(
            package='palletizer_v1_description',
            executable='visual_odometry_publisher',
            name='visual_odometry_publisher',
            parameters=[
                {'model_name': 'palletizer_v1'},
                {'tf_ground_truth_topic': '/tf_ground_truth'},
                {'odom_frame': 'odom_vo_frame'},
                {'odom_topic': '/odom_vo'},
                {'child_frame_id': 'base_footprint'},
                {'publish_rate': 50.0},
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
