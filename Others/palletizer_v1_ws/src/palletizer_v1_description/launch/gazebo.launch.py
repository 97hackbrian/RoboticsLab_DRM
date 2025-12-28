from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
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
    bridge_config = os.path.join(pkg_palletizer, 'config', 'gz_bridge.yaml')
    visual_odom_script = os.path.join(pkg_palletizer, 'scripts', 'visual_odometry_publisher.py')

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

        # Bridge ROS2 <-> Gazebo usando archivo de configuración YAML
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen'
        ),

        # Visual Odometry Publisher (ejecutado con python3)
        ExecuteProcess(
            cmd=['python3', visual_odom_script,
                 '--ros-args',
                 '-p', 'camera_frame:=camera_link',
                 '-p', 'odom_frame:=odom',
                 '-p', 'publish_rate:=30.0'],
            output='screen'
        ),
    ])
