from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # === Parámetro para elegir archivo URDF ===
    robot_file_arg = DeclareLaunchArgument(
        "robot_file",
        default_value="cyl_robot.urdf",
        description="Nombre del archivo URDF del robot dentro de la carpeta 'urdf'"
    )

    robot_file = LaunchConfiguration("robot_file")
    pkg_share = FindPackageShare("robots_description")

    # === Ruta relativa hacia el URDF ===
    urdf_path = PathJoinSubstitution([
        pkg_share,
        "urdf",
        robot_file
    ])
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare("robots_description"),
        "rviz",
        "urdf.rviz"
    ])

    # === Nodo robot_state_publisher ===
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "robot_description": ParameterValue(
                Command(['cat ', urdf_path]),
                value_type=str
            )
        }]
    )

    # === Nodo joint_state_publisher_gui ===
    jsp_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    # === Nodo RViz ===
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config]
    )

    return LaunchDescription([
        robot_file_arg,
        rsp_node,
        jsp_node,
        rviz_node
    ])

