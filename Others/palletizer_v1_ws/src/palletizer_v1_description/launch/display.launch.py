import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    palletizer_pkg = get_package_share_directory("palletizer_v1_description")

    default_urdf_path = os.path.join(palletizer_pkg, "urdf", "palletizer_v1_description.urdf")

    urdf_path_arg = DeclareLaunchArgument(
        name="urdf_path",
        default_value=default_urdf_path
    )

    urdf_path = LaunchConfiguration("urdf_path")
    robot_description_content = open(default_urdf_path).read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2"
    )

    return LaunchDescription([
        urdf_path_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])