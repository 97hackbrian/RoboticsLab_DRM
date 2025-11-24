import os
import time
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    PythonExpression,
)
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent
from launch.events import Shutdown


def generate_launch_description():

    # --- Arguments ---
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="forward",
            description="Select kinematics mode: 'forward' or 'inverse'",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="yaskawa_robot",
            description="Package containing the robot description (URDF/XACRO).",
        )
    )

    mode = LaunchConfiguration("mode")
    description_package = LaunchConfiguration("description_package")

    # --- Function to dynamically configure based on package availability ---
    def launch_setup(context: LaunchContext, *args, **kwargs):
        pkg_name = description_package.perform(context)

        # Check if the package exists
        try:
            pkg_share = FindPackageShare(pkg_name).perform(context)
            if not os.path.isdir(pkg_share):
                raise FileNotFoundError
            fallback_pkg = FindPackageShare("kinematics_mh12").perform(context)
            urdf_path = PathJoinSubstitution([pkg_share, "urdf", "mh12.xacro"])
            rviz_config = PathJoinSubstitution([fallback_pkg, "rviz", "robot_view.rviz"])
        except Exception:
            # Print error message
            print("\033[91m[ERROR]\033[0m Package 'yaskawa_robot' not found!")
            print("Please install or build 'yaskawa_robot' in the same workspace before launching.")
            print("Using fallback RViz configuration from 'kinematics_mh12' package.\n")

            # Fallback to local rviz config in kinematics_mh12
    
            urdf_path = PathJoinSubstitution([fallback_pkg, "urdf", "mh12.xacro"])
            rviz_config = PathJoinSubstitution([fallback_pkg, "rviz", "robot_view.rviz"])

        # --- Load URDF from Xacro ---
        robot_description_content = Command(
            [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_path]
        )

        robot_description = {
            "robot_description": ParameterValue(value=robot_description_content, value_type=str)
        }

        # --- Nodes ---
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
        )

        joint_state_publisher_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(PythonExpression(["'", mode, "' == 'forward'"])),
        )

        forward_node = Node(
            package="kinematics_mh12",
            executable="forward",
            name="forward_kinematics_node",
            output="screen",
            condition=IfCondition(PythonExpression(["'", mode, "' == 'forward'"])),
        )

        inverse_node = Node(
            package="kinematics_mh12",
            executable="inverse",
            name="inverse_kinematics_node",
            output="screen",
            condition=IfCondition(PythonExpression(["'", mode, "' == 'inverse'"])),
        )

        # Initial Pos-
        current_time = int(time.time())
        zero_joint_publisher_node = ExecuteProcess(
            cmd=[
                "bash",
                "-c",
                f"sleep 3 && ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
                f'\'{{\"header\": {{\"stamp\": {{\"sec\": {current_time}, \"nanosec\": 0}}, \"frame_id\": \"\"}}, '
                f'\"name\": [\"joint_1_s\", \"joint_2_l\", \"joint_3_u\", \"joint_4_r\", \"joint_5_b\", \"joint_6_t\"], '
                f'\"position\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \"velocity\": [], \"effort\": []}}\''
            ],
            output="log",
            condition=IfCondition(PythonExpression(["'", mode, "' == 'inverse'"])),
        )

        return [
            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher_node,
            forward_node,
            inverse_node,
            zero_joint_publisher_node
        ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )