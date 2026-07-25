from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    teleop_config = os.path.join(
        get_package_share_directory("glad_teleop"),
        "config",
        "teleop.yaml",
    )

    trajectory_launch = os.path.join(
        get_package_share_directory("glad_trajectory_controller"),
        "launch",
        "trajectory_controller_launch.py",
    )

    twist_mux_launch = os.path.join(
        get_package_share_directory("glad_twist_mux"),
        "launch",
        "twist_mux_launch.py",
    )

    motor_launch = os.path.join(
        get_package_share_directory("glad_motor_controller"),
        "launch",
        "motor_controller.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(trajectory_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(twist_mux_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(motor_launch)),
            DeclareLaunchArgument(
                "config_file",
                default_value=teleop_config,
                description="Path to teleop parameters YAML file",
            ),
            Node(
                package="glad_teleop",
                executable="teleop_node",
                name="teleop_keyboard",
                prefix="xterm -hold -T GLAD_Teleop -e",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
