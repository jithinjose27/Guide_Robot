#!/usr/bin/env python3
"""Launch keyboard teleoperation and motion control stack for G.L.A.D."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
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

    launch_teleop = LaunchConfiguration("launch_teleop")
    use_xterm = LaunchConfiguration("use_xterm")

    # In a graphical environment with xterm installed, launch teleop in its own window
    teleop_xterm_node = Node(
        package="glad_teleop",
        executable="teleop_node",
        name="teleop_keyboard",
        prefix="xterm -hold -T GLAD_Teleop -e",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        condition=IfCondition(use_xterm),
    )

    # In standard terminal or headless setups, run teleop node directly
    teleop_direct_node = Node(
        package="glad_teleop",
        executable="teleop_node",
        name="teleop_keyboard",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        condition=IfCondition(launch_teleop),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_teleop",
                default_value="true",
                description="Whether to launch teleop keyboard node",
            ),
            DeclareLaunchArgument(
                "use_xterm",
                default_value="false",
                description="Launch teleop in separate xterm window (requires xterm installed)",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=teleop_config,
                description="Path to teleop parameters YAML file",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(trajectory_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(twist_mux_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(motor_launch)),
            teleop_direct_node,
        ]
    )
