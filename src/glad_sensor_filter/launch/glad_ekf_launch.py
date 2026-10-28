#!/usr/bin/env python3
"""Launch EKF sensor fusion for G.L.A.D."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("glad_sensor_filter")
    config_path = os.path.join(pkg_share, "config", "glad_ekf_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot_localization = LaunchConfiguration("use_robot_localization")

    # 1. Industrial C++ EKF via robot_localization (active when use_robot_localization:=true)
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="glad_ekf_node",
        parameters=[
            config_path,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        remappings=[("odometry/filtered", "odom")],
        condition=IfCondition(use_robot_localization),
    )

    # 2. Built-in Python 2D EKF (active by default, zero extra C++ dependencies required)
    builtin_ekf_node = Node(
        package="glad_sensor_filter",
        executable="glad_ekf_node",
        name="glad_ekf_node",
        parameters=[
            config_path,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        remappings=[("odometry/filtered", "odom")],
        condition=UnlessCondition(use_robot_localization),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock source (/clock)",
            ),
            DeclareLaunchArgument(
                "use_robot_localization",
                default_value="false",
                description="Set true to use C++ robot_localization, or false for built-in glad_ekf_node",
            ),
            robot_localization_node,
            builtin_ekf_node,
        ]
    )
