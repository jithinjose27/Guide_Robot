#!/usr/bin/env python3
"""Launch RPLidar sensor driver for G.L.A.D."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("glad_rplidar")
    config_path = os.path.join(pkg_share, "config", "rplidar_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    rplidar_node = Node(
        package="glad_rplidar",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[config_path, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock source",
            ),
            rplidar_node,
        ]
    )
