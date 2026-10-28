#!/usr/bin/env python3
"""Launch SLAM Toolbox for online laser mapping on G.L.A.D."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("glad_mapping")
    config_path = os.path.join(pkg_share, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock source (/clock)",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    config_path,
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
        ]
    )
