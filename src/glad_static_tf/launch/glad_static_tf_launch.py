#!/usr/bin/env python3
"""Launch static transform broadcaster for G.L.A.D sensors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock source",
            ),
            Node(
                package="glad_static_tf",
                executable="static_tf_broadcaster",
                name="static_tf_broadcaster",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
