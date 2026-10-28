#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("glad_simulation_navigation"),
        "config",
        "glad_simulation_nav2_params.yaml",
    )

    default_map = PathJoinSubstitution(
        [FindPackageShare("glad_maps"), "robotics_floor", "robotics_floor.yaml"]
    )

    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to map YAML file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock source",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition lifecycle nodes to active",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    config_path,
                    {"yaml_filename": map_yaml_file, "use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[config_path, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[config_path, {"use_sim_time": use_sim_time}],
                output="screen",
                remappings=[("cmd_vel", "/cmd_vel/nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[config_path, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[config_path, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[config_path, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": [
                            "map_server",
                            "amcl",
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                        ],
                    }
                ],
                output="screen",
            ),
        ]
    )
