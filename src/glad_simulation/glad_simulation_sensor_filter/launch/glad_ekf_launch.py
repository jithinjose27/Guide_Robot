import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("glad_simulation_sensor_filter"),
        "config",
        "glad_ekf_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="glad_ekf_node",
                parameters=[
                    config_path,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                output="screen",
                remappings=[("odometry/filtered", "odom")],
            ),
        ]
    )
