import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("glad_simulation_sensor_filter"),
        "config",
        "glad_ekf_params.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot_loc = LaunchConfiguration("use_robot_localization")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock (/clock)",
            ),
            DeclareLaunchArgument(
                "use_robot_localization",
                default_value="false",
                description="If true, use C++ robot_localization::ekf_node; if false, use built-in Python glad_ekf_node",
            ),
            # Built-in lightweight 2D EKF (default: zero extra dependencies needed)
            Node(
                package="glad_simulation_sensor_filter",
                executable="glad_ekf_node",
                name="glad_ekf_node",
                output="screen",
                parameters=[
                    config_path,
                    {"use_sim_time": use_sim_time},
                ],
                condition=UnlessCondition(use_robot_loc),
            ),
            # C++ robot_localization EKF (optional if package is installed)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="glad_ekf_node",
                parameters=[
                    config_path,
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
                remappings=[("odometry/filtered", "odom")],
                condition=IfCondition(use_robot_loc),
            ),
        ]
    )
