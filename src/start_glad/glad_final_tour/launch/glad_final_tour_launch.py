#!/usr/bin/env python3
"""
Final Autonomous Tour Showcase Bringup for G.L.A.D.

Launches the complete G.L.A.D stack (sensors, filtering, EKF, Nav2, motion)
and triggers the autonomous AI tour guide (glad_tour) to navigate designated
stations and deliver spoken technical commentary.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package: str, filename: str, launch_arguments: dict = None):
    if launch_arguments is None:
        launch_arguments = {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_slam = LaunchConfiguration("enable_slam")
    map_yaml = LaunchConfiguration("map_yaml")

    default_map = PathJoinSubstitution(
        [FindPackageShare("glad_maps"), "robotics_floor", "robotics_floor.yaml"]
    )

    args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock source (/clock topic).",
        ),
        DeclareLaunchArgument(
            "enable_slam",
            default_value="false",
            description="Start slam_toolbox online async mapping (replaces static Nav2).",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value=default_map,
            description="Absolute path to map YAML file (used when enable_slam:=false).",
        ),
    ]
    base_args = {"use_sim_time": use_sim_time}

    # 1. Static TF
    static_tf = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 1. Mounting Static Transforms..."),
            _include("glad_static_tf", "glad_static_tf_launch.py", base_args),
        ]
    )

    # 2. RPLidar Driver
    glad_rplidar_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 2. Activating RPLidar Sensor Driver..."),
            _include("glad_rplidar", "glad_rplidar_launch.py", base_args),
        ]
    )

    # 3. Laser Filter
    glad_laser_filter_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 3. Initializing Laser Filter Node..."),
            _include("glad_laser_filter", "glad_laser_filter_launch.py", base_args),
        ]
    )

    # 4. IMU Publisher
    glad_imu_publisher_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 4. Activating IMU Hardware Driver..."),
            _include("glad_imu_publisher", "imu_publisher_launch.py", base_args),
        ]
    )

    # 5. Madgwick Filter
    glad_imu_filter_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 5. Booting IMU Madgwick Filter..."),
            _include("glad_imu_filter", "imu_madgwick_launch.py", base_args),
        ]
    )

    # 6. Laser Odometry
    rf2o_laser_odometry_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 6. Starting RF2O Laser Odometry..."),
            _include("rf2o_laser_odometry", "rf2o_laser_odometry_launch.py", base_args),
        ]
    )

    # 7. EKF Sensor Fusion
    glad_ekf_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 7. Deploying Extended Kalman Filter (/odom)..."),
            _include("glad_sensor_filter", "glad_ekf_launch.py", base_args),
        ]
    )

    # 8. SLAM Mode
    slam_toolbox_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 8. Launching SLAM Toolbox..."),
            _include("glad_mapping", "slam_toolbox_launch.py", base_args),
        ],
        condition=IfCondition(enable_slam),
    )

    # 9. Motion Control
    teleop_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 9. Activating Motion Control Stack..."),
            _include(
                "glad_teleop",
                "teleop_launch.py",
                {**base_args, "launch_teleop": "false", "use_xterm": "false"},
            ),
        ]
    )

    # 10. Nav2 Autonomy
    glad_nav2_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 10. Launching Nav2 Stack..."),
            _include(
                "glad_navigation",
                "glad_nav2_launch.py",
                {**base_args, "map": map_yaml},
            ),
        ],
        condition=UnlessCondition(enable_slam),
    )

    # 11. Autonomous Tour Guide
    glad_tour_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Final Tour] 11. Initializing Autonomous AI Tour Guide..."),
            _include("glad_tour", "glad_tour_launch.py", base_args),
        ]
    )

    return LaunchDescription(
        args
        + [
            LogInfo(msg="========== G.L.A.D FINAL TOUR SHOWCASE STARTED =========="),
            static_tf,
            glad_rplidar_launch,
            TimerAction(period=1.0, actions=[glad_laser_filter_launch]),
            TimerAction(period=1.5, actions=[glad_imu_publisher_launch]),
            TimerAction(period=2.0, actions=[glad_imu_filter_launch]),
            TimerAction(period=2.5, actions=[rf2o_laser_odometry_launch]),
            TimerAction(period=3.0, actions=[glad_ekf_launch]),
            TimerAction(period=3.5, actions=[slam_toolbox_launch]),
            TimerAction(period=4.0, actions=[teleop_launch]),
            TimerAction(period=6.0, actions=[glad_nav2_launch]),
            TimerAction(period=10.0, actions=[glad_tour_launch]),
        ]
    )
