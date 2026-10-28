#!/usr/bin/env python3
"""
Full System Bringup Launch for G.L.A.D Mobile Robot.

Sequences:
1. Static transforms (base_link -> laser, base_link -> imu_link)
2. RPLidar hardware serial driver
3. Laser filter & resampler
4. BNO055 IMU hardware driver
5. Madgwick IMU orientation filter
6. RF2O laser odometry
7. Sensor fusion EKF (/odom)
8. SLAM Toolbox (if enable_slam:=true) OR Nav2 Stack (if enable_slam:=false)
9. Motion control stack (teleop, twist mux, trajectory limiter, ESP32 bridge)
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
    launch_teleop = LaunchConfiguration("launch_teleop")

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
            description="Start slam_toolbox online async mapping (replaces Nav2).",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value=default_map,
            description="Absolute path to map YAML file (used when enable_slam:=false).",
        ),
        DeclareLaunchArgument(
            "launch_teleop",
            default_value="true",
            description="Whether to launch teleop keyboard node alongside motion control.",
        ),
    ]
    base_args = {"use_sim_time": use_sim_time}

    # 1. Static TF
    static_tf = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 1. Mounting Static Transforms (base_link -> laser, imu)..."),
            _include("glad_static_tf", "glad_static_tf_launch.py", base_args),
        ]
    )

    # 2. RPLidar Driver
    glad_rplidar_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 2. Activating RPLidar Sensor Driver..."),
            _include("glad_rplidar", "glad_rplidar_launch.py", base_args),
        ]
    )

    # 3. Laser Filter
    glad_laser_filter_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 3. Initializing Laser Filter & Resampler..."),
            _include("glad_laser_filter", "glad_laser_filter_launch.py", base_args),
        ]
    )

    # 4. IMU Publisher
    glad_imu_publisher_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 4. Activating BNO055 IMU Driver..."),
            _include("glad_imu_publisher", "imu_publisher_launch.py", base_args),
        ]
    )

    # 5. Madgwick Filter
    glad_imu_filter_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 5. Booting IMU Madgwick Orientation Filter..."),
            _include("glad_imu_filter", "imu_madgwick_launch.py", base_args),
        ]
    )

    # 6. Laser Odometry
    rf2o_laser_odometry_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 6. Starting RF2O Laser Odometry..."),
            _include("rf2o_laser_odometry", "rf2o_laser_odometry_launch.py", base_args),
        ]
    )

    # 7. EKF Sensor Fusion
    glad_ekf_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 7. Deploying Extended Kalman Filter (/odom)..."),
            _include("glad_sensor_filter", "glad_ekf_launch.py", base_args),
        ]
    )

    # 8. SLAM Mode
    slam_toolbox_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 8. Launching SLAM Toolbox for Online Mapping..."),
            _include("glad_mapping", "slam_toolbox_launch.py", base_args),
        ],
        condition=IfCondition(enable_slam),
    )

    # 9. Motion Control
    teleop_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 9. Activating Motion Control Stack (Mux, Trajectory, Motor)..."),
            _include(
                "glad_teleop",
                "teleop_launch.py",
                {**base_args, "launch_teleop": launch_teleop, "use_xterm": "false"},
            ),
        ]
    )

    # 10. Nav2 Autonomy
    glad_nav2_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Bringup] 10. Launching Nav2 Stack (AMCL + Planners)..."),
            _include(
                "glad_navigation",
                "glad_nav2_launch.py",
                {**base_args, "map": map_yaml},
            ),
        ],
        condition=UnlessCondition(enable_slam),
    )

    return LaunchDescription(
        args
        + [
            LogInfo(msg="========== G.L.A.D SYSTEM BRINGUP LAUNCHED =========="),
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
        ]
    )
