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
    world_path = LaunchConfiguration("world")

    default_world = PathJoinSubstitution(
        [FindPackageShare("start_glad_simulation"), "worlds", "warehouse.world"]
    )

    args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock source (/clock topic).",
        ),
        DeclareLaunchArgument(
            "enable_slam",
            default_value="true",
            description="Start slam_toolbox online async mapping.",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="",
            description="Absolute path to map YAML file (required when enable_slam:=false).",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Absolute path to Gazebo world SDF file.",
        ),
    ]
    base_args = {"use_sim_time": use_sim_time, "world": world_path}

    glad_urdf_launch = GroupAction(
        [
            LogInfo(
                msg="[GLAD Simulation] 1. Launching URDF, Gazebo, and Robot State Publisher..."
            ),
            _include("glad_simulation_urdf", "glad_urdf_launch.py", base_args),
        ]
    )

    teleop_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Simulation] 2. Launching Teleop and Motion Control..."),
            _include("glad_simulation_teleop", "teleop_launch.py", base_args),
        ]
    )

    sensor_filter_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Simulation] 3. Launching Sensor Filter (EKF)..."),
            _include("glad_simulation_sensor_filter", "glad_ekf_launch.py", base_args),
        ]
    )

    slam_toolbox_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Simulation] 4. Launching SLAM Toolbox..."),
            _include("glad_simulation_mapping", "slam_toolbox_launch.py", base_args),
        ],
        condition=IfCondition(enable_slam),
    )

    glad_nav2_launch = GroupAction(
        [
            LogInfo(msg="[GLAD Simulation] 5. Launching Nav2 Stack..."),
            _include(
                "glad_simulation_navigation",
                "glad_simulation_nav2_launch.py",
                {**base_args, "map": map_yaml},
            ),
        ],
        condition=UnlessCondition(enable_slam),
    )

    return LaunchDescription(
        args
        + [
            LogInfo(msg="========== GLAD SIMULATION STARTED =========="),
            glad_urdf_launch,
            TimerAction(period=1.0, actions=[teleop_launch]),
            TimerAction(period=2.0, actions=[sensor_filter_launch]),
            TimerAction(period=3.0, actions=[slam_toolbox_launch]),
            TimerAction(period=3.0, actions=[glad_nav2_launch]),
        ]
    )
