from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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
    use_xterm = LaunchConfiguration("use_xterm")
    launch_teleop = LaunchConfiguration("launch_teleop")
    base_args = {"use_sim_time": use_sim_time}

    teleop_config = PathJoinSubstitution(
        [FindPackageShare("glad_simulation_teleop"), "config", "teleop.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "use_xterm",
                default_value="false",
                description="Launch teleop in a separate xterm window (set true for GUI desktop)",
            ),
            DeclareLaunchArgument(
                "launch_teleop",
                default_value="true",
                description="Launch keyboard teleop node",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=teleop_config,
                description="Path to teleop parameters YAML file",
            ),
            _include(
                "glad_simulation_trajectory_controller",
                "trajectory_controller_launch.py",
                base_args,
            ),
            _include(
                "glad_simulation_twist_mux",
                "twist_mux_launch.py",
                base_args,
            ),
            _include(
                "glad_simulation_motor_controller",
                "motor_controller.launch.py",
                base_args,
            ),
            Node(
                package="glad_simulation_teleop",
                executable="teleop_node",
                name="teleop_keyboard",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )

