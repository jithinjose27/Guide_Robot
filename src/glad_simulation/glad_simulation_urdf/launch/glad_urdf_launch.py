import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    pkg_share = get_package_share_directory("glad_simulation_urdf")

    default_model_path = os.path.join(pkg_share, "urdf", "glad_final_description.xacro")
    default_yaml_path = os.path.join(pkg_share, "config", "glad_urdf.yaml")
    default_rviz_path = os.path.join(pkg_share, "rviz", "glad_sim.rviz")
    if not os.path.exists(default_rviz_path):
        default_rviz_path = os.path.join(pkg_share, "rviz", "glad_urdf.rviz")

    def _gazebo_launch(context):
        world_path = context.launch_configurations.get("world", "").strip()
        gz_args = f"-r {world_path}" if world_path else "-r empty.sdf"

        # Check if ros_gz_sim is available
        try:
            gz_sim_share = get_package_share_directory("ros_gz_sim")
            gz_sim_launch = os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gz_sim_launch),
                    launch_arguments={"gz_args": gz_args}.items(),
                )
            ]
        except PackageNotFoundError:
            return [
                LogInfo(
                    msg=(
                        "\n=======================================================\n"
                        "[WARNING] 'ros_gz_sim' package is not installed.\n"
                        "To launch the Gazebo Harmonic simulator, install it with:\n"
                        "    sudo apt install ros-jazzy-ros-gz\n"
                        "=======================================================\n"
                    )
                )
            ]

    gazebo_launch = OpaqueFunction(function=_gazebo_launch)

    def launch_setup(context):
        import xacro

        model_path = context.launch_configurations.get("model", default_model_path)
        yaml_path = context.launch_configurations.get("yaml_file", default_yaml_path)
        use_sim_time = context.launch_configurations.get("use_sim_time", "true")
        rviz_config = context.launch_configurations.get(
            "rviz_config", default_rviz_path
        )
        launch_rviz = context.launch_configurations.get("launch_rviz", "true")

        doc = xacro.process_file(
            model_path,
            mappings={
                "yaml_file": yaml_path,
                "bringup": "simulation",
            },
        )
        robot_description = doc.documentElement.toprettyxml(indent="  ")

        use_sim = use_sim_time.strip().lower() in ("true", "1")
        show_rviz = launch_rviz.strip().lower() in ("true", "1")

        nodes = [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim},
                    {"robot_description": robot_description},
                    {"ignore_timestamp": True},
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim},
                    {"robot_description": robot_description},
                ],
            ),
            # ROS-Gazebo Sim Topic Bridge (ROS 2 Jazzy Harmonic)
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim}],
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom/wheel@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/imu/raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                ],
            ),
            # Entity spawner into Gazebo Sim
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_glad",
                arguments=[
                    "-topic", "robot_description",
                    "-name", "glad",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.1",
                ],
                output="screen",
            ),
        ]

        if show_rviz:
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim}],
                    output="screen",
                )
            )

        return nodes

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="Path to robot URDF/Xacro file",
            ),
            DeclareLaunchArgument(
                "yaml_file",
                default_value=default_yaml_path,
                description="Path to robot parameters YAML file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time (/clock topic)",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz2 visualization window",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_path,
                description="Path to RViz configuration file",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="",
                description="Path to Gazebo world SDF file (empty = default empty world)",
            ),
            gazebo_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
