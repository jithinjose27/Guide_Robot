import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("glad_simulation_urdf")

    default_model_path = os.path.join(pkg_share, "urdf", "glad_final_description.xacro")
    default_yaml_path = os.path.join(pkg_share, "config", "glad_urdf.yaml")
    default_rviz_path = os.path.join(pkg_share, "rviz", "glad_urdf.rviz")

    def _gazebo_launch(context):
        world_path = context.launch_configurations.get("world", "")
        world_args = {"world": world_path} if world_path else {}
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    )
                ),
                launch_arguments=world_args.items(),
            )
        ]

    gazebo_launch = OpaqueFunction(function=_gazebo_launch)

    def launch_setup(context):
        from xacro import process_file

        model_path = context.launch_configurations.get("model", default_model_path)
        yaml_path = context.launch_configurations.get("yaml_file", default_yaml_path)
        use_sim_time = context.launch_configurations.get("use_sim_time", "true")
        rviz_config = context.launch_configurations.get(
            "rviz_config", default_rviz_path
        )

        doc = process_file(
            model_path,
            mappings={
                "yaml_file": yaml_path,
                "bringup": "simulation",
            },
        )
        robot_description = doc.documentElement.toprettyxml(indent="  ")

        use_sim = use_sim_time.strip().lower() in ("true", "1")

        return [
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
                parameters=[{"use_sim_time": use_sim}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_glad",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "glad",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.0",
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim}],
                output="screen",
            ),
        ]

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
                description="Use simulation time",
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
