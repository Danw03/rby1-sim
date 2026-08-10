#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_file = LaunchConfiguration("map")
    amcl_params_file = LaunchConfiguration(
        "amcl_params_file"
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_file,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([
                FindPackageShare("rby1_navigation"),
                "maps",
                "test_map.yaml",
            ]),
            description="Map YAML file used by Map Server.",
        ),

        DeclareLaunchArgument(
            "amcl_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rby1_navigation"),
                "config",
                "amcl_sim.yaml",
            ]),
            description="AMCL parameter file.",
        ),

        map_server,
        amcl,
        lifecycle_manager,
    ])