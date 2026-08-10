"""Launch the P1-P2 navigation demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    config_file = PathJoinSubstitution([
        FindPackageShare("rby1_navigation"),
        "config",
        "ab_navigation_demo.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),

        Node(
            package="rby1_navigation",
            executable="ab_navigation_demo",
            name="ab_navigation_demo",
            output="screen",
            parameters=[
                config_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])