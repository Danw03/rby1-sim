#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DRIVER_LAUNCH_FILE = "rby1_ros2_driver.launch.py"


def include_launch(
    package_name,
    launch_file,
    launch_arguments=None,
    condition=None,
):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                "launch",
                launch_file,
            ])
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    start_control_ui = LaunchConfiguration("start_control_ui")
    start_rviz = LaunchConfiguration("start_rviz")

    # 1. RBY1 ROS 2 Driver
    driver = include_launch(
        package_name="rby1_driver",
        launch_file=DRIVER_LAUNCH_FILE,
    )

    # 2. Custom Control UI
    control_ui = TimerAction(
        period=2.0,
        actions=[
            include_launch(
                package_name="rby1_control_ui",
                launch_file="control_ui.launch.py",
                condition=IfCondition(start_control_ui),
            )
        ],
    )

    # 3. MuJoCo pose bridge, dual LiDAR, scan merger, static TF
    lidar_stack = TimerAction(
        period=3.0,
        actions=[
            include_launch(
                package_name="rby1_bringup",
                launch_file="sim.launch.py",
            )
        ],
    )

    # 4. SLAM Toolbox online asynchronous mapping
    slam_mapping = TimerAction(
        period=6.0,
        actions=[
            include_launch(
                package_name="slam_toolbox",
                launch_file="online_async_launch.py",
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params_file,
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                },
            )
        ],
    )

    # 5. RViz
    rviz = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rby1_mapping_rviz",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                }],
                condition=IfCondition(start_rviz),
            )
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=[
                "/root/sdk/rby1-sdk/lib:",
                EnvironmentVariable(
                    "LD_LIBRARY_PATH",
                    default_value="",
                ),
            ],
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=(
                "Keep false because the RBY1 simulator "
                "does not publish /clock."
            ),
        ),

        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rby1_navigation"),
                "config",
                "slam_mapping.yaml",
            ]),
        ),

        DeclareLaunchArgument(
            "start_control_ui",
            default_value="true",
            description="Start the custom RBY1 Control UI.",
        ),

        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 for mapping.",
        ),

        driver,
        control_ui,
        lidar_stack,
        slam_mapping,
        rviz,
    ])