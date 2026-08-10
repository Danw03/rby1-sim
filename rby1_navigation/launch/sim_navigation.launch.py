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
    autostart = LaunchConfiguration("autostart")
    map_file = LaunchConfiguration("map")
    amcl_params_file = LaunchConfiguration(
        "amcl_params_file"
    )
    start_control_ui = LaunchConfiguration(
        "start_control_ui"
    )
    start_rviz = LaunchConfiguration("start_rviz")
    start_ab_navigation_demo = LaunchConfiguration(
        "start_ab_navigation_demo"
    )

    # 1. RBY1 driver
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

    # 3. Simulated LiDAR stack
    lidar = TimerAction(
        period=3.0,
        actions=[
            include_launch(
                package_name="rby1_bringup",
                launch_file="sim.launch.py",
            )
        ],
    )

    # 4. Map Server + AMCL
    localization = TimerAction(
        period=6.0,
        actions=[
            include_launch(
                package_name="rby1_navigation",
                launch_file="localization.launch.py",
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "map": map_file,
                    "amcl_params_file": amcl_params_file,
                },
            )
        ],
    )

    # 5. Nav2 navigation servers
    navigation = TimerAction(
        period=10.0,
        actions=[
            include_launch(
                package_name="rby1_navigation",
                launch_file="navigation.launch.py",
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_composition": "False",
                },
            )
        ],
    )

    # 6. RViz
    rviz = TimerAction(
        period=13.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rby1_navigation_rviz",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                }],
                condition=IfCondition(start_rviz),
            )
        ],
    )

    # 7. Optional P1-P2 demo
    ab_navigation_demo = include_launch(
        package_name="rby1_navigation",
        launch_file="ab_navigation_demo.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
        },
        condition=IfCondition(
            start_ab_navigation_demo
        ),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="RBY1_SDK_PATH",
            value="/root/sdk/rby1-sdk",
        ),

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
                "False because the simulator does not publish /clock."
            ),
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
        ),

        DeclareLaunchArgument(
            "amcl_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rby1_navigation"),
                "config",
                "amcl_sim.yaml",
            ]),
        ),

        DeclareLaunchArgument(
            "start_control_ui",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "start_ab_navigation_demo",
            default_value="false",
        ),

        driver,
        control_ui,
        lidar,
        localization,
        navigation,
        rviz,
        ab_navigation_demo,
    ])