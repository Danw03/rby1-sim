#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        "rby1_lakibeam_sim"
    )

    config_file = os.path.join(
        package_share,
        "config",
        "lakibeam.yaml",
    )

    return LaunchDescription([
        # Dynamic odom -> base_footprint is published by RBY1 Driver.

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_footprint_to_base_link",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "base_link",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_link_to_base_scan",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "base_scan",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_scan_to_laser_front_right",
            output="screen",
            arguments=[
                "--x", "0.2725",
                "--y", "-0.220",
                "--z", "0.185",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "-0.7853981634",
                "--frame-id", "base_scan",
                "--child-frame-id", "laser_front_right",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_scan_to_laser_rear_left",
            output="screen",
            arguments=[
                "--x", "-0.2725",
                "--y", "0.220",
                "--z", "0.185",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "2.3561944902",
                "--frame-id", "base_scan",
                "--child-frame-id", "laser_rear_left",
            ],
        ),

        Node(
            package="rby1_lakibeam_sim",
            executable="mujoco_pose_bridge_node",
            name="mujoco_pose_bridge_node",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="rby1_lakibeam_sim",
            executable="dual_lidar_node",
            name="dual_lidar_node",
            output="screen",
            parameters=[config_file],
        ),

        Node(
            package="rby1_lakibeam_sim",
            executable="scan_merger_node",
            name="scan_merger_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
