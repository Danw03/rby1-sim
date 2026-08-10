from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    lakibeam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rby1_lakibeam_sim'),
                'launch',
                'lakibeam.launch.py',
            ])
        )
    )

    return LaunchDescription([
        lakibeam_launch,
    ])
