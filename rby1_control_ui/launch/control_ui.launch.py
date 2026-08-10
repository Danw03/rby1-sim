from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('rby1_control_ui'))
    default_config = str(package_share / 'config' / 'default.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='ROS namespace containing cmd_vel and RB-Y1 services.',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the UI ROS parameter YAML file.',
    )

    ui_node = Node(
        package='rby1_control_ui',
        executable='control_ui',
        name='rby1_control_ui',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        ui_node,
    ])
