from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('rby1_gripper_bridge'))
    default_config = str(package_share / 'config' / 'default.yaml')

    namespace = LaunchConfiguration('namespace')
    config = LaunchConfiguration('config')
    auto_home = LaunchConfiguration('auto_home')

    bridge = Node(
        package='rby1_gripper_bridge',
        executable='gripper_bridge',
        name='gripper_bridge',
        namespace=namespace,
        parameters=[
            config,
            {
                'auto_home': ParameterValue(auto_home, value_type=bool),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='rby1',
            description='Top-level namespace for the gripper interface.',
        ),
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to the gripper bridge parameter YAML.',
        ),
        DeclareLaunchArgument(
            'auto_home',
            default_value='false',
            description='Run full-travel homing during startup.',
        ),
        bridge,
    ])
