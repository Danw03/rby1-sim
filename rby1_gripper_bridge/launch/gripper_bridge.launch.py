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
    backend = LaunchConfiguration('backend')
    robot_address = LaunchConfiguration('robot_address')
    auto_home = LaunchConfiguration('auto_home')

    bridge = Node(
        package='rby1_gripper_bridge',
        executable='gripper_bridge',
        name='gripper_bridge',
        namespace=namespace,
        parameters=[
            config,
            {
                'backend': ParameterValue(backend, value_type=str),
                'robot_address': ParameterValue(
                    robot_address, value_type=str
                ),
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
            'backend',
            default_value='mujoco',
            description='Gripper backend: mujoco or dynamixel.',
        ),
        DeclareLaunchArgument(
            'robot_address',
            default_value='127.0.0.1:50051',
            description='RBY1 MuJoCo gRPC server address.',
        ),
        DeclareLaunchArgument(
            'auto_home',
            default_value='false',
            description=(
                'Run full-travel homing during Dynamixel backend startup.'
            ),
        ),
        bridge,
    ])
