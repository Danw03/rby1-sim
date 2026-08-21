import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 방금 만든 apriltag.yaml 파일의 위치를 찾습니다.
    config_file = os.path.join(
        get_package_share_directory('rby1_camera'),
        'config',
        'apriltag.yaml'
    )

    # 시스템에 깔려있는 공식 apriltag_ros 패키지를 호출합니다.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[config_file],
        remappings=[
            # 리얼센스 카메라의 토픽 이름을 AprilTag 노드에 맞게 매핑(연결)
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info')
        ],
        output='screen'
    )

    return LaunchDescription([
        apriltag_node
    ])