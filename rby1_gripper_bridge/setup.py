from glob import glob

from setuptools import find_packages, setup


package_name = 'rby1_gripper_bridge'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RB-Y1 Developer',
    maintainer_email='user@example.com',
    description=(
        'ROS 2 bridge for RBY1 MuJoCo and physical dual grippers.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_bridge = rby1_gripper_bridge.bridge_node:main',
            'gripper_debug_controller = '
            'rby1_gripper_bridge.debug_controller:main',
        ],
    },
)
