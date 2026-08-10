import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rby1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='physrobotics4',
    maintainer_email='physrobotics4@example.com',
    description='RBY1 simulation and real robot bringup package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
