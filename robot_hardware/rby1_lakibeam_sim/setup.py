from glob import glob

from setuptools import find_packages, setup

package_name = "rby1_lakibeam_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Dual MuJoCo LiDAR simulation and LaserScan merger",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            (
                "dual_lidar_node = "
                "rby1_lakibeam_sim.dual_lidar_node:main"
            ),
            (
                "scan_merger_node = "
                "rby1_lakibeam_sim.scan_merger_node:main"
            ),
            (
                "mujoco_pose_bridge_node = "
                "rby1_lakibeam_sim.mujoco_pose_bridge_node:main"
            ),
        ],
    },
)

