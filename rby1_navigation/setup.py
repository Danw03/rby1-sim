import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rby1_navigation"


def collect_directory_files(source_directory):
    """하위 폴더 구조를 유지하면서 모든 파일을 data_files에 추가."""
    collected = []

    if not os.path.isdir(source_directory):
        return collected

    for root, _, files in os.walk(source_directory):
        if not files:
            continue

        install_directory = os.path.join(
            "share",
            package_name,
            root,
        )

        source_files = [
            os.path.join(root, filename)
            for filename in files
        ]

        collected.append(
            (install_directory, source_files)
        )

    return collected


data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    (
        os.path.join("share", package_name),
        ["package.xml"],
    ),
    (
        os.path.join("share", package_name, "launch"),
        glob("launch/*.launch.py"),
    ),
    (
        os.path.join("share", package_name, "config"),
        glob("config/*.yaml"),
    ),
]

data_files += collect_directory_files("maps")
data_files += collect_directory_files("rviz")


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="physrobotics4",
    maintainer_email="physrobotics4@example.com",
    description="SLAM Toolbox and Nav2 integration for RBY1",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ab_navigation_demo = "
            "rby1_navigation.ab_navigation_demo_node:main",
        ],
    },
)
