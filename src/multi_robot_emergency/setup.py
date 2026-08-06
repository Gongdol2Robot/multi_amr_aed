from glob import glob
import os

from setuptools import find_packages, setup


package_name = "multi_robot_emergency"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="youngkim99@kakao.com",
    description="Central two-robot Nav2 path comparison and dispatch.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "emergency_mission_manager = "
            "multi_robot_emergency.mission_manager:main",
        ],
    },
)
