import os
from glob import glob

from setuptools import find_packages, setup


package_name = "helper_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="multi_amr_aed team",
    maintainer_email="team@example.com",
    description="On-site rotating helper detection and alert controller.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "helper_mission_controller = helper_mission.helper_mission_controller:main",
            "helper_mission_coordinator = helper_mission.helper_mission_coordinator:main",
        ],
    },
)
