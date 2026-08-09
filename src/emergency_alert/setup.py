import os
from glob import glob

from setuptools import find_packages, setup


package_name = "emergency_alert"

setup(
    name=package_name,
    version="0.2.0",
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
        (
            os.path.join("share", package_name, "assets"),
            glob("assets/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="multi_amr_aed team",
    maintainer_email="team@example.com",
    description="Mission-aware emergency audio alerts for TurtleBot4.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "alert_mission_executor = "
            "emergency_alert.alert_mission_executor:main",
            "mission_status_alert = "
            "emergency_alert.mission_status_alert:main",
            "siren = emergency_alert.siren_node:main",
        ],
    },
)
