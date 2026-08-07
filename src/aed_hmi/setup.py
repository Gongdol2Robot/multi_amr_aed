import os
from glob import glob

from setuptools import find_packages, setup


package_name = "aed_hmi"

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
    description="Operator HMI bridge for Multi-AMR AED status and controls.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hmi_node = aed_hmi.hmi_node:main",
            "hmi_backend = backend.main:main",
        ],
    },
)
