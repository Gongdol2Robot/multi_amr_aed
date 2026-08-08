from glob import glob

from setuptools import find_packages, setup


package_name = "aed_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="multi_amr_aed team",
    maintainer_email="team@example.com",
    description="Camera input and map-coordinate utilities for AED detection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision_detector = aed_vision.vision_detector:main",
            "webcam_publisher = aed_vision.webcam_publisher:main",
        ],
    },
)
