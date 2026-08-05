from setuptools import find_packages, setup


package_name = "sensor_recovery"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="multi_amr_aed team",
    maintainer_email="team@example.com",
    description="LiDAR health monitoring and sensor recovery coordination.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_health_monitor = sensor_recovery.sensor_health_monitor:main",
        ],
    },
)
