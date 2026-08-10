import glob

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
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="multi_amr_aed team",
    maintainer_email="team@example.com",
    description="LiDAR health monitoring and sensor recovery coordination.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_health_monitor = sensor_recovery.sensor_health_monitor:main",
            "lidar_watchdog = sensor_recovery.lidar_watchdog_node:main",
            "lidar_fallback_controller = sensor_recovery.fallback_path_follower:main",
            "fallback_route_test = sensor_recovery.fallback_route_test:main",
            "cmd_vel_distance_test = sensor_recovery.cmd_vel_distance_test:main",
            "cmd_vel_route_follower = sensor_recovery.cmd_vel_route_follower:main",
            "depth_distance_viewer = sensor_recovery.depth_distance_viewer:main",
            "lidar_replacement_request = sensor_recovery.replacement_request:main",
        ],
    },
)
