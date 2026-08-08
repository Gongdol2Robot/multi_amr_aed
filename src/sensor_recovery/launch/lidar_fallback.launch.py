"""Launch one robot's LiDAR watchdog and fallback controller together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    robot_name = LaunchConfiguration("robot_name").perform(context).strip("/")
    if not robot_name:
        raise ValueError("robot_name must not be empty")
    package_share = get_package_share_directory("sensor_recovery")
    watchdog_params = os.path.join(package_share, "config", "lidar_watchdog.yaml")
    fallback_params = os.path.join(package_share, "config", "lidar_fallback.yaml")
    return [
        Node(
            package="sensor_recovery",
            executable="lidar_watchdog",
            name=f"lidar_watchdog_{robot_name}",
            output="screen",
            parameters=[
                watchdog_params,
                {
                    "robot_names": [robot_name],
                    "scan_timeout_sec": 5.0,
                },
            ],
        ),
        Node(
            package="sensor_recovery",
            executable="lidar_fallback_controller",
            namespace=robot_name,
            name="lidar_fallback_controller",
            output="screen",
            parameters=[fallback_params, {"debug_enabled": True}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="robot1",
                description="Robot namespace without a leading slash",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
