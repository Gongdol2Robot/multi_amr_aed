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
    # Robot1 calibration measured on the 2026-08-08 manual takeover run:
    # odom estimated displacement=(3.812,-0.561)m while AMCL measured
    # approximately (3.788,-0.291)m. Robot2 stays neutral until measured.
    odom_calibration = {
        "robot1": {
            "odom_translation_scale": 0.986,
            "odom_translation_heading_correction_deg": 4.0,
            "odom_yaw_delta_scale": 0.92,
        }
    }.get(robot_name, {})
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
            parameters=[
                fallback_params,
                {"debug_enabled": True, **odom_calibration},
            ],
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
