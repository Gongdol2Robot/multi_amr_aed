"""Launch the LiDAR watchdog node with an overridable parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_params_file = os.path.join(
        get_package_share_directory("sensor_recovery"),
        "config",
        "lidar_watchdog.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_timeout_sec = LaunchConfiguration("scan_timeout_sec")
    recovery_duration_sec = LaunchConfiguration("recovery_duration_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="YAML file with robot_names/scan_topic_suffix and thresholds",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "scan_timeout_sec",
                default_value="5.0",
                description=(
                    "Seconds without /scan before a robot is FAULT; the "
                    "5-second default tolerates discovery-server jitter"
                ),
            ),
            DeclareLaunchArgument(
                "recovery_duration_sec",
                default_value="3.0",
                description="Seconds of continuous reception required to leave RECOVERING",
            ),
            Node(
                package="sensor_recovery",
                executable="lidar_watchdog",
                name="lidar_watchdog_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "scan_timeout_sec": ParameterValue(
                            scan_timeout_sec, value_type=float
                        ),
                        "recovery_duration_sec": ParameterValue(
                            recovery_duration_sec, value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
