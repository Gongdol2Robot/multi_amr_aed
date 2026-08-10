"""Launch Nav2 and LiDAR fallback after localization is already running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package: str, launch_file: str, arguments: dict):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package), "launch", launch_file]
            )
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description() -> LaunchDescription:
    """Start the normal Nav2 stack and its matching recovery controller."""
    robot_name = LaunchConfiguration("robot_name")
    default_params = PathJoinSubstitution(
        [FindPackageShare("aed_bringup"), "config", "nav2_aed.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot1"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            _include(
                "turtlebot4_navigation",
                "nav2.launch.py",
                {
                    "namespace": robot_name,
                    "params_file": LaunchConfiguration("params_file"),
                },
            ),
            _include(
                "sensor_recovery",
                "lidar_fallback.launch.py",
                {"robot_name": robot_name},
            ),
        ]
    )
