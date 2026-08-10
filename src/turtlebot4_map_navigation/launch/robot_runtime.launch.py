"""Launch one robot's map navigation and LiDAR fallback runtime."""

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
    """Keep Nav2 and recovery lifecycle in the same foreground launch."""
    robot_name = LaunchConfiguration("robot_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot1"),
            DeclareLaunchArgument(
                "rviz", default_value="true", choices=("true", "false")
            ),
            _include(
                "turtlebot4_map_navigation",
                "map_navigation.launch.py",
                {
                    "namespace": robot_name,
                    "rviz": LaunchConfiguration("rviz"),
                },
            ),
            _include(
                "sensor_recovery",
                "lidar_fallback.launch.py",
                {"robot_name": robot_name},
            ),
        ]
    )
