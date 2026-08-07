"""Bring up localization (map_server + AMCL) and RViz together.

Equivalent to running `loc <n>` and `rv <n>` in two separate terminals,
combined into a single command so the map/robot show up without extra steps.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    map_yaml = LaunchConfiguration("map")

    default_map = PathJoinSubstitution(
        [EnvironmentVariable("AED_WS", default_value="."), "maps", "map.yaml"]
    )

    localization_launch = PathJoinSubstitution(
        [FindPackageShare("turtlebot4_navigation"), "launch", "localization.launch.py"]
    )
    view_robot_launch = PathJoinSubstitution(
        [FindPackageShare("turtlebot4_viz"), "launch", "view_robot.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="/robot1", description="e.g. /robot1"
            ),
            DeclareLaunchArgument(
                "map", default_value=default_map, description="map.yaml path"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([localization_launch]),
                launch_arguments={"namespace": namespace, "map": map_yaml}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([view_robot_launch]),
                launch_arguments={"namespace": namespace}.items(),
            ),
        ]
    )
