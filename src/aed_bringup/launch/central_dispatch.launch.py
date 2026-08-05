"""Launch central robot-state path evaluation and mission assignment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the central path-cost producer and mission manager."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("planner_id", default_value="GridBased"),
            DeclareLaunchArgument("pose_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("plan_retry_sec", default_value="3.0"),
            DeclareLaunchArgument(
                "path_collection_timeout_sec", default_value="10.0"
            ),
            Node(
                package="robot_state_monitor",
                executable="robot_state_monitor",
                name="robot_state_monitor",
                output="screen",
                parameters=[
                    {
                        "robot_ids": ["robot1", "robot2"],
                        "planner_id": LaunchConfiguration("planner_id"),
                        "pose_timeout_sec": ParameterValue(
                            LaunchConfiguration("pose_timeout_sec"),
                            value_type=float,
                        ),
                        "plan_retry_sec": ParameterValue(
                            LaunchConfiguration("plan_retry_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="mission_manager",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                parameters=[
                    {
                        "robot_ids": ["robot1", "robot2"],
                        "path_collection_timeout_sec": ParameterValue(
                            LaunchConfiguration("path_collection_timeout_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
