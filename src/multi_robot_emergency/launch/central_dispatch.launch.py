"""Launch only the central two-robot Nav2 path comparison manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start no Nav2 or RViz processes; those run on each robot PC."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dispatch_enabled",
                default_value="false",
                choices=["true", "false"],
                description="Send the goal to the robot with the shorter path",
            ),
            DeclareLaunchArgument(
                "pose_timeout_sec", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "allow_stale_pose",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "use_planner_start",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "planning_timeout_sec", default_value="30.0"
            ),
            DeclareLaunchArgument("planner_id", default_value="GridBased"),
            DeclareLaunchArgument(
                "automatic_request",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "automatic_request_delay_sec", default_value="5.0"
            ),
            DeclareLaunchArgument("emergency_x", default_value="1.2"),
            DeclareLaunchArgument("emergency_y", default_value="2.4"),
            DeclareLaunchArgument("emergency_yaw", default_value="0.0"),
            Node(
                package="multi_robot_emergency",
                executable="emergency_mission_manager",
                name="emergency_mission_manager",
                output="screen",
                parameters=[
                    {
                        "robot_ids": ["robot1", "robot2"],
                        "dispatch_enabled": ParameterValue(
                            LaunchConfiguration("dispatch_enabled"),
                            value_type=bool,
                        ),
                        "pose_timeout_sec": ParameterValue(
                            LaunchConfiguration("pose_timeout_sec"),
                            value_type=float,
                        ),
                        "allow_stale_pose": ParameterValue(
                            LaunchConfiguration("allow_stale_pose"),
                            value_type=bool,
                        ),
                        "use_planner_start": ParameterValue(
                            LaunchConfiguration("use_planner_start"),
                            value_type=bool,
                        ),
                        "planning_timeout_sec": ParameterValue(
                            LaunchConfiguration("planning_timeout_sec"),
                            value_type=float,
                        ),
                        "planner_id": LaunchConfiguration("planner_id"),
                        "automatic_request": ParameterValue(
                            LaunchConfiguration("automatic_request"),
                            value_type=bool,
                        ),
                        "automatic_request_delay_sec": ParameterValue(
                            LaunchConfiguration("automatic_request_delay_sec"),
                            value_type=float,
                        ),
                        "initial_target_x": ParameterValue(
                            LaunchConfiguration("emergency_x"),
                            value_type=float,
                        ),
                        "initial_target_y": ParameterValue(
                            LaunchConfiguration("emergency_y"),
                            value_type=float,
                        ),
                        "initial_target_yaw": ParameterValue(
                            LaunchConfiguration("emergency_yaw"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
