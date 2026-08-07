"""Launch the central path manager and the two mission executors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start no Nav2 or RViz processes; those run on each robot PC."""
    crowd_config = PathJoinSubstitution(
        [
            FindPackageShare("multi_robot_emergency"),
            "config",
            "crowd_zones.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dispatch_enabled",
                default_value="false",
                choices=["true", "false"],
                description="Send assignments chosen by the ETA policy",
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
                "docked_start_offset_m", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "planning_timeout_sec", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "dispatch_retry_timeout_sec", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "assignment_ack_timeout_sec", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "dual_dispatch_enabled",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Dispatch both valid robots when the fastest ETA is "
                    "close to the target arrival time"
                ),
            ),
            DeclareLaunchArgument(
                "target_arrival_time_sec", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "dual_dispatch_trigger_ratio", default_value="0.85"
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
                    crowd_config,
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
                        "docked_start_offset_m": ParameterValue(
                            LaunchConfiguration("docked_start_offset_m"),
                            value_type=float,
                        ),
                        "planning_timeout_sec": ParameterValue(
                            LaunchConfiguration("planning_timeout_sec"),
                            value_type=float,
                        ),
                        "dispatch_retry_timeout_sec": ParameterValue(
                            LaunchConfiguration("dispatch_retry_timeout_sec"),
                            value_type=float,
                        ),
                        "assignment_ack_timeout_sec": ParameterValue(
                            LaunchConfiguration("assignment_ack_timeout_sec"),
                            value_type=float,
                        ),
                        "dual_dispatch_enabled": ParameterValue(
                            LaunchConfiguration("dual_dispatch_enabled"),
                            value_type=bool,
                        ),
                        "target_arrival_time_sec": ParameterValue(
                            LaunchConfiguration("target_arrival_time_sec"),
                            value_type=float,
                        ),
                        "dual_dispatch_trigger_ratio": ParameterValue(
                            LaunchConfiguration(
                                "dual_dispatch_trigger_ratio"
                            ),
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
            Node(
                package="robot_missions",
                executable="mission_executor",
                name="robot1_mission_executor",
                output="screen",
                parameters=[{
                    "robot_id": "robot1",
                    "assignment_topic": "/robot1/mission_assignment",
                    "navigate_action": "/robot1/navigate_to_pose",
                    "dispatch_retry_timeout_sec": ParameterValue(
                        LaunchConfiguration("dispatch_retry_timeout_sec"),
                        value_type=float,
                    ),
                }],
            ),
            Node(
                package="robot_missions",
                executable="mission_executor",
                name="robot2_mission_executor",
                output="screen",
                parameters=[{
                    "robot_id": "robot2",
                    "assignment_topic": "/robot2/mission_assignment",
                    "navigate_action": "/robot2/navigate_to_pose",
                    "dispatch_retry_timeout_sec": ParameterValue(
                        LaunchConfiguration("dispatch_retry_timeout_sec"),
                        value_type=float,
                    ),
                }],
            ),
        ]
    )
