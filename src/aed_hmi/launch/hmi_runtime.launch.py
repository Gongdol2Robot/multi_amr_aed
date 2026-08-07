"""Start the optional HMI ROS support and web backend.

This launch is intentionally separate from central_dispatch.launch.py so the
navigation and mission stack never depends on the operator screen.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start RobotState publishing and, optionally, the FastAPI backend."""
    database_path = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), ".ros", "aed_hmi.sqlite3"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_backend",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("backend_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("backend_port", default_value="8000"),
            DeclareLaunchArgument(
                "database_path", default_value=database_path
            ),
            DeclareLaunchArgument("planner_id", default_value="GridBased"),
            DeclareLaunchArgument("pose_timeout_sec", default_value="15.0"),
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
            Node(
                package="robot_state_monitor",
                executable="robot_state_monitor",
                name="hmi_robot_state_monitor",
                output="screen",
                parameters=[
                    {
                        "robot_ids": ["robot1", "robot2"],
                        "planner_id": LaunchConfiguration("planner_id"),
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
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    FindExecutable(name="ros2"),
                    "run",
                    "aed_hmi",
                    "hmi_backend",
                    "--db",
                    LaunchConfiguration("database_path"),
                    "--host",
                    LaunchConfiguration("backend_host"),
                    "--port",
                    LaunchConfiguration("backend_port"),
                ],
                condition=IfCondition(LaunchConfiguration("start_backend")),
                output="screen",
                # The ROS workspace intentionally disables user site packages
                # during colcon builds. The web dependencies are installed in
                # the user site, so enable it only for this subprocess.
                additional_env={"PYTHONNOUSERSITE": ""},
            ),
        ]
    )
