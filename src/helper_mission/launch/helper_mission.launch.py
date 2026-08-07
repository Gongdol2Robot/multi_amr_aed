"""Launch one coordinator and one GuideHelper server per robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    """Launch 인자를 해석해 중앙 coordinator와 로봇별 controller를 생성한다."""
    raw_robot_ids = LaunchConfiguration("robot_ids").perform(context)
    robot_ids = [
        value.strip().strip("/") for value in raw_robot_ids.split(",")
    ]
    if not robot_ids or any(not value for value in robot_ids):
        raise ValueError("robot_ids must be a comma-separated non-empty list")
    if len(set(robot_ids)) != len(robot_ids):
        raise ValueError("robot_ids must not contain duplicates")

    station = {
        "helper_station_frame": LaunchConfiguration(
            "helper_station_frame"
        ).perform(context),
        "helper_station_x": float(
            LaunchConfiguration("helper_station_x").perform(context)
        ),
        "helper_station_y": float(
            LaunchConfiguration("helper_station_y").perform(context)
        ),
        "helper_station_yaw": float(
            LaunchConfiguration("helper_station_yaw").perform(context)
        ),
    }
    nodes = [
        Node(
            package="helper_mission",
            executable="helper_mission_coordinator",
            name="helper_mission_coordinator",
            output="screen",
            parameters=[{"robot_ids": robot_ids}, station],
        )
    ]
    nodes.extend(
        Node(
            package="helper_mission",
            executable="helper_mission_controller",
            namespace=robot_id,
            name="helper_mission_controller",
            output="screen",
            parameters=[{"robot_id": robot_id}],
        )
        for robot_id in robot_ids
    )
    return nodes


def generate_launch_description():
    """로봇 목록과 구조 인력 대기 좌표를 받는 launch 구성을 반환한다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ids", default_value="robot1,robot2"),
            DeclareLaunchArgument("helper_station_frame", default_value=""),
            DeclareLaunchArgument("helper_station_x", default_value="0.0"),
            DeclareLaunchArgument("helper_station_y", default_value="0.0"),
            DeclareLaunchArgument("helper_station_yaw", default_value="0.0"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
