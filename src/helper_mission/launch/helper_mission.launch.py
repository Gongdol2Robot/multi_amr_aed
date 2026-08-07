"""중앙 coordinator와 로봇별 현장 회전 탐색 서버를 실행한다."""

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

    controller_parameters = {
        "rotation_speed_rps": float(
            LaunchConfiguration("rotation_speed_rps").perform(context)
        ),
        "helper_wait_timeout": float(
            LaunchConfiguration("helper_wait_timeout").perform(context)
        ),
    }
    nodes = [
        Node(
            package="helper_mission",
            executable="helper_mission_coordinator",
            name="helper_mission_coordinator",
            output="screen",
            parameters=[{"robot_ids": robot_ids}],
        )
    ]
    nodes.extend(
        Node(
            package="helper_mission",
            executable="helper_mission_controller",
            namespace=robot_id,
            name="helper_mission_controller",
            output="screen",
            parameters=[{"robot_id": robot_id}, controller_parameters],
        )
        for robot_id in robot_ids
    )
    return nodes


def generate_launch_description():
    """로봇 목록·회전 속도·대기 제한 시간을 받는 launch 구성을 반환한다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ids", default_value="robot1,robot2"),
            DeclareLaunchArgument("rotation_speed_rps", default_value="0.35"),
            # 0은 구조 인력이 감지될 때까지 무제한으로 회전·호출한다.
            DeclareLaunchArgument("helper_wait_timeout", default_value="0.0"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
