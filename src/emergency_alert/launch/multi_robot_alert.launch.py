"""요청된 모든 로봇에 출동 경보 실행기를 하나씩 기동한다."""

from emergency_alert.robot_ids import parse_robot_ids
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_robot_nodes(context):
    """robot_ids launch 인자를 읽어 namespace별 독립 노드를 생성한다."""
    raw_robot_ids = LaunchConfiguration("robot_ids").perform(context)
    robot_ids = parse_robot_ids(raw_robot_ids)

    return [
        Node(
            package="emergency_alert",
            executable="alert_mission_executor",
            namespace=robot_id,
            name="alert_mission_executor",
            output="screen",
            parameters=[{"robot_id": robot_id}],
        )
        for robot_id in robot_ids
    ]


def generate_launch_description():
    """임의 개수의 로봇 ID를 받는 공통 launch 구성을 반환한다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ids",
                default_value="robot1,robot2",
                description="Comma-separated robot IDs without duplicates",
            ),
            OpaqueFunction(function=_create_robot_nodes),
        ]
    )
