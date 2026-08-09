"""Nav2 제어 없이 MissionStatus 기반 경보 노드만 로봇별로 실행한다."""

from emergency_alert.robot_ids import parse_audio_devices, parse_robot_ids
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_robot_nodes(context):
    """robot_ids 인자를 검증하고 namespace별 상태 구독형 경보 노드를 만든다."""
    robot_ids = parse_robot_ids(
        LaunchConfiguration("robot_ids").perform(context)
    )
    audio_devices = parse_audio_devices(
        LaunchConfiguration("audio_devices").perform(context), robot_ids
    )
    shared_audio = {
        "audio_backend": LaunchConfiguration("audio_backend").perform(context),
        "audio_player": LaunchConfiguration("audio_player").perform(context),
    }
    return [
        Node(
            package="emergency_alert",
            executable="mission_status_alert",
            namespace=robot_id,
            name="mission_status_alert",
            output="screen",
            parameters=[
                shared_audio,
                {
                    "robot_id": robot_id,
                    "audio_device": audio_devices[robot_id],
                },
            ],
        )
        for robot_id in robot_ids
    ]


def generate_launch_description():
    """중복 Nav2 Goal이 없는 권장 실기 경보 launch 구성을 반환한다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ids",
                default_value="robot1,robot2",
                description="Comma-separated robot IDs without duplicates",
            ),
            DeclareLaunchArgument("audio_backend", default_value="system"),
            DeclareLaunchArgument("audio_player", default_value="auto"),
            DeclareLaunchArgument(
                "audio_devices",
                default_value="",
                description=(
                    "Comma-separated output device per robot in robot_ids "
                    "order. Empty uses the OS default output for all robots."
                ),
            ),
            OpaqueFunction(function=_create_robot_nodes),
        ]
    )
