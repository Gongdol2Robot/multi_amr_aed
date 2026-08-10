"""중앙 coordinator와 로봇별 현장 회전 탐색 서버를 실행한다."""

from emergency_alert.robot_ids import parse_audio_devices
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
        "vision_timeout_seconds": float(
            LaunchConfiguration("vision_timeout_seconds").perform(context)
        ),
        "vision_stale_seconds": float(
            LaunchConfiguration("vision_stale_seconds").perform(context)
        ),
        "audio_backend": LaunchConfiguration("audio_backend").perform(context),
        "audio_player": LaunchConfiguration("audio_player").perform(context),
        "handoff_wait_seconds": float(
            LaunchConfiguration("handoff_wait_seconds").perform(context)
        ),
    }
    # audio_devices가 비면 모든 로봇이 기존 audio_device 하나를 그대로 쓴다.
    shared_audio_device = LaunchConfiguration("audio_device").perform(context)
    audio_devices = parse_audio_devices(
        LaunchConfiguration("audio_devices").perform(context), robot_ids
    )
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
            parameters=[
                controller_parameters,
                {
                    "robot_id": robot_id,
                    "audio_device": (
                        audio_devices[robot_id] or shared_audio_device
                    ),
                },
            ],
        )
        for robot_id in robot_ids
    )
    return nodes


def generate_launch_description():
    """로봇 목록·회전 속도·대기 제한 시간을 받는 launch 구성을 반환한다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ids", default_value="robot1,robot2"),
            DeclareLaunchArgument("rotation_speed_rps", default_value="0.12"),
            DeclareLaunchArgument("audio_backend", default_value="system"),
            DeclareLaunchArgument("audio_player", default_value="auto"),
            DeclareLaunchArgument("audio_device", default_value=""),
            DeclareLaunchArgument(
                "audio_devices",
                default_value="",
                description=(
                    "Comma-separated output device per robot in robot_ids "
                    "order. Overrides audio_device when set."
                ),
            ),
            DeclareLaunchArgument("handoff_wait_seconds", default_value="5.0"),
            DeclareLaunchArgument(
                "vision_stale_seconds", default_value="2.5"
            ),
            # 0은 구조 인력이 감지될 때까지 무제한으로 회전·호출한다.
            DeclareLaunchArgument("helper_wait_timeout", default_value="0.0"),
            # Vision 토픽이 5분간 끊기면 회전과 호출음을 안전 정지한다.
            DeclareLaunchArgument(
                "vision_timeout_seconds", default_value="300.0"
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
