"""중앙 노트북에서 사용하는 전체 AED 런타임을 한 번만 실행한다."""

import fcntl
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


_RUNTIME_LOCK = None


def _acquire_runtime_lock(_context):
    """같은 중앙 PC에서 통합 launch가 중복 실행되는 것을 막는다."""
    global _RUNTIME_LOCK
    lock_path = Path("/tmp/multi_amr_aed_server_runtime.lock")
    handle = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        handle.close()
        raise RuntimeError(
            "server_runtime.launch.py is already running on this computer"
        ) from error
    handle.seek(0)
    handle.truncate()
    handle.write(f"{os.getpid()}\n")
    handle.flush()
    _RUNTIME_LOCK = handle
    return []


def _include(package: str, launch_file: str, *, condition=None, arguments=None):
    """설치된 패키지의 launch 파일을 중복 경로 코드 없이 포함한다."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package), "launch", launch_file]
            )
        ),
        condition=condition,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description() -> LaunchDescription:
    """Nav2와 골목 카메라를 제외한 중앙 노트북 구성을 실행한다."""
    start_alert = LaunchConfiguration("start_alert")
    start_hmi = LaunchConfiguration("start_hmi")
    start_frontend = LaunchConfiguration("start_frontend")
    start_open_camera = LaunchConfiguration("start_open_camera")
    start_robot_vision = LaunchConfiguration("start_robot_vision")
    start_helper_mission = LaunchConfiguration("start_helper_mission")

    alert = _include(
        "emergency_alert",
        "multi_robot_status_alert.launch.py",
        condition=IfCondition(start_alert),
        arguments={"robot_ids": "robot1,robot2"},
    )
    hmi = _include(
        "aed_hmi",
        "hmi_runtime.launch.py",
        condition=IfCondition(start_hmi),
        arguments={
            "backend_host": LaunchConfiguration("backend_host"),
            "backend_port": LaunchConfiguration("backend_port"),
        },
    )
    open_camera = _include(
        "aed_vision",
        "camera_vision.launch.py",
        condition=IfCondition(start_open_camera),
        arguments={
            "camera": "1",
            "target": LaunchConfiguration("open_camera_target"),
        },
    )
    central = _include(
        "multi_robot_emergency",
        "central_dispatch.launch.py",
        arguments={
            "dispatch_enabled": LaunchConfiguration("dispatch_enabled"),
            "target_arrival_time_sec": LaunchConfiguration(
                "target_arrival_time_sec"
            ),
            "dual_dispatch_trigger_ratio": LaunchConfiguration(
                "dual_dispatch_trigger_ratio"
            ),
            "dual_dispatch_enabled": LaunchConfiguration(
                "dual_dispatch_enabled"
            ),
            "start_robot_vision": start_robot_vision,
            "robot_vision_target": LaunchConfiguration(
                "robot_vision_target"
            ),
            "start_helper_mission": start_helper_mission,
        },
    )

    frontend_dir = LaunchConfiguration("frontend_dir")
    frontend = ExecuteProcess(
        cmd=[
            FindExecutable(name="npm"),
            "run",
            "dev",
            "--",
            "--host",
            LaunchConfiguration("frontend_host"),
        ],
        cwd=frontend_dir,
        condition=IfCondition(start_frontend),
        output="screen",
    )

    return LaunchDescription(
        [
            OpaqueFunction(function=_acquire_runtime_lock),
            DeclareLaunchArgument(
                "dispatch_enabled",
                default_value="false",
                choices=("true", "false"),
                description="실제 로봇에 출동 목표를 전송할지 여부",
            ),
            DeclareLaunchArgument(
                "target_arrival_time_sec", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "dual_dispatch_trigger_ratio", default_value="0.85"
            ),
            DeclareLaunchArgument(
                "dual_dispatch_enabled",
                default_value="true",
                choices=("true", "false"),
            ),
            DeclareLaunchArgument(
                "start_alert", default_value="true",
                choices=("true", "false"),
            ),
            DeclareLaunchArgument(
                "start_hmi", default_value="true",
                choices=("true", "false"),
            ),
            DeclareLaunchArgument("backend_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("backend_port", default_value="8000"),
            DeclareLaunchArgument(
                "start_frontend", default_value="true",
                choices=("true", "false"),
            ),
            DeclareLaunchArgument("frontend_host", default_value="0.0.0.0"),
            DeclareLaunchArgument(
                "frontend_dir",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        "multi_amr_aed",
                        "src",
                        "aed_hmi",
                        "frontend",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "start_open_camera", default_value="true",
                choices=("true", "false"),
            ),
            DeclareLaunchArgument(
                "open_camera_target", default_value="person",
                choices=("person", "mannequin"),
            ),
            DeclareLaunchArgument(
                "start_robot_vision", default_value="true",
                choices=("true", "false"),
            ),
            DeclareLaunchArgument(
                "robot_vision_target", default_value="person",
                choices=("person", "mannequin"),
            ),
            DeclareLaunchArgument(
                "start_helper_mission", default_value="true",
                choices=("true", "false"),
            ),
            DeclareLaunchArgument(
                "central_start_delay_sec", default_value="2.0"
            ),
            alert,
            hmi,
            open_camera,
            frontend,
            # MissionStatus는 VOLATILE이라 경보 구독자가 먼저 준비되도록 한다.
            TimerAction(
                period=LaunchConfiguration("central_start_delay_sec"),
                actions=[central],
            ),
        ]
    )
