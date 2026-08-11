"""중앙 노트북에서 사용하는 전체 AED 런타임을 한 번만 실행한다."""

import fcntl
import os
from pathlib import Path
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


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


def _stop_all_local_processes(context):
    """launch 종료 이벤트 안에서 AED 프로세스 정리가 끝날 때까지 기다린다."""
    cleanup_path = context.perform_substitution(
        PathJoinSubstitution(
            [
                FindPackagePrefix("aed_bringup"),
                "lib",
                "aed_bringup",
                "stop_aed_processes.sh",
            ]
        )
    )
    cleanup_environment = os.environ.copy()
    cleanup_environment.pop("AED_WS", None)
    completed = subprocess.run(
        [cleanup_path], check=False, env=cleanup_environment
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "AED process cleanup failed with exit code "
            f"{completed.returncode}"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    """로컬 비전 검출기를 제외한 중앙 노트북 구성을 실행한다."""
    start_alert = LaunchConfiguration("start_alert")
    start_hmi = LaunchConfiguration("start_hmi")
    start_frontend = LaunchConfiguration("start_frontend")
    start_helper_mission = LaunchConfiguration("start_helper_mission")

    alert = _include(
        "emergency_alert",
        "multi_robot_status_alert.launch.py",
        condition=IfCondition(start_alert),
        arguments={
            "robot_ids": "robot1,robot2",
            "audio_devices": LaunchConfiguration("audio_devices"),
        },
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
            "start_helper_mission": start_helper_mission,
            "audio_devices": LaunchConfiguration("audio_devices"),
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
    shutdown_cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=_stop_all_local_processes)
            ]
        )
    )

    return LaunchDescription(
        [
            OpaqueFunction(function=_acquire_runtime_lock),
            shutdown_cleanup,
            DeclareLaunchArgument(
                "dispatch_enabled",
                default_value="true",
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
                "audio_devices",
                default_value="",
                description=(
                    "로봇별 오디오 출력 장치를 robot1,robot2 순서로 나열한다. "
                    "블루투스 스피커를 로봇마다 따로 붙일 때 사용하며, "
                    "비우면 두 로봇 모두 OS 기본 출력으로 재생한다."
                ),
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
                        EnvironmentVariable(
                            "AED_WS",
                            default_value=PathJoinSubstitution(
                                [
                                    EnvironmentVariable("HOME"),
                                    "rokey_ws",
                                    "multi_amr_aed",
                                ]
                            ),
                        ),
                        "src",
                        "aed_hmi",
                        "frontend",
                    ]
                ),
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
            frontend,
            # MissionStatus는 VOLATILE이라 경보 구독자가 먼저 준비되도록 한다.
            TimerAction(
                period=LaunchConfiguration("central_start_delay_sec"),
                actions=[central],
            ),
        ]
    )
