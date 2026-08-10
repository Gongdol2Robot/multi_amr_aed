"""로봇 수와 무관하게 공통 launch에서 사용할 ID 목록을 검증한다."""

import re


_ROS_NAME_TOKEN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def parse_robot_ids(raw_robot_ids: str):
    """
    쉼표로 구분한 로봇 ID를 검증하고 순서를 유지한 목록으로 변환한다.

    각 ID는 ROS namespace 한 단계로 바로 사용할 수 있어야 한다. 빈 항목이나
    중복 ID를 조용히 무시하면 일부 로봇의 출동 노드가 누락될 수 있으므로
    잘못된 설정은 시작 시점에 명확한 ``ValueError``로 처리한다.
    """
    if not isinstance(raw_robot_ids, str) or not raw_robot_ids.strip():
        raise ValueError("robot_ids must contain at least one robot ID")

    raw_items = raw_robot_ids.split(",")
    robot_ids = [item.strip().strip("/") for item in raw_items]
    if any(not robot_id for robot_id in robot_ids):
        raise ValueError("robot_ids must not contain an empty item")

    invalid = [
        robot_id
        for robot_id in robot_ids
        if _ROS_NAME_TOKEN.fullmatch(robot_id) is None
    ]
    if invalid:
        raise ValueError(
            "robot IDs must be valid ROS name tokens: "
            + ", ".join(invalid)
        )

    duplicates = []
    seen = set()
    for robot_id in robot_ids:
        if robot_id in seen and robot_id not in duplicates:
            duplicates.append(robot_id)
        seen.add(robot_id)
    if duplicates:
        raise ValueError(
            "robot_ids contains duplicates: " + ", ".join(duplicates)
        )
    return robot_ids


def parse_audio_devices(raw_audio_devices, robot_ids):
    """
    쉼표로 구분한 오디오 출력 장치를 ``robot_ids`` 순서대로 매핑한다.

    로봇마다 블루투스 스피커를 따로 붙이면 로봇별로 다른 sink 이름을 줘야
    한다. 빈 문자열이면 모든 로봇이 OS 기본 출력을 사용한다. 항목 수가
    로봇 수와 다르면 어느 로봇이 어느 스피커를 쓰는지 확정할 수 없으므로
    조용히 잘라내지 않고 시작 시점에 ``ValueError``로 처리한다.
    """
    if raw_audio_devices is None:
        raw_audio_devices = ""
    if not isinstance(raw_audio_devices, str):
        raise ValueError("audio_devices must be a comma-separated string")
    if not raw_audio_devices.strip():
        return {robot_id: "" for robot_id in robot_ids}

    devices = [item.strip() for item in raw_audio_devices.split(",")]
    if len(devices) != len(robot_ids):
        raise ValueError(
            "audio_devices must list exactly one device per robot: "
            f"{len(robot_ids)} robots but {len(devices)} devices"
        )
    return dict(zip(robot_ids, devices))
