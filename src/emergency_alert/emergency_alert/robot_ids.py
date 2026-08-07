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
