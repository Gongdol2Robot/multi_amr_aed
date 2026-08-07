"""로봇 수 확장을 위한 ID 목록 검증 단위시험."""

import pytest

from emergency_alert.robot_ids import parse_robot_ids


def test_accepts_one_or_many_robot_ids():
    """로봇 수에 상관없이 입력 순서를 보존한다."""
    assert parse_robot_ids("robot1") == ["robot1"]
    assert parse_robot_ids("robot1, robot2,robot12") == [
        "robot1",
        "robot2",
        "robot12",
    ]


def test_normalizes_outer_namespace_slashes():
    """사용자가 붙인 namespace 바깥쪽 슬래시는 제거한다."""
    assert parse_robot_ids("/robot1/, /robot2") == ["robot1", "robot2"]


@pytest.mark.parametrize("raw_value", ["", " ", "robot1,,robot2"])
def test_rejects_empty_robot_ids(raw_value):
    """누락된 로봇 프로세스가 생기지 않도록 빈 ID를 거부한다."""
    with pytest.raises(ValueError):
        parse_robot_ids(raw_value)


@pytest.mark.parametrize(
    "raw_value",
    ["robot1,robot1", "robot-1", "robot/one", "1robot"],
)
def test_rejects_duplicate_or_invalid_robot_ids(raw_value):
    """중복 namespace와 ROS 이름으로 사용할 수 없는 ID를 거부한다."""
    with pytest.raises(ValueError):
        parse_robot_ids(raw_value)
