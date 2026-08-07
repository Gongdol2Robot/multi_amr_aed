"""Unit tests for helper mission decisions."""

from helper_mission.mission_logic import (
    Candidate,
    PresenceGate,
    select_helper_robot,
)


def candidate(robot_id, **changes):
    """개별 필드만 덮어쓸 수 있는 정상 후보 시험 데이터를 만든다."""
    values = {
        "robot_id": robot_id,
        "available": True,
        "network_ok": True,
        "localization_ok": True,
        "nav2_ok": True,
        "emergency_stop": False,
        "path_valid": True,
        "battery_percentage": 80.0,
        "path_cost": -1.0,
    }
    values.update(changes)
    return Candidate(**values)


def test_selects_reserve_robot_and_excludes_aed_robot():
    """AED 전달 로봇을 제외하고 남은 정상 로봇을 선택한다."""
    result = select_helper_robot(
        [
            candidate("robot1", path_cost=1.0),
            candidate("robot2", path_cost=4.0),
        ],
        excluded_robot_id="robot1",
    )
    assert result == "robot2"


def test_rejects_unsafe_or_low_battery_candidates():
    """Nav2 장애 또는 배터리 20% 미만인 로봇을 후보에서 제외한다."""
    result = select_helper_robot(
        [
            candidate("robot1", nav2_ok=False),
            candidate("robot2", battery_percentage=19.9),
        ],
        excluded_robot_id="",
    )
    assert result is None


def test_uses_path_cost_then_battery_then_robot_id():
    """경로비용과 배터리 및 ID의 고정 우선순위를 검증한다."""
    candidates = [
        candidate("robot3", path_cost=-1.0, battery_percentage=99.0),
        candidate("robot2", path_cost=3.0, battery_percentage=70.0),
        candidate("robot1", path_cost=3.0, battery_percentage=80.0),
    ]
    assert select_helper_robot(candidates, "") == "robot1"


def test_presence_gate_requires_continuous_fresh_observation():
    """도착 판정이 신선한 검출의 연속 유지 시간을 요구하는지 확인한다."""
    gate = PresenceGate(3, 1.0, 2.0, 0.6)
    gate.observe(
        helper_count=1, evidence_count=3, distance=0.8, observed_at=10.0
    )
    for observed_at in (10.5, 11.0, 11.5):
        gate.observe(
            helper_count=1,
            evidence_count=4,
            distance=0.7,
            observed_at=observed_at,
        )
    assert not gate.confirmed(11.9)
    gate.observe(
        helper_count=1, evidence_count=4, distance=0.7, observed_at=12.0
    )
    assert gate.confirmed(12.0)


def test_presence_gate_resets_on_loss_or_stale_sample():
    """구조 인력 소실이나 검출 만료 시 연속 판정이 초기화되는지 확인한다."""
    gate = PresenceGate(3, 1.0, 2.0, 0.5)
    gate.observe(
        helper_count=1, evidence_count=3, distance=0.8, observed_at=5.0
    )
    assert not gate.confirmed(5.6)

    gate.observe(
        helper_count=0, evidence_count=3, distance=0.8, observed_at=6.0
    )
    gate.observe(
        helper_count=1, evidence_count=3, distance=0.8, observed_at=7.0
    )
    assert not gate.confirmed(8.0)
