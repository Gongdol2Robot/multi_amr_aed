"""ROS 없이 실행하는 경보 패턴 및 상태 전이 단위시험."""

from math import inf, nan

import pytest

from emergency_alert.alert_logic import (
    AlertCommand,
    MissionAlertPolicy,
    MissionPhase,
    TonePattern,
)


def status(policy, phase, version=1, robot_id="robot1", event_id="event-1"):
    """반복되는 상태 입력 인자를 기본값과 함께 정책에 전달한다."""
    return policy.handle(
        robot_id=robot_id,
        event_id=event_id,
        assignment_version=version,
        phase=phase,
    )


def test_tone_pattern_validates_values_and_duration():
    """정상 음 패턴의 총 길이를 계산하고 잘못된 값은 거부한다."""
    pattern = TonePattern.from_values([1000, 440], 0.25)
    assert pattern.frequencies == (1000, 440)
    assert pattern.total_duration == 0.5
    with pytest.raises(ValueError):
        TonePattern.from_values([], 0.25)
    with pytest.raises(ValueError):
        TonePattern.from_values([440, 0], 0.25)
    with pytest.raises(ValueError):
        TonePattern.from_values([440], 0.0)
    with pytest.raises(ValueError):
        TonePattern.from_values([440], nan)
    with pytest.raises(ValueError):
        TonePattern.from_values([440], inf)


def test_en_route_starts_once_and_arrival_plays_once():
    """동일 EN_ROUTE와 ARRIVED 상태가 중복 경보를 만들지 않는지 확인한다."""
    policy = MissionAlertPolicy("robot1")
    assert status(policy, MissionPhase.ASSIGNED) is AlertCommand.IGNORE
    assert status(policy, MissionPhase.EN_ROUTE) is AlertCommand.START_TRAVEL
    assert status(policy, MissionPhase.EN_ROUTE) is AlertCommand.IGNORE
    assert status(policy, MissionPhase.ARRIVED) is AlertCommand.PLAY_ARRIVAL
    assert status(policy, MissionPhase.ARRIVED) is AlertCommand.IGNORE
    assert status(policy, MissionPhase.COMPLETED) is AlertCommand.IGNORE


def test_new_assignment_stops_previous_travel_alarm():
    """높은 version의 새 배정이 이전 이동 경보를 먼저 정지하는지 검증한다."""
    policy = MissionAlertPolicy("robot1")
    assert status(policy, MissionPhase.EN_ROUTE) is AlertCommand.START_TRAVEL
    assert status(policy, MissionPhase.ASSIGNED, version=2) is AlertCommand.STOP
    assert status(policy, MissionPhase.EN_ROUTE, version=2) is (
        AlertCommand.START_TRAVEL
    )


def test_new_en_route_version_recovers_when_assigned_status_was_missed():
    """새 배정 상태가 유실돼도 높은 version의 EN_ROUTE로 경보를 복구한다."""
    policy = MissionAlertPolicy("robot1")
    assert status(policy, MissionPhase.EN_ROUTE) is AlertCommand.START_TRAVEL
    assert status(policy, MissionPhase.EN_ROUTE, version=2) is (
        AlertCommand.START_TRAVEL
    )


def test_failure_plays_interrupted_and_stale_status_is_ignored():
    """실패음 출력 후 이전 version의 늦은 도착 상태를 무시하는지 검증한다."""
    policy = MissionAlertPolicy("robot1")
    status(policy, MissionPhase.EN_ROUTE, version=2)
    assert status(policy, MissionPhase.INTERRUPTED, version=2) is (
        AlertCommand.PLAY_INTERRUPTED
    )
    assert status(policy, MissionPhase.ARRIVED, version=1) is AlertCommand.IGNORE


def test_terminal_and_en_route_states_never_regress():
    """늦은 진행 상태가 경보를 다시 켜거나 이전 단계로 되돌리지 못하게 한다."""
    policy = MissionAlertPolicy("robot1")
    assert status(policy, MissionPhase.EN_ROUTE) is AlertCommand.START_TRAVEL
    assert status(policy, MissionPhase.DISPATCHING) is AlertCommand.IGNORE
    assert status(policy, MissionPhase.ARRIVED) is AlertCommand.PLAY_ARRIVAL
    assert status(policy, MissionPhase.EN_ROUTE) is AlertCommand.IGNORE


def test_foreign_robot_and_unrelated_status_are_ignored():
    """다른 로봇과 조력자 등 무관한 상태가 AED 경보에 영향을 주지 않는다."""
    policy = MissionAlertPolicy("robot1")
    assert status(
        policy, MissionPhase.EN_ROUTE, robot_id="robot2"
    ) is AlertCommand.IGNORE
    assert status(policy, MissionPhase.UNRELATED) is AlertCommand.IGNORE
