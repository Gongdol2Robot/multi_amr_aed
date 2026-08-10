"""현장 구조 인력 Vision 판정 단위 테스트."""

import math

import pytest

from helper_mission.mission_logic import (
    arrival_dispatch_allowed,
    dispatch_response_is_current,
    helper_confirmation_is_fresh,
    is_aed_delivery_arrival,
    vision_stream_timed_out,
)


def test_accepts_recent_confirmed_helper():
    """최근 true 관측은 회전과 호출음을 멈추는 조건으로 인정한다."""
    assert helper_confirmation_is_fresh(
        confirmed=True, observed_at=10.0, now=10.5, stale_seconds=1.0
    )


def test_rejects_false_missing_future_or_stale_observation():
    """false·미수신·미래 시각·만료 관측은 구조 인력 감지로 인정하지 않는다."""
    cases = [
        (False, 10.0, 10.1),
        (True, None, 10.1),
        (True, 11.0, 10.0),
        (True, 8.9, 10.0),
        (True, math.inf, 10.0),
    ]
    for confirmed, observed_at, now in cases:
        assert not helper_confirmation_is_fresh(
            confirmed=confirmed,
            observed_at=observed_at,
            now=now,
            stale_seconds=1.0,
        )


def test_rejects_invalid_stale_duration():
    """0 이하의 Vision 유효 시간 설정은 구성 오류로 처리한다."""
    with pytest.raises(ValueError):
        helper_confirmation_is_fresh(
            confirmed=True,
            observed_at=1.0,
            now=1.0,
            stale_seconds=0.0,
        )


def test_duplicate_arrival_does_not_create_another_dispatch():
    """pending 또는 active 이벤트의 중복 ARRIVED는 새 Goal을 만들지 않는다."""
    assert arrival_dispatch_allowed(
        "event-1",
        canceled_events=set(),
        handled_events=set(),
        pending_events=set(),
        active_events=set(),
    )
    assert not arrival_dispatch_allowed(
        "event-1",
        canceled_events=set(),
        handled_events=set(),
        pending_events={"event-1"},
        active_events=set(),
    )
    assert not arrival_dispatch_allowed(
        "event-1",
        canceled_events=set(),
        handled_events=set(),
        pending_events=set(),
        active_events={"event-1"},
    )
    assert not arrival_dispatch_allowed(
        "event-1",
        canceled_events=set(),
        handled_events={"event-1"},
        pending_events=set(),
        active_events=set(),
    )


def test_only_aed_delivery_arrival_starts_helper_scan():
    """재할당 복귀 ARRIVED는 무시하고 실제 환자 도착만 인정한다."""
    assert is_aed_delivery_arrival(
        "event-1-aed-robot1", "robot1"
    )
    assert is_aed_delivery_arrival(
        "event-1-live-aed-robot2", "robot2"
    )
    assert not is_aed_delivery_arrival(
        "event-1-live-return-robot1", "robot1"
    )
    assert not is_aed_delivery_arrival(
        "event-1-helper-return-robot2", "robot2"
    )
    assert not is_aed_delivery_arrival(
        "event-1-return-robot1", "robot1"
    )
    assert not is_aed_delivery_arrival(
        "event-1-live-aed-robot2", "robot1"
    )


def test_late_action_acceptance_is_rejected_after_cancel_or_new_serial():
    """취소 뒤 또는 이전 세대에서 늦게 온 Action 수락 응답을 거부한다."""
    assert dispatch_response_is_current(
        event_exists=True,
        canceled=False,
        context_serial=4,
        response_serial=4,
        dispatching=True,
    )
    assert not dispatch_response_is_current(
        event_exists=False,
        canceled=True,
        context_serial=None,
        response_serial=4,
        dispatching=False,
    )
    assert not dispatch_response_is_current(
        event_exists=True,
        canceled=False,
        context_serial=5,
        response_serial=4,
        dispatching=True,
    )


def test_vision_times_out_after_five_minutes_without_messages():
    """Vision 메시지가 없으면 탐색 시작 또는 마지막 수신 300초 뒤 만료된다."""
    assert not vision_stream_timed_out(
        search_started_at=100.0,
        last_observed_at=None,
        now=399.999,
        timeout_seconds=300.0,
    )
    assert vision_stream_timed_out(
        search_started_at=100.0,
        last_observed_at=None,
        now=400.0,
        timeout_seconds=300.0,
    )
    assert not vision_stream_timed_out(
        search_started_at=100.0,
        last_observed_at=350.0,
        now=649.0,
        timeout_seconds=300.0,
    )
    assert vision_stream_timed_out(
        search_started_at=100.0,
        last_observed_at=350.0,
        now=650.0,
        timeout_seconds=300.0,
    )


def test_vision_timeout_rejects_invalid_duration():
    """0 이하의 카메라 생존 제한 시간은 구성 오류로 처리한다."""
    with pytest.raises(ValueError):
        vision_stream_timed_out(
            search_started_at=1.0,
            last_observed_at=None,
            now=1.0,
            timeout_seconds=0.0,
        )
