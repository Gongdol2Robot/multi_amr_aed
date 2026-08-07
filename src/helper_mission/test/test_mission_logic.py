"""현장 구조 인력 Vision 판정 단위 테스트."""

import math

import pytest

from helper_mission.mission_logic import (
    helper_confirmation_is_fresh,
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


def test_vision_times_out_five_minutes_after_search_without_messages():
    """탐색 시작 후 Vision 메시지가 전혀 없으면 정확히 300초에 만료된다."""
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


def test_vision_timeout_uses_latest_true_or_false_message():
    """검출 여부와 무관하게 마지막 Vision 메시지부터 5분을 다시 계산한다."""
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
    """0 이하의 카메라 생존 타임아웃 설정은 구성 오류로 처리한다."""
    with pytest.raises(ValueError):
        vision_stream_timed_out(
            search_started_at=1.0,
            last_observed_at=None,
            now=1.0,
            timeout_seconds=0.0,
        )
