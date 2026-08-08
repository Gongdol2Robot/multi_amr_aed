from sensor_recovery.fallback_state_machine import (
    FallbackState,
    FallbackTickInputs,
    next_fallback_state,
)


def _inputs(**overrides):
    base = dict(
        has_plan=True,
        has_anchor=True,
        odom_stale=False,
        depth_blocked=False,
        blocked_duration_sec=0.0,
        blocked_timeout_sec=5.0,
        stuck=False,
        path_deviation_m=0.0,
        max_path_deviation_m=0.7,
        arrived=False,
    )
    base.update(overrides)
    return FallbackTickInputs(**base)


def test_healthy_tick_moves_starting_to_active():
    result = next_fallback_state(FallbackState.STARTING, _inputs())
    assert result == FallbackState.ACTIVE


def test_missing_plan_fails():
    result = next_fallback_state(FallbackState.STARTING, _inputs(has_plan=False))
    assert result == FallbackState.FAILED


def test_missing_anchor_fails():
    result = next_fallback_state(FallbackState.STARTING, _inputs(has_anchor=False))
    assert result == FallbackState.FAILED


def test_odom_stale_stops_without_terminal_failure():
    result = next_fallback_state(FallbackState.ACTIVE, _inputs(odom_stale=True))
    assert result == FallbackState.BLOCKED


def test_fresh_odom_resumes_after_odom_block():
    result = next_fallback_state(FallbackState.BLOCKED, _inputs(odom_stale=False))
    assert result == FallbackState.ACTIVE


def test_stuck_fails():
    result = next_fallback_state(FallbackState.ACTIVE, _inputs(stuck=True))
    assert result == FallbackState.FAILED


def test_excess_path_deviation_fails():
    result = next_fallback_state(
        FallbackState.ACTIVE, _inputs(path_deviation_m=1.0, max_path_deviation_m=0.7)
    )
    assert result == FallbackState.FAILED


def test_depth_blocked_moves_to_blocked_not_failed():
    result = next_fallback_state(
        FallbackState.ACTIVE,
        _inputs(depth_blocked=True, blocked_duration_sec=1.0, blocked_timeout_sec=5.0),
    )
    assert result == FallbackState.BLOCKED


def test_blocked_past_timeout_fails():
    result = next_fallback_state(
        FallbackState.BLOCKED,
        _inputs(depth_blocked=True, blocked_duration_sec=6.0, blocked_timeout_sec=5.0),
    )
    assert result == FallbackState.FAILED


def test_blocked_clears_back_to_active_once_depth_is_clear_again():
    result = next_fallback_state(FallbackState.BLOCKED, _inputs(depth_blocked=False))
    assert result == FallbackState.ACTIVE


def test_arrival_succeeds():
    result = next_fallback_state(FallbackState.ACTIVE, _inputs(arrived=True))
    assert result == FallbackState.SUCCEEDED


def test_succeeded_is_terminal():
    result = next_fallback_state(FallbackState.SUCCEEDED, _inputs(has_plan=False))
    assert result == FallbackState.SUCCEEDED


def test_failed_is_terminal():
    result = next_fallback_state(FallbackState.FAILED, _inputs(arrived=True))
    assert result == FallbackState.FAILED
