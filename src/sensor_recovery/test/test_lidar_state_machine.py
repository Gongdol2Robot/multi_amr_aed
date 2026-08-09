from sensor_recovery.lidar_state_machine import (
    LidarMonitor,
    LidarState,
    LidarWatchdogConfig,
)


def _config(timeout=1.0, grace=3.0, recovery=3.0) -> LidarWatchdogConfig:
    return LidarWatchdogConfig(
        scan_timeout_sec=timeout,
        startup_grace_sec=grace,
        recovery_duration_sec=recovery,
    )


def test_starts_in_starting_state():
    monitor = LidarMonitor(config=_config(), start_time=0.0)
    assert monitor.state == LidarState.STARTING


def test_first_scan_moves_starting_to_alive():
    monitor = LidarMonitor(config=_config(), start_time=0.0)
    new_state = monitor.on_scan_received(now=0.5)
    assert new_state == LidarState.ALIVE
    assert monitor.state == LidarState.ALIVE


def test_no_scan_within_grace_stays_starting():
    monitor = LidarMonitor(config=_config(grace=3.0), start_time=0.0)
    assert monitor.on_tick(now=2.9) is None
    assert monitor.state == LidarState.STARTING


def test_no_scan_past_grace_moves_to_fault():
    monitor = LidarMonitor(config=_config(grace=3.0), start_time=0.0)
    new_state = monitor.on_tick(now=3.1)
    assert new_state == LidarState.FAULT
    assert monitor.state == LidarState.FAULT


def test_alive_short_delay_within_timeout_does_not_fault():
    monitor = LidarMonitor(config=_config(timeout=1.0), start_time=0.0)
    monitor.on_scan_received(now=0.0)
    assert monitor.on_tick(now=0.8) is None
    assert monitor.state == LidarState.ALIVE


def test_alive_past_timeout_moves_to_fault():
    monitor = LidarMonitor(config=_config(timeout=1.0), start_time=0.0)
    monitor.on_scan_received(now=0.0)
    new_state = monitor.on_tick(now=1.1)
    assert new_state == LidarState.FAULT
    assert monitor.state == LidarState.FAULT


def test_fault_repeated_ticks_do_not_repeat_transition():
    monitor = LidarMonitor(config=_config(timeout=1.0), start_time=0.0)
    monitor.on_scan_received(now=0.0)
    monitor.on_tick(now=1.1)
    assert monitor.state == LidarState.FAULT
    # A later tick while still in FAULT must not report a new transition.
    assert monitor.on_tick(now=5.0) is None
    assert monitor.state == LidarState.FAULT


def test_fault_scan_moves_to_recovering():
    monitor = LidarMonitor(config=_config(timeout=1.0), start_time=0.0)
    monitor.on_scan_received(now=0.0)
    monitor.on_tick(now=1.1)
    new_state = monitor.on_scan_received(now=1.2)
    assert new_state == LidarState.RECOVERING
    assert monitor.state == LidarState.RECOVERING


def test_recovering_confirmed_after_duration_moves_to_alive():
    monitor = LidarMonitor(
        config=_config(timeout=1.0, recovery=3.0), start_time=0.0
    )
    monitor.on_scan_received(now=0.0)
    monitor.on_tick(now=1.1)
    monitor.on_scan_received(now=1.2)
    # Continuous reception keeps scan age low throughout the recovery window.
    monitor.on_scan_received(now=2.5)
    monitor.on_scan_received(now=3.8)
    new_state = monitor.on_tick(now=4.3)
    assert new_state == LidarState.ALIVE
    assert monitor.state == LidarState.ALIVE


def test_recovering_interrupted_by_new_timeout_returns_to_fault():
    monitor = LidarMonitor(
        config=_config(timeout=1.0, recovery=3.0), start_time=0.0
    )
    monitor.on_scan_received(now=0.0)
    monitor.on_tick(now=1.1)
    monitor.on_scan_received(now=1.2)  # one frame only, then stops again
    new_state = monitor.on_tick(now=2.5)
    assert new_state == LidarState.FAULT
    assert monitor.state == LidarState.FAULT


def test_two_robots_are_independent():
    robot1 = LidarMonitor(config=_config(timeout=1.0), start_time=0.0)
    robot2 = LidarMonitor(config=_config(timeout=1.0), start_time=0.0)
    robot1.on_scan_received(now=0.0)
    robot2.on_scan_received(now=0.0)

    robot1.on_tick(now=1.1)  # only robot1 times out

    assert robot1.state == LidarState.FAULT
    assert robot2.state == LidarState.ALIVE
