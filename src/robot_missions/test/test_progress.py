import math
from types import SimpleNamespace

import pytest

from robot_missions.mission_executor import (
    MissionExecutor,
    angle_distance,
    fallback_takes_ownership,
    lidar_blocks_nav2,
    pose_has_progress,
    position_within_radius,
)


def test_rotation_in_place_counts_as_progress() -> None:
    assert pose_has_progress(
        (1.0, 2.0, 0.0),
        (1.0, 2.0, math.radians(10.0)),
        translation_epsilon=0.03,
        rotation_epsilon=math.radians(8.0),
    )


def test_small_pose_noise_does_not_count_as_progress() -> None:
    assert not pose_has_progress(
        (1.0, 2.0, 0.0),
        (1.01, 2.01, math.radians(2.0)),
        translation_epsilon=0.03,
        rotation_epsilon=math.radians(8.0),
    )


def test_translation_counts_as_progress() -> None:
    assert pose_has_progress(
        (1.0, 2.0, 0.0),
        (1.04, 2.0, 0.0),
        translation_epsilon=0.03,
        rotation_epsilon=math.radians(8.0),
    )


def test_angle_distance_wraps_at_pi() -> None:
    assert angle_distance(
        math.radians(179.0), math.radians(-179.0)
    ) == pytest.approx(math.radians(2.0))


def test_position_inside_return_radius_is_arrived() -> None:
    assert position_within_radius((0.3, 0.4), (0.0, 0.0), 0.5)


def test_position_outside_return_radius_is_not_arrived() -> None:
    assert not position_within_radius((0.31, 0.4), (0.0, 0.0), 0.5)


@pytest.mark.parametrize(
    "state",
    ["STARTING", "ACTIVE", "BLOCKED", "RECOVERING"],
)
def test_fallback_motion_states_own_cmd_vel(state: str) -> None:
    assert fallback_takes_ownership(state)


@pytest.mark.parametrize("state", ["IDLE", "RESUMED", "SUCCEEDED", "FAILED"])
def test_terminal_fallback_states_release_cmd_vel(state: str) -> None:
    assert not fallback_takes_ownership(state)


def test_lidar_fault_or_unready_recovery_holds_nav2() -> None:
    assert lidar_blocks_nav2("FAULT", True)
    assert lidar_blocks_nav2("ALIVE", False)
    assert not lidar_blocks_nav2("ALIVE", True)


def test_recovery_update_does_not_resubmit_pending_goal() -> None:
    sent_goals = []
    executor = SimpleNamespace(
        assignment=SimpleNamespace(target=object()),
        fallback_terminal_reported=False,
        goal_handle=None,
        goal_request_pending=True,
        pending_pose=object(),
        fallback_resume_requested=False,
        _goal_blocked_for_recovery=lambda: False,
        _send_goal=lambda pose, serial: sent_goals.append((pose, serial)),
    )

    MissionExecutor._maybe_resume_after_recovery(executor)

    assert sent_goals == []
