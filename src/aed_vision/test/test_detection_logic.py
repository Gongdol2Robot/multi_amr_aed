"""검출 후처리와 시간 확정 로직의 단위 테스트."""

import pytest

from aed_vision.detection_logic import (
    Box,
    CrowdStateStabilizer,
    FallenStateConfirmation,
    StationaryFallConfirmation,
    TemporalConfirmation,
    crowd_metrics,
    filter_nonfallen_people,
    intersection_over_union,
    intersection_over_smaller_area,
    point_inside_normalized_roi,
    update_presence_confirmation,
    is_fallen_bbox_candidate,
)


def test_temporal_confirmation_uses_recent_window() -> None:
    confirmation = TemporalConfirmation(4, 3)

    assert confirmation.update(True) is False
    assert confirmation.update(False) is False
    assert confirmation.update(True) is False
    assert confirmation.update(True) is True
    assert confirmation.hit_count == 3
    assert confirmation.update(False) is False


def test_fallen_state_uses_frame_confirmation_and_timed_clear() -> None:
    confirmation = FallenStateConfirmation(3, 3, 2.0)

    assert not confirmation.update(True, now=0.0)
    assert not confirmation.update(True, now=0.1)
    assert confirmation.update(True, now=0.2)
    assert confirmation.update(False, now=1.0)
    assert confirmation.update(True, now=2.0)
    assert confirmation.update(False, now=3.0)
    assert confirmation.update(False, now=4.9)
    assert not confirmation.update(False, now=5.0)
    assert confirmation.hit_count == 0
    assert not confirmation.update(True, now=5.1)


def test_fallen_state_rejects_invalid_clear_timeout() -> None:
    with pytest.raises(ValueError, match="clear_after_seconds"):
        FallenStateConfirmation(3, 3, 0.0)


def _stationary_confirmation() -> StationaryFallConfirmation:
    return StationaryFallConfirmation(1.0, 0.025, 0.25, 0.3, 0.25, 2.0)


def test_stationary_fall_confirms_same_stable_box_after_one_second() -> None:
    confirmation = _stationary_confirmation()
    box = Box(20, 20, 100, 80, 0.9)

    for now in (0.0, 0.2, 0.4, 0.6, 0.8):
        assert not confirmation.update([box], (640, 480), now=now)
    assert confirmation.update([box], (640, 480), now=1.0)
    assert confirmation.stationary_duration == pytest.approx(1.0)
    assert confirmation.hit_count == 6


def test_stationary_fall_resets_timer_when_box_moves_or_resizes() -> None:
    confirmation = _stationary_confirmation()
    original = Box(20, 20, 100, 80, 0.9)
    moved = Box(50, 20, 130, 80, 0.9)
    resized = Box(50, 20, 150, 90, 0.9)

    assert not confirmation.update([original], (640, 480), now=0.0)
    assert not confirmation.update([original], (640, 480), now=0.2)
    assert not confirmation.update([moved], (640, 480), now=0.4)
    assert confirmation.stationary_duration == 0.0
    assert not confirmation.update([moved], (640, 480), now=0.6)
    assert not confirmation.update([resized], (640, 480), now=0.8)
    assert confirmation.stationary_duration == 0.0


def test_stationary_fall_measures_motion_from_stationary_anchor() -> None:
    confirmation = _stationary_confirmation()

    for index, now in enumerate((0.0, 0.2, 0.4, 0.6, 0.8, 1.0)):
        # 프레임 간 이동은 작지만 최초 위치로부터는 계속 멀어지는 bbox다.
        box = Box(20 + index * 6, 20, 100 + index * 6, 80, 0.9)
        assert not confirmation.update([box], (640, 480), now=now)

    assert confirmation.stationary_duration < 1.0


def test_stationary_fall_tolerates_short_gap_and_clears_after_timeout() -> None:
    confirmation = _stationary_confirmation()
    box = Box(20, 20, 100, 80, 0.9)

    for now in (0.0, 0.2, 0.4, 0.6, 0.8):
        assert not confirmation.update([box], (640, 480), now=now)
    assert not confirmation.update([], (640, 480), now=0.9)
    assert confirmation.update([box], (640, 480), now=1.0)
    assert confirmation.update([], (640, 480), now=1.2)
    assert confirmation.update([], (640, 480), now=3.1)
    assert not confirmation.update([], (640, 480), now=3.2)


def test_stationary_fall_clear_resets_confirmed_state_for_new_mission() -> None:
    confirmation = _stationary_confirmation()
    box = Box(20, 20, 100, 80, 0.9)

    for now in (0.0, 0.2, 0.4, 0.6, 0.8):
        assert not confirmation.update([box], (640, 480), now=now)
    assert confirmation.update([box], (640, 480), now=1.0)

    confirmation.clear()

    assert not confirmation.update([], (640, 480), now=1.1)
    assert confirmation.hit_count == 0


def test_stationary_fall_rejects_invalid_parameters_and_frame_size() -> None:
    with pytest.raises(ValueError, match="positive"):
        StationaryFallConfirmation(0.0, 0.025, 0.25, 0.3, 0.25, 2.0)
    confirmation = _stationary_confirmation()
    with pytest.raises(ValueError, match="frame_size"):
        confirmation.update([], (0, 480), now=0.0)


def test_horizontal_bbox_is_kept_as_fallen_candidate() -> None:
    assert is_fallen_bbox_candidate(Box(0, 0, 103, 100), 1.03)
    assert not is_fallen_bbox_candidate(Box(0, 0, 102, 100), 1.03)


def test_bbox_fallback_rejects_invalid_threshold() -> None:
    with pytest.raises(ValueError, match="aspect_threshold"):
        is_fallen_bbox_candidate(Box(0, 0, 100, 100), 0.0)


@pytest.mark.parametrize("window,hits", [(0, 1), (2, 0), (2, 3)])
def test_temporal_confirmation_rejects_invalid_ranges(
    window: int, hits: int
) -> None:
    with pytest.raises(ValueError):
        TemporalConfirmation(window, hits)


def test_presence_confirmation_requires_current_detection() -> None:
    confirmation = TemporalConfirmation(3, 2)

    assert update_presence_confirmation(confirmation, True) is False
    assert update_presence_confirmation(confirmation, True) is True
    assert update_presence_confirmation(confirmation, False) is False


def test_iou_handles_overlap_and_degenerate_boxes() -> None:
    overlap = intersection_over_union(
        Box(0, 0, 10, 10), Box(5, 0, 15, 10)
    )
    assert overlap == pytest.approx(1 / 3)
    assert intersection_over_union(Box(0, 0, 0, 0), Box(0, 0, 0, 0)) == 0.0
    assert intersection_over_smaller_area(
        Box(0, 0, 10, 10), Box(-5, -5, 15, 15)
    ) == 1.0


def test_roi_and_fallen_person_filtering() -> None:
    fallen = Box(20, 20, 80, 80)
    duplicate = Box(20, 20, 80, 80)
    valid = Box(120, 20, 180, 80)
    outside = Box(220, 20, 280, 80)

    assert point_inside_normalized_roi((100, 50), (200, 100), [0, 0, 1, 1])
    assert filter_nonfallen_people(
        [duplicate, valid, outside],
        [fallen],
        (200, 100),
        [0, 0, 1, 1],
        0.4,
    ) == [valid]


def test_large_person_box_is_removed_as_same_fallen_target() -> None:
    fallen = Box(40, 40, 80, 80)
    large_duplicate = Box(10, 10, 110, 110)
    nearby_helper = Box(85, 40, 125, 80)

    assert filter_nonfallen_people(
        [large_duplicate, nearby_helper], [fallen], (200, 100),
        [0, 0, 1, 1], 0.4,
    ) == [nearby_helper]


@pytest.mark.parametrize(
    "count,level,multiplier",
    [(0, 0, 1.0), (1, 1, 1.1), (2, 2, 1.2), (3, 3, None), (8, 3, None)],
)
def test_crowd_levels_and_time_multiplier(count, level, multiplier) -> None:
    assert crowd_metrics(count) == (level, multiplier, multiplier is not None)


def test_crowd_functions_reject_negative_values() -> None:
    with pytest.raises(ValueError):
        crowd_metrics(-1)


def test_crowd_stabilizer_confirms_worsening_in_short_window() -> None:
    stabilizer = CrowdStateStabilizer(5, 3, 10, 7, 3.0)

    assert stabilizer.update(1, now=0.0) == 1
    assert stabilizer.update(3, now=0.1) == 1
    assert stabilizer.update(3, now=0.2) == 1
    assert stabilizer.update(3, now=0.3) == 3


def test_crowd_stabilizer_delays_improvement_until_hold_and_hits() -> None:
    stabilizer = CrowdStateStabilizer(5, 3, 10, 7, 3.0)

    assert stabilizer.update(3, now=0.0) == 3
    for index in range(6):
        assert stabilizer.update(0, now=0.5 + index * 0.4) == 3
    # 일곱 번째 CLEAR 관측이며 BLOCKED 유지시간도 3초를 넘겼다.
    assert stabilizer.update(0, now=3.1) == 0


def test_crowd_stabilizer_rejects_invalid_configuration_and_level() -> None:
    with pytest.raises(ValueError):
        CrowdStateStabilizer(2, 3, 10, 7, 3.0)
    with pytest.raises(ValueError):
        CrowdStateStabilizer(5, 3, 5, 0, 3.0)
    with pytest.raises(ValueError):
        CrowdStateStabilizer(5, 3, 10, 7, -1.0)

    stabilizer = CrowdStateStabilizer(5, 3, 10, 7, 3.0)
    with pytest.raises(ValueError):
        stabilizer.update(4)
