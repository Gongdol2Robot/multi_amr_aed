"""검출 후처리와 시간 확정 로직의 단위 테스트."""

import pytest

from aed_vision.detection_logic import (
    Box,
    CrowdStateStabilizer,
    TemporalConfirmation,
    crowd_metrics,
    filter_nonfallen_people,
    intersection_over_union,
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
