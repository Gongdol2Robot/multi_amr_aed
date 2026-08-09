"""검출 후처리와 시간 확정 로직의 단위 테스트."""

import pytest

from aed_vision.detection_logic import (
    Box,
    TemporalConfirmation,
    apply_crowd_time_penalty,
    classify_crowd,
    filter_nonfallen_people,
    intersection_over_union,
    point_inside_normalized_roi,
    update_presence_confirmation,
)


def test_temporal_confirmation_uses_recent_window() -> None:
    confirmation = TemporalConfirmation(4, 3)

    assert confirmation.update(True) is False
    assert confirmation.update(False) is False
    assert confirmation.update(True) is False
    assert confirmation.update(True) is True
    assert confirmation.hit_count == 3
    assert confirmation.update(False) is False


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
    "count,level,multiplied",
    [(0, 0, 10.0), (1, 1, 11.0), (2, 2, 12.0), (3, 3, None), (8, 3, None)],
)
def test_crowd_levels_and_time_penalty(count, level, multiplied) -> None:
    assert classify_crowd(count) == level
    assert apply_crowd_time_penalty(10.0, count) == multiplied


def test_crowd_functions_reject_negative_values() -> None:
    with pytest.raises(ValueError):
        classify_crowd(-1)
    with pytest.raises(ValueError):
        apply_crowd_time_penalty(-0.1, 0)
