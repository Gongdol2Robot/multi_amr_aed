"""RGB 화면 좌표 기반 조력자-환자 근접 필터 테스트."""

import pytest

from aed_vision.detection_logic import Box, filter_helpers_near_fallen


def test_keeps_helper_near_fallen_person() -> None:
    fallen = [Box(250, 250, 390, 390)]
    nearby = Box(390, 150, 470, 360)

    assert filter_helpers_near_fallen(
        [nearby], fallen, (640, 480), 0.30
    ) == [nearby]


def test_rejects_helper_far_from_fallen_person() -> None:
    fallen = [Box(20, 300, 140, 450)]
    far_away = Box(520, 20, 620, 220)

    assert filter_helpers_near_fallen(
        [far_away], fallen, (640, 480), 0.30
    ) == []


def test_rejects_every_helper_when_fallen_person_is_missing() -> None:
    helper = Box(200, 100, 300, 400)

    assert filter_helpers_near_fallen(
        [helper], [], (640, 480), 0.30
    ) == []


def test_rejects_invalid_distance_ratio() -> None:
    with pytest.raises(ValueError):
        filter_helpers_near_fallen(
            [Box(1, 1, 2, 2)], [Box(1, 1, 2, 2)], (640, 480), 0.0
        )
