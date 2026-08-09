"""호모그래피 투영과 측량 영역 판정 테스트."""

import numpy as np
import pytest

from aed_vision.homography import Homography, _distance_to_segment


def test_identity_projection_round_trip() -> None:
    homography = Homography(np.eye(3))

    assert homography.pixel_to_map(12.5, 8.0) == (12.5, 8.0)
    assert homography.map_to_pixel(12.5, 8.0) == (12.5, 8.0)


def test_box_to_map_uses_scaled_bottom_center() -> None:
    homography = Homography(
        np.eye(3), camera={"width": 640, "height": 480}
    )

    assert homography.box_to_map(
        100, 50, 300, 200, image_size=(320, 240)
    ) == (400.0, 400.0)


def test_survey_area_accepts_inside_and_margin() -> None:
    homography = Homography(
        np.eye(3), survey_area=[(0, 0), (2, 0), (2, 2), (0, 2)]
    )

    assert homography.inside_survey_area(1, 1)
    assert not homography.inside_survey_area(2.2, 1)
    assert homography.inside_survey_area(2.2, 1, margin=0.21)


def test_distance_to_zero_length_segment() -> None:
    assert _distance_to_segment((3, 4), (0, 0), (0, 0)) == pytest.approx(5)
