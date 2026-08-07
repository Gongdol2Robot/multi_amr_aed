import math

import pytest

from robot_missions.mission_executor import angle_distance, pose_has_progress


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
