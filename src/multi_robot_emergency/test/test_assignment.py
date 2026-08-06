import math

import pytest

from multi_robot_emergency.assignment import path_length


def test_path_length_accumulates_every_segment() -> None:
    assert path_length([(0.0, 0.0), (3.0, 4.0), (6.0, 4.0)]) == 8.0


def test_empty_path_has_zero_length() -> None:
    assert path_length([]) == 0.0


def test_non_finite_path_is_rejected() -> None:
    with pytest.raises(ValueError):
        path_length([(0.0, 0.0), (math.inf, 0.0)])
