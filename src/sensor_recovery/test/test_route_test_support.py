import math

import pytest

from sensor_recovery.route_test_support import (
    densify_route,
    minimum_range_in_sector,
    parse_flat_route,
)


def test_parse_flat_route_builds_points():
    assert parse_flat_route([0, 1, 2, 3, 4, 5]) == [
        (0.0, 1.0),
        (2.0, 3.0),
        (4.0, 5.0),
    ]


@pytest.mark.parametrize("values", [[], [0, 1], [0, 1, 2], [0, 1, math.inf, 2]])
def test_parse_flat_route_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        parse_flat_route(values)


def test_densify_route_preserves_corner_and_endpoints():
    result = densify_route([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 0.25)
    assert result[0] == (0.0, 0.0)
    assert (1.0, 0.0) in result
    assert result[-1] == (1.0, 1.0)
    assert max(math.dist(a, b) for a, b in zip(result, result[1:])) <= 0.25


def test_densify_route_rejects_zero_length_route():
    with pytest.raises(ValueError):
        densify_route([(1.0, 1.0), (1.0, 1.0)], 0.1)


def test_minimum_range_uses_only_forward_sector_and_valid_values():
    ranges = [0.1, 2.0, float("nan"), 0.4, 0.2]
    result = minimum_range_in_sector(
        ranges, angle_min=-1.0, angle_increment=0.5, half_angle_rad=0.55
    )
    assert math.isclose(result, 0.4)


def test_minimum_range_returns_inf_without_valid_sample():
    result = minimum_range_in_sector(
        [float("inf"), float("nan")], -0.1, 0.2, 0.5
    )
    assert math.isinf(result)
