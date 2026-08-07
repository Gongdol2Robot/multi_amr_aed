import math

import pytest

from multi_robot_emergency.assignment import (
    crowd_delay_seconds,
    dispatch_candidates,
    path_length,
    path_length_in_polygon,
    path_motion_cost,
    point_in_polygon,
    point_to_polygon_distance,
    simplify_path,
)


def test_dispatch_candidates_uses_one_robot_below_deadline_risk() -> None:
    assert dispatch_candidates(
        [("robot1", 25.49), ("robot2", 27.0)],
        dual_dispatch_enabled=True,
        target_arrival_time=30.0,
        trigger_ratio=0.85,
    ) == ["robot1"]


def test_dispatch_candidates_uses_both_at_deadline_risk_boundary() -> None:
    assert dispatch_candidates(
        [("robot1", 25.5), ("robot2", 27.0)],
        dual_dispatch_enabled=True,
        target_arrival_time=30.0,
        trigger_ratio=0.85,
    ) == ["robot1", "robot2"]


def test_dispatch_candidates_requires_two_valid_candidates() -> None:
    assert dispatch_candidates(
        [("robot1", 40.0)],
        dual_dispatch_enabled=True,
        target_arrival_time=30.0,
        trigger_ratio=0.85,
    ) == ["robot1"]


def test_dispatch_candidates_can_be_disabled() -> None:
    assert dispatch_candidates(
        [("robot1", 40.0), ("robot2", 41.0)],
        dual_dispatch_enabled=False,
        target_arrival_time=30.0,
        trigger_ratio=0.85,
    ) == ["robot1"]


@pytest.mark.parametrize(
    "target,ratio", [(0.0, 0.85), (30.0, 0.0), (30.0, 1.1)]
)
def test_dispatch_candidates_rejects_invalid_policy(
    target: float, ratio: float
) -> None:
    with pytest.raises(ValueError):
        dispatch_candidates(
            [("robot1", 10.0)],
            dual_dispatch_enabled=True,
            target_arrival_time=target,
            trigger_ratio=ratio,
        )


def test_path_length_accumulates_every_segment() -> None:
    assert path_length([(0.0, 0.0), (3.0, 4.0), (6.0, 4.0)]) == 8.0


def test_empty_path_has_zero_length() -> None:
    assert path_length([]) == 0.0


def test_non_finite_path_is_rejected() -> None:
    with pytest.raises(ValueError):
        path_length([(0.0, 0.0), (math.inf, 0.0)])


def test_polygon_contains_boundary_and_rejects_short_polygon() -> None:
    polygon = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    assert point_in_polygon((1.0, 1.0), polygon)
    assert point_in_polygon((2.0, 1.0), polygon)
    assert not point_in_polygon((3.0, 1.0), polygon)
    with pytest.raises(ValueError):
        point_in_polygon((0.0, 0.0), [])


def test_point_to_polygon_distance_handles_inside_and_outside() -> None:
    polygon = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    assert point_to_polygon_distance((1.0, 1.0), polygon) == 0.0
    assert point_to_polygon_distance((3.0, 1.0), polygon) == 1.0
    assert point_to_polygon_distance((3.0, 3.0), polygon) == pytest.approx(
        math.sqrt(2.0)
    )


def test_path_length_in_polygon_uses_segment_midpoints() -> None:
    polygon = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    points = [(-1.0, 1.0), (0.0, 1.0), (1.0, 1.0), (2.0, 1.0),
              (3.0, 1.0)]
    assert path_length_in_polygon(points, polygon) == pytest.approx(2.0)


def test_crowd_delay_adds_only_extra_travel_time() -> None:
    assert crowd_delay_seconds(
        1.0, normal_speed=0.20, crowded_speed=0.10
    ) == pytest.approx(5.0)
    with pytest.raises(ValueError):
        crowd_delay_seconds(
            1.0, normal_speed=0.20, crowded_speed=0.30
        )


def test_motion_cost_adds_turn_and_slowdown_penalty() -> None:
    eta, turn, slowdown_count = path_motion_cost(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        linear_speed=1.0,
        angular_speed=1.0,
        slowdown_turn_threshold=math.radians(45.0),
        slowdown_penalty=2.0,
    )
    assert eta == pytest.approx(2.0 + math.pi / 2.0 + 2.0)
    assert turn == pytest.approx(math.pi / 2.0)
    assert slowdown_count == 1


def test_straight_path_has_no_turn_cost() -> None:
    eta, turn, slowdown_count = path_motion_cost(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        linear_speed=0.5,
        angular_speed=1.0,
        slowdown_turn_threshold=math.radians(45.0),
        slowdown_penalty=1.0,
    )
    assert eta == pytest.approx(4.0)
    assert turn == 0.0
    assert slowdown_count == 0


def test_simplification_removes_grid_jitter() -> None:
    points = [(0.0, 0.0), (0.5, 0.01), (1.0, -0.01), (1.5, 0.0)]
    assert simplify_path(points, 0.05) == [points[0], points[-1]]
    eta, turn, slowdown_count = path_motion_cost(
        points,
        linear_speed=0.5,
        angular_speed=1.0,
        slowdown_turn_threshold=math.radians(45.0),
        slowdown_penalty=1.0,
        simplification_tolerance=0.05,
    )
    assert eta == pytest.approx(path_length(points) / 0.5)
    assert turn == 0.0
    assert slowdown_count == 0


def test_motion_cost_includes_endpoint_turns_without_corner_penalty() -> None:
    eta, turn, slowdown_count = path_motion_cost(
        [(0.0, 0.0), (1.0, 0.0)],
        linear_speed=1.0,
        angular_speed=1.0,
        slowdown_turn_threshold=math.radians(45.0),
        slowdown_penalty=2.0,
        initial_yaw=math.pi / 2.0,
        final_yaw=-math.pi / 2.0,
    )
    assert turn == pytest.approx(math.pi)
    assert slowdown_count == 0
    assert eta == pytest.approx(1.0 + math.pi)


@pytest.mark.parametrize(
    "argument",
    [
        {"linear_speed": math.nan},
        {"angular_speed": math.nan},
        {"slowdown_penalty": math.nan},
        {"simplification_tolerance": math.nan},
    ],
)
def test_motion_cost_rejects_non_finite_parameters(argument) -> None:
    values = {
        "linear_speed": 1.0,
        "angular_speed": 1.0,
        "slowdown_turn_threshold": 0.5,
        "slowdown_penalty": 0.0,
    }
    values.update(argument)
    with pytest.raises(ValueError):
        path_motion_cost([(0.0, 0.0), (1.0, 0.0)], **values)
