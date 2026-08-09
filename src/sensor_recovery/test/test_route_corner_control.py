import math
from pathlib import Path

import yaml

from sensor_recovery.route_corner_control import (
    build_route_geometry,
    corner_speed_limit,
    heading_after_index,
    interior_corner_indices,
    remaining_path_distance,
    select_target_before_index,
)


ROBOT1_ROUTE = (
    Path(__file__).parents[1] / "config" / "robot1_undock_to_goal.yaml"
)


def test_straight_route_has_no_hard_corner():
    route = build_route_geometry(
        [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)],
        math.radians(35.0),
        0.2,
        0.3,
    )
    assert route.corner_indices == ()


def test_l_route_has_one_corner():
    points = [(index * 0.05, 0.0) for index in range(11)]
    points += [(0.5, index * 0.05) for index in range(1, 11)]
    route = build_route_geometry(points, math.radians(35.0), 0.2, 0.3)
    assert len(route.corner_indices) == 1
    corner = route.points[route.corner_indices[0]]
    assert math.hypot(corner[0] - 0.5, corner[1]) <= 0.05


def test_robot1_route_keeps_real_corner_and_filters_endpoint_artifacts():
    document = yaml.safe_load(ROBOT1_ROUTE.read_text(encoding="utf-8"))
    geometry = build_route_geometry(
        document["route"]["points"], math.radians(35.0), 0.25, 0.35
    )
    assert geometry.corner_indices == (2, 45, 209)
    assert interior_corner_indices(geometry, 0.25, 0.25) == (45,)
    assert geometry.points[45] == (-1.449, 0.89)
    assert math.isclose(
        math.degrees(heading_after_index(geometry, 45, 0.25)),
        60.9,
        abs_tol=0.2,
    )


def test_lookahead_never_crosses_corner():
    points = [(index * 0.1, 0.0) for index in range(6)]
    points += [(0.5, index * 0.1) for index in range(1, 6)]
    route = build_route_geometry(points, math.radians(35.0), 0.2, 0.3)
    corner_index = route.corner_indices[0]
    _, target_index = select_target_before_index(route, 2, 5.0, corner_index)
    assert target_index == corner_index


def test_heading_after_corner_uses_outgoing_segment():
    route = build_route_geometry(
        [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)],
        math.radians(35.0),
        0.2,
        0.3,
    )
    assert math.isclose(
        heading_after_index(route, route.corner_indices[0], 0.2),
        math.pi / 2,
    )


def test_remaining_distance_and_corner_speed_decrease_together():
    route = build_route_geometry(
        [(0.0, 0.0), (0.25, 0.0), (0.5, 0.0), (0.5, 0.5)],
        math.radians(35.0),
        0.2,
        0.3,
    )
    corner = route.corner_indices[0]
    far = remaining_path_distance(route, 0, corner)
    near = remaining_path_distance(route, 1, corner)
    assert far > near
    assert corner_speed_limit(0.05, far, 0.4) > corner_speed_limit(
        0.05, near, 0.4
    )
