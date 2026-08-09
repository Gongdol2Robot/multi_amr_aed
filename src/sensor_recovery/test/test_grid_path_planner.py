import math

from sensor_recovery.grid_path_planner import (
    OccupancyGridData,
    compute_clearance_field,
    path_segment_is_safe,
    plan_path,
    simplify_path,
)

FREE = 0
OCC = 100
UNKNOWN = -1


def _grid(rows, resolution=1.0, origin=(0.0, 0.0)):
    """rows[0] is the *bottom* row (row 0, smallest y) to match
    world_to_cell's y-up convention."""
    height = len(rows)
    width = len(rows[0])
    flat = []
    for row in rows:
        assert len(row) == width
        flat.extend(row)
    return OccupancyGridData(
        width=width, height=height, resolution=resolution,
        origin_x=origin[0], origin_y=origin[1], data=flat,
    )


def test_straight_line_open_field():
    rows = [[FREE] * 10 for _ in range(10)]
    grid = _grid(rows)
    path = plan_path(grid, (0.5, 0.5), (8.5, 0.5), robot_radius_m=0.1, hard_margin_m=0.0)
    assert path is not None
    assert path[0] == (0.5, 0.5)
    assert path[-1] == (8.5, 0.5)
    # Should not wander far from the straight line in an empty grid.
    for x, y in path:
        assert abs(y - 0.5) < 1.5


def test_blocked_by_full_wall_returns_none():
    rows = [[FREE] * 10 for _ in range(10)]
    for col in range(10):
        rows[5][col] = OCC
    grid = _grid(rows)
    path = plan_path(grid, (0.5, 0.5), (0.5, 9.5), robot_radius_m=0.1, hard_margin_m=0.0)
    assert path is None


def test_goes_around_a_partial_wall():
    rows = [[FREE] * 10 for _ in range(10)]
    for col in range(0, 8):
        rows[5][col] = OCC
    grid = _grid(rows)
    path = plan_path(grid, (0.5, 0.5), (0.5, 9.5), robot_radius_m=0.1, hard_margin_m=0.0)
    assert path is not None
    assert path[-1] == (0.5, 9.5)


def test_prefers_wide_corridor_over_narrow_one():
    # Two horizontal corridors from left to right: a wide one (rows 0-3)
    # and a narrow one (rows 6-6, squeezed by walls on both sides at 5/7).
    # Start/goal are equidistant from both, so with clearance weighting
    # the path should route through the wide corridor.
    width, height = 20, 10
    rows = [[FREE] * width for _ in range(height)]
    for col in range(width):
        rows[5][col] = OCC
        rows[7][col] = OCC
    grid = _grid(rows)
    path = plan_path(
        grid, (0.5, 1.5), (19.5, 1.5),
        robot_radius_m=0.1, hard_margin_m=0.0,
        soft_clearance_m=2.0, wall_clearance_weight=5.0,
    )
    assert path is not None
    # Every point should stay in the wide corridor (y < 5), never dip into
    # the narrow row-6 slot between the two walls.
    assert all(y < 5.0 for _, y in path)


def test_hard_radius_blocks_narrow_gap():
    # A one-cell-wide gap in a wall (1 m resolution, so the gap cell's own
    # clearance from either side is exactly 1 cell = 1.0 m). A robot radius
    # bigger than that clearance must make the gap impassable.
    rows = [[FREE] * 10 for _ in range(10)]
    for col in range(10):
        if col != 5:
            rows[5][col] = OCC
    grid = _grid(rows, resolution=1.0)
    path = plan_path(
        grid, (0.5, 0.5), (0.5, 9.5),
        robot_radius_m=1.1, hard_margin_m=0.0,
    )
    assert path is None


def test_unknown_blocked_by_default():
    rows = [[FREE] * 10 for _ in range(10)]
    for col in range(10):
        rows[5][col] = UNKNOWN
    grid = _grid(rows)
    path = plan_path(grid, (0.5, 0.5), (0.5, 9.5), robot_radius_m=0.1, hard_margin_m=0.0)
    assert path is None


def test_unknown_allowed_when_flag_set():
    rows = [[FREE] * 10 for _ in range(10)]
    for col in range(10):
        rows[5][col] = UNKNOWN
    grid = _grid(rows)
    path = plan_path(
        grid, (0.5, 0.5), (0.5, 9.5),
        robot_radius_m=0.1, hard_margin_m=0.0, allow_unknown=True,
    )
    assert path is not None


def test_start_inside_inflation_radius_can_still_escape():
    # Robot's last known pose sits one cell from a single wall cell
    # (clearance below the hard radius at the start cell itself) — this
    # must not make planning fail outright. The start has other neighbors
    # just outside the inflation radius, so it can still escape; only the
    # *rest* of the path is required to respect the hard radius.
    rows = [[FREE] * 10 for _ in range(10)]
    rows[5][5] = OCC
    grid = _grid(rows)
    start_xy = (5.5, 4.5)
    goal_xy = (1.5, 1.5)
    path = plan_path(
        grid, start_xy, goal_xy,
        robot_radius_m=1.5, hard_margin_m=0.0,
    )
    assert path is not None
    assert path[0] == start_xy
    assert path[-1] == goal_xy


def test_start_or_goal_out_of_bounds_returns_none():
    rows = [[FREE] * 5 for _ in range(5)]
    grid = _grid(rows)
    assert plan_path(grid, (-5.0, -5.0), (2.5, 2.5)) is None
    assert plan_path(grid, (2.5, 2.5), (50.0, 50.0)) is None


def test_start_equals_goal_returns_two_point_path():
    rows = [[FREE] * 5 for _ in range(5)]
    grid = _grid(rows)
    path = plan_path(grid, (2.5, 2.5), (2.5, 2.5))
    assert path == [(2.5, 2.5), (2.5, 2.5)]


def test_goal_on_occupied_cell_returns_none():
    rows = [[FREE] * 5 for _ in range(5)]
    rows[2][2] = OCC
    grid = _grid(rows)
    path = plan_path(grid, (0.5, 0.5), (2.5, 2.5), robot_radius_m=0.1, hard_margin_m=0.0)
    assert path is None


def test_clearance_field_zero_at_walls_and_grows_away():
    rows = [[FREE] * 5 for _ in range(5)]
    rows[2][2] = OCC
    grid = _grid(rows)
    clearance = compute_clearance_field(grid)
    assert clearance[grid.index(2, 2)] == 0.0
    assert clearance[grid.index(2, 3)] == grid.resolution
    far = clearance[grid.index(0, 0)]
    assert far > clearance[grid.index(2, 3)]


def test_clearance_field_respects_resolution_scale():
    rows = [[FREE] * 5 for _ in range(5)]
    rows[2][2] = OCC
    grid = _grid(rows, resolution=0.1)
    clearance = compute_clearance_field(grid)
    assert math.isclose(clearance[grid.index(2, 3)], 0.1)


def test_clearance_field_uses_euclidean_diagonal_distance():
    rows = [[FREE] * 5 for _ in range(5)]
    rows[2][2] = OCC
    grid = _grid(rows, resolution=0.1)
    clearance = compute_clearance_field(grid)
    assert math.isclose(
        clearance[grid.index(3, 3)], math.sqrt(2.0) * 0.1
    )


def test_precomputed_clearance_field_reused():
    rows = [[FREE] * 10 for _ in range(10)]
    for col in range(0, 8):
        rows[5][col] = OCC
    grid = _grid(rows)
    clearance = compute_clearance_field(grid)
    path = plan_path(
        grid, (0.5, 0.5), (0.5, 9.5),
        robot_radius_m=0.1, hard_margin_m=0.0, clearance=clearance,
    )
    assert path is not None


def test_astar_does_not_cut_diagonally_between_blocked_cells():
    rows = [[FREE] * 3 for _ in range(3)]
    rows[0][1] = OCC
    rows[1][0] = OCC
    grid = _grid(rows)
    path = plan_path(
        grid,
        (0.5, 0.5),
        (1.5, 1.5),
        robot_radius_m=0.1,
        hard_margin_m=0.0,
    )
    assert path is None


def test_simplify_path_removes_collinear_grid_points():
    rows = [[FREE] * 8 for _ in range(4)]
    grid = _grid(rows)
    clearance = compute_clearance_field(grid)
    path = [(0.5, 1.5), (1.5, 1.5), (2.5, 1.5), (6.5, 1.5)]
    simplified = simplify_path(
        grid,
        path,
        clearance,
        robot_radius_m=0.1,
        hard_margin_m=0.0,
    )
    assert simplified == [path[0], path[-1]]


def test_simplify_path_preserves_corner_around_obstacle():
    rows = [[FREE] * 7 for _ in range(7)]
    rows[3][3] = OCC
    grid = _grid(rows)
    clearance = compute_clearance_field(grid)
    path = [
        (1.5, 3.5),
        (2.5, 2.5),
        (3.5, 1.5),
        (4.5, 2.5),
        (5.5, 3.5),
    ]
    simplified = simplify_path(
        grid,
        path,
        clearance,
        robot_radius_m=0.1,
        hard_margin_m=0.0,
    )
    assert simplified[0] == path[0]
    assert simplified[-1] == path[-1]
    assert len(simplified) > 2
    for start, end in zip(simplified, simplified[1:]):
        assert path_segment_is_safe(
            grid,
            start,
            end,
            clearance,
            robot_radius_m=0.1,
            hard_margin_m=0.0,
        )


def test_simplify_path_does_not_cross_hard_clearance_margin():
    rows = [[FREE] * 7 for _ in range(7)]
    rows[3][3] = OCC
    grid = _grid(rows)
    clearance = compute_clearance_field(grid)
    path = [(0.5, 2.5), (1.5, 1.5), (5.5, 1.5), (6.5, 2.5)]
    simplified = simplify_path(
        grid,
        path,
        clearance,
        robot_radius_m=1.1,
        hard_margin_m=0.0,
    )
    assert simplified != [path[0], path[-1]]


def test_world_to_cell_uses_floor_for_negative_out_of_bounds_coordinate():
    grid = _grid([[FREE] * 3 for _ in range(3)])
    assert grid.world_to_cell(-0.1, 0.5) == (-1, 0)
