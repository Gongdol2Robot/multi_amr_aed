"""Pure grid-based path planning over a static occupancy grid map.

No ROS, no live sensors, no Nav2 — only the (latched) static map plus a
known start/goal pose. This exists because Nav2's own planner_server can't
be trusted once LiDAR is dead: its global costmap has an obstacle_layer
sourced from /scan, and AMCL's map->odom TF eventually goes stale too, so
compute_path_to_pose can fail or hang for the same underlying reason the
robot lost LiDAR in the first place. A plan computed straight from the
static map has no such dependency.

Biases toward wall clearance rather than shortest distance: cells within
robot_radius_m (+ a safety margin) of any known obstacle are outright
blocked, and cells that are merely close to a wall (but still technically
passable) are penalized so the search prefers open space when a choice
exists.
"""

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Cell = Tuple[int, int]
WorldPoint = Tuple[float, float]

_SQRT2 = math.sqrt(2.0)
_NEIGHBORS_8 = (
    (-1, -1, _SQRT2), (-1, 0, 1.0), (-1, 1, _SQRT2),
    (0, -1, 1.0), (0, 1, 1.0),
    (1, -1, _SQRT2), (1, 0, 1.0), (1, 1, _SQRT2),
)


@dataclass(frozen=True)
class OccupancyGridData:
    """ROS-free mirror of nav_msgs/OccupancyGrid: row-major, -1 unknown,
    0-100 occupancy probability, origin at the cell (0, 0)'s corner."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def index(self, col: int, row: int) -> int:
        return row * self.width + col

    def value(self, col: int, row: int) -> int:
        return self.data[self.index(col, row)]

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def world_to_cell(self, x: float, y: float) -> Cell:
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        return col, row

    def cell_to_world(self, col: int, row: int) -> WorldPoint:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y


def compute_clearance_field(
    grid: OccupancyGridData,
    occupied_threshold: int = 50,
    max_clearance_cells: int = 60,
) -> List[float]:
    """Exact Euclidean cell distance to the nearest known occupied cell.

    Unknown (-1) cells are not treated as walls here
    — passability for unknown space is a separate policy decision (see
    `plan_path`'s `allow_unknown`), clearance is purely about known walls.
    Distance saturates at `max_clearance_cells` (only used to shape cost
    near walls, so anything farther is "clearly open" either way).
    """
    width, height = grid.width, grid.height
    infinity = float((width + height + max_clearance_cells + 1) ** 2)

    def transform_1d(values: Sequence[float]) -> List[float]:
        """Felzenszwalb/Huttenlocher squared Euclidean transform."""
        count = len(values)
        sites = [0] * count
        boundaries = [0.0] * (count + 1)
        result = [0.0] * count
        envelope = 0
        boundaries[0] = -math.inf
        boundaries[1] = math.inf
        for q in range(1, count):
            site = sites[envelope]
            crossing = (
                (values[q] + q * q) - (values[site] + site * site)
            ) / (2.0 * (q - site))
            while crossing <= boundaries[envelope]:
                envelope -= 1
                site = sites[envelope]
                crossing = (
                    (values[q] + q * q) - (values[site] + site * site)
                ) / (2.0 * (q - site))
            envelope += 1
            sites[envelope] = q
            boundaries[envelope] = crossing
            boundaries[envelope + 1] = math.inf
        envelope = 0
        for q in range(count):
            while boundaries[envelope + 1] < q:
                envelope += 1
            delta = q - sites[envelope]
            result[q] = delta * delta + values[sites[envelope]]
        return result

    vertical = [infinity] * (width * height)
    for col in range(width):
        column = [
            0.0
            if grid.value(col, row) >= occupied_threshold
            else infinity
            for row in range(height)
        ]
        transformed = transform_1d(column)
        for row, distance_sq in enumerate(transformed):
            vertical[grid.index(col, row)] = distance_sq

    squared = [infinity] * (width * height)
    for row in range(height):
        transformed = transform_1d(
            [vertical[grid.index(col, row)] for col in range(width)]
        )
        for col, distance_sq in enumerate(transformed):
            squared[grid.index(col, row)] = distance_sq

    return [
        min(math.sqrt(distance_sq), max_clearance_cells) * grid.resolution
        for distance_sq in squared
    ]


def plan_path(
    grid: OccupancyGridData,
    start_xy: WorldPoint,
    goal_xy: WorldPoint,
    robot_radius_m: float = 0.20,
    hard_margin_m: float = 0.05,
    soft_clearance_m: float = 0.4,
    wall_clearance_weight: float = 2.0,
    allow_unknown: bool = False,
    occupied_threshold: int = 50,
    clearance: Optional[List[float]] = None,
    max_expansions: int = 400000,
) -> Optional[List[WorldPoint]]:
    """8-connected A* from start to goal over the static grid.

    Cells within `robot_radius_m + hard_margin_m` of a known wall are
    impassable. Among passable cells, distance cost is scaled up as
    clearance drops below `soft_clearance_m`, by up to
    `1 + wall_clearance_weight`, so the search prefers open space over the
    literal shortest path when both are available. The start cell is
    exempt from the hard-radius block (the robot may already be legitimately
    close to a wall when the fault happened) but not from being on a known
    occupied cell.

    Returns world-frame waypoints (start_xy first, goal_xy last) or None if
    no path exists / start or goal is unusable.
    """
    if clearance is None:
        clearance = compute_clearance_field(grid, occupied_threshold)

    hard_radius_m = robot_radius_m + hard_margin_m

    def occupied(col: int, row: int) -> bool:
        v = grid.value(col, row)
        if v >= occupied_threshold:
            return True
        if v < 0 and not allow_unknown:
            return True
        return False

    def passable(col: int, row: int) -> bool:
        if occupied(col, row):
            return False
        return clearance[grid.index(col, row)] >= hard_radius_m

    start_cell = grid.world_to_cell(*start_xy)
    goal_cell = grid.world_to_cell(*goal_xy)
    if not grid.in_bounds(*start_cell) or not grid.in_bounds(*goal_cell):
        return None
    if occupied(*start_cell):
        return None
    if not passable(*goal_cell):
        return None
    if start_cell == goal_cell:
        return [start_xy, goal_xy]

    def cell_passable(cell: Cell) -> bool:
        return cell == start_cell or passable(*cell)

    def step_cost(cell: Cell) -> float:
        if soft_clearance_m <= 0.0:
            return 1.0
        c = clearance[grid.index(*cell)]
        penalty = max(0.0, 1.0 - c / soft_clearance_m)
        return 1.0 + wall_clearance_weight * penalty

    def heuristic(cell: Cell) -> float:
        return math.hypot(cell[0] - goal_cell[0], cell[1] - goal_cell[1])

    open_heap: List[Tuple[float, Cell]] = [(0.0, start_cell)]
    g_score = {start_cell: 0.0}
    came_from = {}
    visited = set()
    expansions = 0

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal_cell:
            break
        expansions += 1
        if expansions > max_expansions:
            return None
        for dc, dr, base_dist in _NEIGHBORS_8:
            neighbor = (current[0] + dc, current[1] + dr)
            if not grid.in_bounds(*neighbor) or neighbor in visited:
                continue
            if not cell_passable(neighbor):
                continue
            if dc != 0 and dr != 0:
                # Never squeeze diagonally between blocked orthogonal cells.
                # Besides being unsafe for a circular robot, such corner
                # cuts create sharp one-cell zigzags for the follower.
                if not cell_passable((current[0] + dc, current[1])):
                    continue
                if not cell_passable((current[0], current[1] + dr)):
                    continue
            tentative = g_score[current] + base_dist * step_cost(neighbor)
            if tentative < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                heapq.heappush(open_heap, (tentative + heuristic(neighbor), neighbor))

    if goal_cell not in visited:
        return None

    path_cells = [goal_cell]
    cur = goal_cell
    while cur != start_cell:
        cur = came_from[cur]
        path_cells.append(cur)
    path_cells.reverse()

    waypoints = [grid.cell_to_world(c, r) for c, r in path_cells]
    waypoints[0] = start_xy
    waypoints[-1] = goal_xy
    return waypoints


def _supercover_cells(start: Cell, end: Cell) -> List[Cell]:
    """Every grid cell touched by the line between two cell centers."""
    x, y = start
    end_x, end_y = end
    dx = end_x - x
    dy = end_y - y
    nx = abs(dx)
    ny = abs(dy)
    step_x = 0 if dx == 0 else (1 if dx > 0 else -1)
    step_y = 0 if dy == 0 else (1 if dy > 0 else -1)
    ix = iy = 0
    cells = [(x, y)]

    def add(cell: Cell) -> None:
        if cell not in cells:
            cells.append(cell)

    while ix < nx or iy < ny:
        decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if decision == 0:
            # The line crosses a cell corner. Include both side cells as
            # well as the diagonal cell so a shortcut cannot pass through
            # the corner between two walls.
            if ix < nx:
                add((x + step_x, y))
            if iy < ny:
                add((x, y + step_y))
            if ix < nx:
                x += step_x
                ix += 1
            if iy < ny:
                y += step_y
                iy += 1
            add((x, y))
        elif decision < 0:
            x += step_x
            ix += 1
            add((x, y))
        else:
            y += step_y
            iy += 1
            add((x, y))
    return cells


def path_segment_is_safe(
    grid: OccupancyGridData,
    start_xy: WorldPoint,
    end_xy: WorldPoint,
    clearance: Sequence[float],
    robot_radius_m: float,
    hard_margin_m: float,
    allow_unknown: bool = False,
    occupied_threshold: int = 50,
    allow_start_inside_margin: bool = False,
) -> bool:
    """Whether a straight shortcut stays inside known, wall-clear cells."""
    start_cell = grid.world_to_cell(*start_xy)
    end_cell = grid.world_to_cell(*end_xy)
    hard_radius_m = robot_radius_m + hard_margin_m
    for cell in _supercover_cells(start_cell, end_cell):
        if not grid.in_bounds(*cell):
            return False
        value = grid.value(*cell)
        if value >= occupied_threshold or (value < 0 and not allow_unknown):
            return False
        if cell == start_cell and allow_start_inside_margin:
            continue
        if clearance[grid.index(*cell)] < hard_radius_m:
            return False
    return True


def simplify_path(
    grid: OccupancyGridData,
    path: Sequence[WorldPoint],
    clearance: Sequence[float],
    robot_radius_m: float = 0.20,
    hard_margin_m: float = 0.05,
    allow_unknown: bool = False,
    occupied_threshold: int = 50,
) -> List[WorldPoint]:
    """Greedily remove dense A* points using clearance-safe line of sight.

    Start and goal are always preserved. Every shortcut is checked against
    all grid cells it touches and uses the same occupancy/clearance policy
    as A*, so simplification cannot cut through a wall or its hard margin.
    """
    if len(path) <= 2:
        return list(path)

    simplified = [path[0]]
    anchor_index = 0
    last_index = len(path) - 1
    while anchor_index < last_index:
        next_index = anchor_index + 1
        for candidate_index in range(last_index, anchor_index, -1):
            if path_segment_is_safe(
                grid,
                path[anchor_index],
                path[candidate_index],
                clearance,
                robot_radius_m,
                hard_margin_m,
                allow_unknown,
                occupied_threshold,
                allow_start_inside_margin=anchor_index == 0,
            ):
                next_index = candidate_index
                break
        simplified.append(path[next_index])
        anchor_index = next_index
    return simplified
