"""Pure role-assignment policy, independent of ROS runtime."""

import math


def rank_candidates(robot_candidates, emergency_position):
    """Order path-valid robots by path cost, then direct distance.

    A non-negative path cost is preferred. Direct distance is used as a
    temporary fallback until the Nav2 path-cost response is available.
    """
    valid = {
        robot_id: candidate
        for robot_id, candidate in robot_candidates.items()
        if candidate["path_valid"]
    }
    if not valid:
        return []
    emergency_x, emergency_y = emergency_position
    return sorted(
        valid,
        key=lambda robot_id: (
            valid[robot_id]["path_cost"]
            if valid[robot_id]["path_cost"] >= 0.0
            else math.hypot(
                valid[robot_id]["position"][0] - emergency_x,
                valid[robot_id]["position"][1] - emergency_y,
            ),
            robot_id,
        ),
    )
