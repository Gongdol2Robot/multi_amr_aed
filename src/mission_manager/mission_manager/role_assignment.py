"""Pure role-assignment policy, independent of ROS runtime."""

import math


def select_roles(robot_positions, emergency_position):
    """Return `(aed_robot_id, guide_robot_id)` ordered by distance.

    `robot_positions` maps a robot ID to an `(x, y)` pair. At least two
    available robots are required.
    """
    if len(robot_positions) < 2:
        raise ValueError("at least two available robots are required")
    emergency_x, emergency_y = emergency_position
    ranked = sorted(
        robot_positions,
        key=lambda robot_id: (
            math.hypot(
                robot_positions[robot_id][0] - emergency_x,
                robot_positions[robot_id][1] - emergency_y,
            ),
            robot_id,
        ),
    )
    return ranked[0], ranked[1]

