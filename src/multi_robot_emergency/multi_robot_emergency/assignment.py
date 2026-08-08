"""Pure policies used by the emergency mission manager.

[CODE REVIEW]
ROS2 I/O와 분리한 순수 정책/수학 모듈이다. 입력값만 주면 결과가 결정되므로
standoff, dual dispatch, live ETA switch, ETA 계산을 단위 테스트하기 쉽다.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

Position = tuple[float, float]


def proximity_retreat_candidate(
    robot_positions: dict[str, Position],
    patient: Position,
    *,
    primary_robot: str,
    threshold: float,
) -> str | None:
    """Choose the robot farther from the patient when two robots are close."""
    # dual dispatch 중 두 로봇이 너무 가까워졌을 때만 사용한다.
    # 반환값은 "계속 갈 로봇"이 아니라 "복귀시킬 로봇"이다.
    if len(robot_positions) != 2:
        raise ValueError("exactly two robot positions are required")
    values = (
        *(value for point in robot_positions.values() for value in point),
        *patient,
        threshold,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("proximity inputs must be finite")
    if threshold <= 0.0:
        raise ValueError("proximity threshold must be positive")
    first_id, second_id = robot_positions
    first = robot_positions[first_id]
    second = robot_positions[second_id]
    if math.hypot(first[0] - second[0], first[1] - second[1]) > threshold:
        return None
    return max(
        robot_positions,
        key=lambda robot_id: (
            math.hypot(
                robot_positions[robot_id][0] - patient[0],
                robot_positions[robot_id][1] - patient[1],
            ),
            robot_id != primary_robot,
        ),
    )


def patient_standoff(
    patient: Position,
    robot: Position,
    distance: float,
    *,
    fallback_robot_yaw: float = 0.0,
) -> tuple[float, float, float]:
    """Return a stop point on the patient circle and a patient-facing yaw."""
    # [CODE REVIEW] 환자 중심 반지름 distance 원 위에 정지점을 만들고,
    # 최종 yaw는 환자를 바라보게 해서 안전거리와 접근 방향을 동시에 만족시킨다.
    values = (*patient, *robot, distance, fallback_robot_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("patient standoff inputs must be finite")
    if distance <= 0.0:
        raise ValueError("patient standoff distance must be positive")

    # patient -> robot 방향 벡터를 구한다. 이 방향을 유지한 채 환자 중심에서
    # distance만큼 떨어진 원 위의 점을 실제 정지점으로 사용한다.
    dx = robot[0] - patient[0]
    dy = robot[1] - patient[1]
    radial_angle = (
        # 로봇과 환자 좌표가 사실상 같으면 방향 계산이 불가능하므로
        # 현재 로봇 yaw의 반대 방향을 임시 radial 방향으로 사용한다.
        fallback_robot_yaw + math.pi
        if math.hypot(dx, dy) <= 1e-6
        else math.atan2(dy, dx)
    )
    # 환자와 정확히 distance만큼 떨어진 최종 NavigateToPose goal 좌표.
    stop_x = patient[0] + distance * math.cos(radial_angle)
    stop_y = patient[1] + distance * math.sin(radial_angle)
    # radial 방향의 반대를 바라보게 하면 최종 자세가 환자 방향을 향한다.
    facing_yaw = normalize_angle(radial_angle + math.pi)
    return stop_x, stop_y, facing_yaw


def dispatch_candidates(
    ranked_candidates: Iterable[tuple[str, float]],
    *,
    dual_dispatch_enabled: bool,
    target_arrival_time: float,
    trigger_ratio: float,
) -> list[str]:
    """Choose one primary robot or both robots for a deadline-risk mission.

    ``ranked_candidates`` must be ordered from the smallest ETA to the
    largest. Dual dispatch is used only when both candidates are valid and
    even the fastest ETA reaches the configured fraction of the target time.
    """
    # [CODE REVIEW] 기본값은 30 s 목표 * 0.85 = 25.5 s.
    # 가장 빠른 로봇도 임계 ETA를 넘으면 두 대를 보내 deadline miss 위험을 줄인다.
    if not math.isfinite(target_arrival_time) or target_arrival_time <= 0.0:
        raise ValueError("target_arrival_time must be positive")
    if (
        not math.isfinite(trigger_ratio)
        or trigger_ratio <= 0.0
        or trigger_ratio > 1.0
    ):
        raise ValueError("trigger_ratio must be in the (0, 1] interval")

    # mission_manager가 ETA 오름차순으로 정렬해서 넘긴다.
    # 따라서 candidates[0]은 항상 현재 가장 빠른 후보이다.
    candidates = list(ranked_candidates)
    if not all(
        robot_id and math.isfinite(eta) and eta >= 0.0
        for robot_id, eta in candidates
    ):
        raise ValueError("candidate IDs and ETAs must be valid")
    if not candidates:
        return []
    # 예: 목표 30초 * 0.85 = 25.5초. 가장 빠른 로봇도 이 값 이상이면
    # deadline miss 위험으로 보고 상위 두 로봇을 동시에 출동시킨다.
    trigger_eta = target_arrival_time * trigger_ratio
    if (
        dual_dispatch_enabled
        and len(candidates) >= 2
        and candidates[0][1] >= trigger_eta
    ):
        return [robot_id for robot_id, _ in candidates[:2]]
    return [candidates[0][0]]


def should_switch_for_live_eta(
    current_eta: float,
    replacement_eta: float,
    *,
    minimum_gain: float,
    switch_ratio: float,
) -> bool:
    """Return whether a fresh standby ETA safely beats the active robot."""
    # [CODE REVIEW] 절대 이득(minimum_gain)과 상대 비율(switch_ratio)을 둘 다 만족해야
    # 교체해서 작은 ETA 흔들림 때문에 로봇 선택이 계속 바뀌는 것을 막는다.
    if not math.isfinite(replacement_eta) or replacement_eta < 0.0:
        return False
    if math.isinf(current_eta) and current_eta > 0.0:
        return True
    if not math.isfinite(current_eta) or current_eta < 0.0:
        return False
    if not math.isfinite(minimum_gain) or minimum_gain < 0.0:
        raise ValueError("minimum_gain must be finite and non-negative")
    if not math.isfinite(switch_ratio) or not 0.0 < switch_ratio <= 1.0:
        raise ValueError("switch_ratio must be in (0, 1]")
    # 두 조건을 동시에 만족해야 standby 로봇으로 교체한다.
    # 1) replacement가 minimum_gain 이상 절대적으로 빠를 것
    # 2) replacement가 current_eta * switch_ratio 이하로 충분히 빠를 것
    # 이렇게 해야 ETA가 0.x초 흔들릴 때마다 로봇 선택이 계속 바뀌지 않는다.
    return (
        replacement_eta + minimum_gain <= current_eta
        and replacement_eta <= current_eta * switch_ratio
    )


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the [-pi, pi] interval."""
    return math.atan2(math.sin(angle), math.cos(angle))


def path_length(points: Iterable[Position]) -> float:
    """Return the accumulated planar length of a path."""
    point_list = list(points)
    if not all(
        math.isfinite(value) for point in point_list for value in point
    ):
        raise ValueError("path points must contain only finite values")
    return sum(
        math.hypot(
            current[0] - previous[0], current[1] - previous[1]
        )
        for previous, current in zip(point_list, point_list[1:])
    )


def point_in_polygon(point: Position, polygon: Iterable[Position]) -> bool:
    """Return whether a point lies inside or on a polygon boundary."""
    x, y = point
    vertices = list(polygon)
    if len(vertices) < 3:
        raise ValueError("polygon must contain at least three points")
    if not all(
        math.isfinite(value) for vertex in vertices for value in vertex
    ):
        raise ValueError("polygon must contain only finite values")

    inside = False
    for index, first in enumerate(vertices):
        second = vertices[(index + 1) % len(vertices)]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        cross = (x - first[0]) * dy - (y - first[1]) * dx
        if abs(cross) <= 1e-9:
            dot = (x - first[0]) * dx + (y - first[1]) * dy
            if -1e-9 <= dot <= dx * dx + dy * dy + 1e-9:
                return True
        if (first[1] > y) != (second[1] > y):
            intersection = (
                (second[0] - first[0])
                * (y - first[1])
                / (second[1] - first[1])
                + first[0]
            )
            if x < intersection:
                inside = not inside
    return inside


def point_to_polygon_distance(
    point: Position, polygon: Iterable[Position]
) -> float:
    """Return zero inside a polygon, otherwise distance to its boundary."""
    vertices = list(polygon)
    if point_in_polygon(point, vertices):
        return 0.0
    x, y = point
    result = math.inf
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        squared = dx * dx + dy * dy
        if squared <= 1e-18:
            nearest_x, nearest_y = start
        else:
            ratio = max(
                0.0,
                min(
                    1.0,
                    ((x - start[0]) * dx + (y - start[1]) * dy)
                    / squared,
                ),
            )
            nearest_x = start[0] + ratio * dx
            nearest_y = start[1] + ratio * dy
        result = min(result, math.hypot(x - nearest_x, y - nearest_y))
    return result


def path_length_in_polygon(
    points: Iterable[Position], polygon: Iterable[Position]
) -> float:
    """Approximate path length in a polygon using dense-path midpoints."""
    point_list = list(points)
    polygon_list = list(polygon)
    path_length(point_list)
    point_in_polygon((0.0, 0.0), polygon_list)
    result = 0.0
    for start, end in zip(point_list, point_list[1:]):
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
        midpoint = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5)
        if point_in_polygon(midpoint, polygon_list):
            result += segment_length
    return result


def crowd_delay_seconds(
    crowded_distance: float,
    *,
    normal_speed: float,
    crowded_speed: float,
) -> float:
    """Return only the extra time beyond the base normal-speed cost."""
    # [CODE REVIEW] base ETA에 정상속도 이동시간이 이미 있으므로 추가분만 더한다.
    # delay = d_zone * (1 / v_crowd - 1 / v_normal)
    values = (crowded_distance, normal_speed, crowded_speed)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("crowd delay inputs must be finite")
    if crowded_distance < 0.0:
        raise ValueError("crowded_distance must be non-negative")
    if normal_speed <= 0.0 or crowded_speed <= 0.0:
        raise ValueError("crowd speeds must be positive")
    if crowded_speed > normal_speed:
        raise ValueError("crowded_speed must not exceed normal_speed")
    return crowded_distance * (1.0 / crowded_speed - 1.0 / normal_speed)


def simplify_path(
    points: Iterable[Position], tolerance: float
) -> list[Position]:
    """Reduce grid-scale path jitter with Ramer-Douglas-Peucker."""
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError(
            "simplification_tolerance must be finite and non-negative"
        )
    point_list = list(points)
    path_length(point_list)
    if len(point_list) <= 2 or tolerance == 0.0:
        return point_list

    start = point_list[0]
    end = point_list[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    segment_length = math.hypot(dx, dy)
    if segment_length <= 1e-12:
        distances = [
            math.hypot(point[0] - start[0], point[1] - start[1])
            for point in point_list[1:-1]
        ]
    else:
        distances = [
            abs(dy * (point[0] - start[0]) - dx * (point[1] - start[1]))
            / segment_length
            for point in point_list[1:-1]
        ]
    if not distances:
        return [start, end]
    maximum = max(distances)
    if maximum <= tolerance:
        return [start, end]
    split = distances.index(maximum) + 1
    left = simplify_path(point_list[: split + 1], tolerance)
    right = simplify_path(point_list[split:], tolerance)
    return left[:-1] + right


def path_motion_cost(
    points: Iterable[Position],
    *,
    linear_speed: float,
    angular_speed: float,
    slowdown_turn_threshold: float,
    slowdown_penalty: float,
    simplification_tolerance: float = 0.0,
    initial_yaw: float | None = None,
    final_yaw: float | None = None,
) -> tuple[float, float, int]:
    """Estimate travel time from distance, turns, and corner slowdowns.

    Returns ``(estimated_seconds, total_turn_radians, slowdown_count)``.
    """
    # [CODE REVIEW] ETA = 거리/선속도 + 총 회전각/각속도 + 큰 코너 수*slowdown penalty.
    # GridBased path의 작은 지터가 회전 비용으로 과대계산되지 않도록 먼저 단순화한다.
    if not math.isfinite(linear_speed) or linear_speed <= 0.0:
        raise ValueError("linear_speed must be positive")
    if not math.isfinite(angular_speed) or angular_speed <= 0.0:
        raise ValueError("angular_speed must be positive")
    if (
        not math.isfinite(slowdown_turn_threshold)
        or slowdown_turn_threshold < 0.0
    ):
        raise ValueError("slowdown_turn_threshold must be non-negative")
    if not math.isfinite(slowdown_penalty) or slowdown_penalty < 0.0:
        raise ValueError("slowdown_penalty must be non-negative")
    if initial_yaw is not None and not math.isfinite(initial_yaw):
        raise ValueError("initial_yaw must be finite")
    if final_yaw is not None and not math.isfinite(final_yaw):
        raise ValueError("final_yaw must be finite")

    # 이동거리는 원본 Nav2 path로 계산한다. 경로 단순화는 회전각 계산에만 사용해서
    # 실제 이동거리 자체가 인위적으로 짧아지는 문제를 만들지 않는다.
    point_list = list(points)
    distance = path_length(point_list)
    simplified_points = simplify_path(point_list, simplification_tolerance)
    headings: list[float] = []
    for previous, current in zip(
        simplified_points, simplified_points[1:]
    ):
        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        if math.hypot(dx, dy) <= 1e-9:
            continue
        # 각 유효 선분이 향하는 방향(yaw)을 저장한다.
        headings.append(math.atan2(dy, dx))

    # 연속 선분 heading 차이가 실제 코너에서 회전해야 하는 각도다.
    internal_turn_angles = [
        abs(normalize_angle(current - previous))
        for previous, current in zip(headings, headings[1:])
    ]
    turn_angles = list(internal_turn_angles)
    if headings and initial_yaw is not None:
        turn_angles.insert(
            0, abs(normalize_angle(headings[0] - initial_yaw))
        )
    if headings and final_yaw is not None:
        turn_angles.append(abs(normalize_angle(final_yaw - headings[-1])))
    if not headings and initial_yaw is not None and final_yaw is not None:
        turn_angles.append(abs(normalize_angle(final_yaw - initial_yaw)))
    total_turn = sum(turn_angles)
    slowdown_count = sum(
        1
        for angle in internal_turn_angles
        if angle >= slowdown_turn_threshold
    )
    estimated_seconds = (
        distance / linear_speed
        + total_turn / angular_speed
        + slowdown_count * slowdown_penalty
    )
    return estimated_seconds, total_turn, slowdown_count
