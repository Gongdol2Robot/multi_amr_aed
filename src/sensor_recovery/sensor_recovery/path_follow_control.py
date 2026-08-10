"""Pure control-loop math for the LiDAR-fallback path follower.

Kept free of rclpy (only numpy for depth arrays) so it can be unit tested
without a running node, mirroring lidar_state_machine.py.

[CODE REVIEW]
경로 진행률, cmd_vel, odom 누적 위치, depth 안전 판정을 ROS callback에서 분리했다.
fallback_path_follower는 이 순수 함수들을 조합하고 실제 Topic I/O만 담당한다.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

ANGULAR_GAIN = 1.5
LINEAR_GAIN = 0.5
# Above this heading error, drive angle-only (no forward motion) instead of
# letting cos(heading_error) taper it — avoids near-90deg creeping and never
# lets it go negative (which would otherwise need an extra clamp).
MAX_HEADING_FOR_LINEAR = math.radians(60.0)

Point2D = Tuple[float, float]
Pose2D = Tuple[float, float, float]  # x, y, yaw


@dataclass(frozen=True)
class PathProgress:
    """One stable path-progress update for a control tick."""

    closest_index: int
    target_index: int
    target_point: Point2D
    closest_distance_m: float
    reacquired: bool
    search_start_index: int
    search_end_index: int


class DepthSafetyResult(str, Enum):
    CLEAR = "CLEAR"
    OBSTACLE = "OBSTACLE"
    NOISY_DEPTH = "NOISY_DEPTH"
    INSUFFICIENT_DATA = "INSUFFICIENT_DATA"


# Worst-first: how several ROIs should be prioritized when they disagree.
# Whether unavailable data blocks motion is a separate, explicit policy in
# depth_result_blocks_motion().
_DEPTH_SEVERITY = {
    DepthSafetyResult.OBSTACLE: 4,
    DepthSafetyResult.NOISY_DEPTH: 3,
    DepthSafetyResult.INSUFFICIENT_DATA: 1,
    DepthSafetyResult.CLEAR: 0,
}


def depth_result_blocks_motion(
    result: DepthSafetyResult,
    allow_insufficient: bool = False,
) -> bool:
    """Return whether a depth verdict must stop fallback motion.

    Real obstacle/noise detections always fail closed. INSUFFICIENT_DATA can
    be configured fail-open for the known unreliable robot network.
    """
    if result in (DepthSafetyResult.OBSTACLE, DepthSafetyResult.NOISY_DEPTH):
        return True
    if result == DepthSafetyResult.INSUFFICIENT_DATA:
        return not allow_insufficient
    return False


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the range from negative pi to positive pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def find_closest_index(
    path_points: List[Point2D],
    x: float,
    y: float,
    min_index: int = 0,
    max_index: Optional[int] = None,
) -> int:
    """Closest path point inside an inclusive index window.

    The caller decides how much backward/forward search is safe. Limiting
    the upper bound prevents a U-shaped path from snapping to a spatially
    close but much later path branch.
    """
    if not path_points:
        return 0
    min_index = max(0, min(min_index, len(path_points) - 1))
    if max_index is None:
        max_index = len(path_points) - 1
    max_index = max(min_index, min(max_index, len(path_points) - 1))
    best_index = min_index
    best_dist = math.hypot(path_points[min_index][0] - x, path_points[min_index][1] - y)
    for i in range(min_index + 1, max_index + 1):
        dist = math.hypot(path_points[i][0] - x, path_points[i][1] - y)
        if dist < best_dist:
            best_dist = dist
            best_index = i
    return best_index


def remaining_path_from_pose(
    path_points: List[Point2D], x: float, y: float
) -> List[Point2D]:
    """Trim an active Nav2 path to the portion starting at the current pose.

    The fault pose is inserted as the first point so odometry integration and
    path progress share exactly the same anchor.  Nav2 continuously replans,
    therefore its latest global path should already start nearby; selecting
    the closest point also removes any short prefix left behind the robot.
    """
    if not path_points:
        return []
    # [CODE REVIEW] 장애 전 전체 Nav2 경로에서 이미 지나온 prefix를 제거하고,
    # odom 적분 기준과 경로 시작점을 일치시키기 위해 fault pose를 첫 점으로 넣는다.
    closest = find_closest_index(path_points, x, y)
    result = [(float(x), float(y))]
    for point in path_points[closest:]:
        normalized = (float(point[0]), float(point[1]))
        if math.hypot(normalized[0] - result[-1][0], normalized[1] - result[-1][1]) > 1e-6:
            result.append(normalized)
    return result


def _index_at_path_distance(
    path_points: List[Point2D], start_index: int, distance_m: float, direction: int
) -> int:
    """Walk an index forward/backward until cumulative path distance is met."""
    index = max(0, min(start_index, len(path_points) - 1))
    if distance_m <= 0.0:
        return index
    accumulated = 0.0
    while 0 <= index + direction < len(path_points):
        next_index = index + direction
        accumulated += math.hypot(
            path_points[next_index][0] - path_points[index][0],
            path_points[next_index][1] - path_points[index][1],
        )
        index = next_index
        if accumulated >= distance_m:
            break
    return index


def update_path_progress(
    path_points: List[Point2D],
    x: float,
    y: float,
    previous_closest_index: int,
    previous_target_index: int,
    lookahead_m: float,
    search_ahead_m: float,
    search_backtrack_m: float,
    reacquire_distance_m: float,
) -> PathProgress:
    """Update closest/target indices without jumping across nearby branches.

    Normal progress is monotonic. A small bounded backward search exists so
    localization noise or mild path departure can be recovered, but a
    backward index is accepted only when the previous progress point is
    farther than ``reacquire_distance_m`` and the candidate is closer.
    """
    # [CODE REVIEW] U자형 경로처럼 공간상 가까운 다른 구간으로 index가 순간이동하지 않도록
    # 이전 index 주변의 제한된 거리만 검색하고, 정상 진행 중에는 index를 단조 증가시킨다.
    if not path_points:
        raise ValueError("path_points must not be empty")

    previous_closest_index = max(
        0, min(previous_closest_index, len(path_points) - 1)
    )
    previous_target_index = max(
        previous_closest_index,
        min(previous_target_index, len(path_points) - 1),
    )
    search_start = _index_at_path_distance(
        path_points, previous_closest_index, search_backtrack_m, -1
    )
    search_end = _index_at_path_distance(
        path_points, previous_closest_index, search_ahead_m, 1
    )
    candidate = find_closest_index(
        path_points, x, y, min_index=search_start, max_index=search_end
    )
    previous_distance = math.hypot(
        path_points[previous_closest_index][0] - x,
        path_points[previous_closest_index][1] - y,
    )
    candidate_distance = math.hypot(
        path_points[candidate][0] - x,
        path_points[candidate][1] - y,
    )
    reacquired = (
        candidate < previous_closest_index
        and previous_distance > reacquire_distance_m
        and candidate_distance < previous_distance
    )
    if candidate < previous_closest_index and not reacquired:
        candidate = previous_closest_index
        candidate_distance = previous_distance

    target_point, target_index = select_lookahead_target(
        path_points, candidate, lookahead_m
    )
    if not reacquired and target_index < previous_target_index:
        target_index = previous_target_index
        target_point = path_points[target_index]

    return PathProgress(
        closest_index=candidate,
        target_index=target_index,
        target_point=target_point,
        closest_distance_m=candidate_distance,
        reacquired=reacquired,
        search_start_index=search_start,
        search_end_index=search_end,
    )


def select_lookahead_target(
    path_points: List[Point2D], closest_index: int, lookahead_m: float
) -> Tuple[Point2D, int]:
    """Walk forward from closest_index by cumulative path distance to lookahead_m.

    Always returns a target (the final point if the path is shorter than
    lookahead_m from here) — arrival is decided separately by the caller
    comparing current position to the last point.
    """
    closest_index = max(0, min(closest_index, len(path_points) - 1))
    accumulated = 0.0
    for i in range(closest_index, len(path_points) - 1):
        x0, y0 = path_points[i]
        x1, y1 = path_points[i + 1]
        segment = math.hypot(x1 - x0, y1 - y0)
        if accumulated + segment >= lookahead_m:
            return path_points[i + 1], i + 1
        accumulated += segment
    return path_points[-1], len(path_points) - 1


def _point_segment_distance(x: float, y: float, a: Point2D, b: Point2D) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq))
    proj_x, proj_y = ax + t * dx, ay + t * dy
    return math.hypot(x - proj_x, y - proj_y)


def path_deviation_m(
    path_points: List[Point2D],
    x: float,
    y: float,
    min_index: int = 0,
    max_index: Optional[int] = None,
) -> float:
    """Cross-track distance to an inclusive path-point index window."""
    if not path_points:
        return 0.0
    min_index = max(0, min(min_index, len(path_points) - 1))
    if max_index is None:
        max_index = len(path_points) - 1
    max_index = max(min_index, min(max_index, len(path_points) - 1))
    if min_index == max_index:
        return math.hypot(
            path_points[min_index][0] - x, path_points[min_index][1] - y
        )
    return min(
        _point_segment_distance(x, y, path_points[i], path_points[i + 1])
        for i in range(min_index, max_index)
    )


def goal_reached(
    path_points: List[Point2D], x: float, y: float, tolerance_m: float
) -> bool:
    """Whether the current position is within tolerance of the final goal."""
    if not path_points:
        return False
    goal_x, goal_y = path_points[-1]
    return math.hypot(goal_x - x, goal_y - y) <= tolerance_m


def heading_error_to_target(
    current_x: float,
    current_y: float,
    current_yaw: float,
    target_x: float,
    target_y: float,
) -> float:
    """Normalized heading error from the current pose to a target point."""
    return normalize_angle(
        math.atan2(target_y - current_y, target_x - current_x) - current_yaw
    )


def compute_cmd_vel(
    current_x: float,
    current_y: float,
    current_yaw: float,
    target_x: float,
    target_y: float,
    max_linear: float,
    max_angular: float,
    linear_heading_threshold_rad: float = MAX_HEADING_FOR_LINEAR,
) -> Tuple[float, float]:
    """Proportional heading + distance controller toward a single target point."""
    # [CODE REVIEW] 목표 방향 오차가 크면 제자리 회전만 하고,
    # 방향이 맞은 뒤에만 전진시켜 벽 근처에서 옆으로 기어가는 동작을 막는다.
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.hypot(dx, dy)
    heading_error = heading_error_to_target(
        current_x, current_y, current_yaw, target_x, target_y
    )

    angular_z = max(-max_angular, min(max_angular, heading_error * ANGULAR_GAIN))
    if abs(heading_error) > linear_heading_threshold_rad:
        # Target is well off to the side or behind: rotate in place, don't
        # creep forward (and never drive backward toward it).
        linear_x = 0.0
    else:
        linear_scale = max(0.0, math.cos(heading_error))
        linear_x = max(0.0, min(max_linear, distance * LINEAR_GAIN)) * linear_scale
    return linear_x, angular_z


def rate_limit(previous: float, target: float, max_delta: float) -> float:
    """Clamp target so it moves at most max_delta away from previous this tick."""
    if max_delta <= 0.0:
        return target
    delta = max(-max_delta, min(max_delta, target - previous))
    return previous + delta


def integrate_odom_delta(
    anchor: Pose2D,
    odom_start: Pose2D,
    odom_now: Pose2D,
    translation_scale: float = 1.0,
    translation_heading_correction_rad: float = 0.0,
    yaw_delta_scale: float = 1.0,
) -> Pose2D:
    """Compose the odom-frame motion since odom_start onto the anchor pose.

    odom_start/odom_now share an arbitrary odom-frame origin; only their
    relative motion is meaningful, so it's expressed in the robot's frame at
    odom_start and then re-applied on top of anchor (the last AMCL pose).
    Optional calibration is applied to that relative motion, leaving the
    anchor unchanged and growing gradually with traveled distance/rotation.
    """
    # [CODE REVIEW] LiDAR 장애 뒤 AMCL을 계속 신뢰하지 않고 마지막 map pose를 anchor로 둔다.
    # 이후 odom의 상대 이동량만 anchor에 합성해 fallback 중 map 기준 위치를 추정한다.
    anchor_x, anchor_y, anchor_yaw = anchor
    start_x, start_y, start_yaw = odom_start
    now_x, now_y, now_yaw = odom_now

    raw_dx = now_x - start_x
    raw_dy = now_y - start_y
    local_dx = raw_dx * math.cos(start_yaw) + raw_dy * math.sin(start_yaw)
    local_dy = -raw_dx * math.sin(start_yaw) + raw_dy * math.cos(start_yaw)
    correction_cos = math.cos(translation_heading_correction_rad)
    correction_sin = math.sin(translation_heading_correction_rad)
    corrected_local_dx = translation_scale * (
        local_dx * correction_cos - local_dy * correction_sin
    )
    corrected_local_dy = translation_scale * (
        local_dx * correction_sin + local_dy * correction_cos
    )
    dyaw = normalize_angle(now_yaw - start_yaw) * yaw_delta_scale

    new_x = (
        anchor_x
        + corrected_local_dx * math.cos(anchor_yaw)
        - corrected_local_dy * math.sin(anchor_yaw)
    )
    new_y = (
        anchor_y
        + corrected_local_dx * math.sin(anchor_yaw)
        + corrected_local_dy * math.cos(anchor_yaw)
    )
    new_yaw = normalize_angle(anchor_yaw + dyaw)
    return (new_x, new_y, new_yaw)


def is_stale(last_time: Optional[float], now: float, timeout_sec: float) -> bool:
    """True if last_time is missing or older than timeout_sec relative to now."""
    if last_time is None:
        return True
    return (now - last_time) > timeout_sec


def time_regressed(previous_time: Optional[float], new_time: float) -> bool:
    """True if new_time is earlier than previous_time (clock jump / odom reset).

    [CODE REVIEW]
    시간 역행을 별도로 차단하는 대안 판정을 위해 만들었지만 현재 운영 제어
    경로에서는 호출하지 않는다. 관련 순수 함수 단위 테스트용으로만 보존한다.
    """
    if previous_time is None:
        return False
    return new_time < previous_time


def evaluate_depth_safety(
    depth_image_mm: Optional[np.ndarray],
    min_obstacle_distance_m: float,
    obstacle_pixel_ratio: float,
    min_valid_pixel_ratio: float,
    roi: Optional[Tuple[int, int, int, int]] = None,
    noise_valid_pixel_ratio: Optional[float] = None,
) -> DepthSafetyResult:
    """Classify one ROI using pixel-count thresholds instead of a single min.

    A single noisy near-zero pixel can no longer flip the result: an
    obstacle is only reported when at least obstacle_pixel_ratio of the ROI
    is closer than min_obstacle_distance_m, and the ROI must have at least
    min_valid_pixel_ratio real (non-zero, finite) pixels to be trusted at all.
    A separate, higher noise_valid_pixel_ratio marks the partially collapsed
    stereo pattern seen inside the camera's useful minimum range.  A real
    close obstacle takes precedence over that diagnostic label.
    """
    # [CODE REVIEW] 단일 최소 depth 픽셀은 노이즈에 취약하므로 ROI 안의
    # 유효 픽셀 비율과 근거리 픽셀 비율을 함께 사용해 정지 여부를 결정한다.
    if depth_image_mm is None:
        return DepthSafetyResult.INSUFFICIENT_DATA

    region = depth_image_mm if roi is None else depth_image_mm[roi[1]:roi[3], roi[0]:roi[2]]
    if region.size == 0:
        return DepthSafetyResult.INSUFFICIENT_DATA

    valid_mask = (region > 0) & np.isfinite(region)
    valid_ratio = np.count_nonzero(valid_mask) / region.size
    if valid_ratio < min_valid_pixel_ratio:
        return DepthSafetyResult.INSUFFICIENT_DATA

    close_mask = valid_mask & (region < (min_obstacle_distance_m * 1000.0))
    close_ratio = np.count_nonzero(close_mask) / region.size
    if close_ratio >= obstacle_pixel_ratio:
        return DepthSafetyResult.OBSTACLE
    if (
        noise_valid_pixel_ratio is not None
        and valid_ratio < noise_valid_pixel_ratio
    ):
        return DepthSafetyResult.NOISY_DEPTH
    return DepthSafetyResult.CLEAR


def worst_depth_result(results: List[DepthSafetyResult]) -> DepthSafetyResult:
    """Combine several ROI verdicts into the single most severe one."""
    if not results:
        return DepthSafetyResult.INSUFFICIENT_DATA
    return max(results, key=lambda r: _DEPTH_SEVERITY[r])


def pose_error(pose_a: Pose2D, pose_b: Pose2D) -> Tuple[float, float]:
    """Distance (m) and absolute heading difference (deg) between two poses."""
    ax, ay, ayaw = pose_a
    bx, by, byaw = pose_b
    distance = math.hypot(bx - ax, by - ay)
    angle_deg = math.degrees(abs(normalize_angle(byaw - ayaw)))
    return distance, angle_deg
