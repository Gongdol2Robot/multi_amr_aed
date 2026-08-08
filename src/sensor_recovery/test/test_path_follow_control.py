import math

import numpy as np

from sensor_recovery.path_follow_control import (
    DepthSafetyResult,
    compute_cmd_vel,
    evaluate_depth_safety,
    find_closest_index,
    goal_reached,
    integrate_odom_delta,
    is_stale,
    normalize_angle,
    path_deviation_m,
    pose_error,
    rate_limit,
    remaining_path_from_pose,
    select_lookahead_target,
    time_regressed,
    update_path_progress,
    worst_depth_result,
)


def test_normalize_angle_wraps_to_pi_range():
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-9) or math.isclose(
        normalize_angle(3 * math.pi), -math.pi, abs_tol=1e-9
    )
    assert math.isclose(normalize_angle(0.0), 0.0, abs_tol=1e-9)
    assert math.isclose(normalize_angle(-3 * math.pi), math.pi, abs_tol=1e-9) or math.isclose(
        normalize_angle(-3 * math.pi), -math.pi, abs_tol=1e-9
    )


# --- find_closest_index / select_lookahead_target -------------------------


def test_find_closest_index_picks_nearest_point():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert find_closest_index(path, 0.9, 0.0) == 1


def test_find_closest_index_never_looks_before_min_index():
    # A looping path revisits (0, 0)-ish territory later at index 3. Without
    # the min_index floor, the naive nearest point would snap back to index
    # 0 (also near the query); min_index=2 must keep it forward at index 3.
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (0.1, 0.0)]
    assert find_closest_index(path, 0.05, 0.0, min_index=2) == 3
    # Sanity check: without the floor, the unrestricted search would pick
    # the earlier, equally-close point instead.
    assert find_closest_index(path, 0.05, 0.0, min_index=0) == 0


def test_find_closest_index_empty_path():
    assert find_closest_index([], 1.0, 1.0) == 0


def test_find_closest_index_respects_forward_window():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (0.1, 0.1)]
    assert find_closest_index(path, 0.1, 0.1, min_index=1, max_index=2) == 1


def test_remaining_path_starts_at_fault_pose_and_removes_passed_prefix():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    assert remaining_path_from_pose(path, 1.1, 0.1) == [
        (1.1, 0.1),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
    ]


def test_remaining_path_empty_input_stays_empty():
    assert remaining_path_from_pose([], 1.0, 2.0) == []


def test_select_lookahead_target_walks_forward_by_cumulative_distance():
    path = [(0.0, 0.0), (0.1, 0.0), (0.4, 0.0), (1.0, 0.0)]
    target, index = select_lookahead_target(path, closest_index=0, lookahead_m=0.3)
    assert target == (0.4, 0.0)
    assert index == 2


def test_select_lookahead_target_clamped_to_final_point():
    path = [(0.0, 0.0), (0.05, 0.0)]
    target, index = select_lookahead_target(path, closest_index=0, lookahead_m=5.0)
    assert target == (0.05, 0.0)
    assert index == 1


def test_select_lookahead_target_index_is_monotonic_with_closest_index():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    _, index = select_lookahead_target(path, closest_index=2, lookahead_m=0.1)
    assert index >= 2


def _progress(path, x, y, closest=0, target=0, **overrides):
    values = dict(
        lookahead_m=0.2,
        search_ahead_m=0.6,
        search_backtrack_m=0.2,
        reacquire_distance_m=0.5,
    )
    values.update(overrides)
    return update_path_progress(path, x, y, closest, target, **values)


def test_path_progress_indices_advance_on_straight_path():
    path = [(index * 0.1, 0.0) for index in range(11)]
    first = _progress(path, 0.21, 0.0)
    second = _progress(
        path,
        0.42,
        0.0,
        closest=first.closest_index,
        target=first.target_index,
    )
    assert first.closest_index == 2
    assert second.closest_index == 4
    assert second.target_index >= first.target_index


def test_path_progress_does_not_reselect_passed_point_for_small_noise():
    path = [(index * 0.1, 0.0) for index in range(11)]
    progress = _progress(
        path,
        0.38,
        0.02,
        closest=4,
        target=6,
        reacquire_distance_m=0.5,
    )
    assert progress.closest_index >= 4
    assert progress.target_index >= 6
    assert progress.reacquired is False


def test_zero_backtrack_window_never_searches_previous_index():
    path = [(index * 0.1, 0.0) for index in range(6)]
    progress = _progress(
        path,
        0.29,
        0.0,
        closest=3,
        target=4,
        search_backtrack_m=0.0,
    )
    assert progress.search_start_index == 3
    assert progress.closest_index >= 3


def test_path_progress_does_not_jump_to_nearby_later_u_turn_branch():
    lower = [(float(index), 0.0) for index in range(11)]
    upper = [(float(index), 0.2) for index in range(10, -1, -1)]
    path = lower + upper
    progress = _progress(
        path,
        2.0,
        0.1,
        closest=2,
        target=3,
        search_ahead_m=2.0,
    )
    assert progress.closest_index == 2
    assert progress.closest_index < len(lower)


def test_path_progress_can_reacquire_a_bounded_earlier_segment():
    path = [(index * 0.1, 0.0) for index in range(11)]
    progress = _progress(
        path,
        0.21,
        0.0,
        closest=6,
        target=8,
        search_backtrack_m=0.5,
        reacquire_distance_m=0.2,
    )
    assert progress.closest_index == 2
    assert progress.reacquired is True


def test_path_progress_selects_last_point_near_goal():
    path = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0)]
    progress = _progress(path, 0.3, 0.0, closest=1, target=1)
    assert progress.target_point == path[-1]
    assert progress.target_index == len(path) - 1


def test_goal_reached_uses_final_point_and_tolerance():
    path = [(0.0, 0.0), (1.0, 0.0)]
    assert goal_reached(path, 0.91, 0.0, 0.1)
    assert not goal_reached(path, 0.89, 0.0, 0.1)
    assert not goal_reached([], 0.0, 0.0, 0.1)


# --- path_deviation_m -------------------------------------------------


def test_path_deviation_zero_when_on_path():
    path = [(0.0, 0.0), (2.0, 0.0)]
    assert math.isclose(path_deviation_m(path, 1.0, 0.0), 0.0, abs_tol=1e-9)


def test_path_deviation_perpendicular_distance():
    path = [(0.0, 0.0), (2.0, 0.0)]
    assert math.isclose(path_deviation_m(path, 1.0, 0.5), 0.5, abs_tol=1e-9)


def test_path_deviation_single_point_path():
    assert math.isclose(path_deviation_m([(0.0, 0.0)], 3.0, 4.0), 5.0, abs_tol=1e-9)


def test_path_deviation_empty_path_is_zero():
    assert path_deviation_m([], 5.0, 5.0) == 0.0


def test_path_deviation_can_be_limited_to_progress_window():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 0.2), (0.0, 0.2)]
    whole_path = path_deviation_m(path, 1.0, 0.2)
    active_branch = path_deviation_m(path, 1.0, 0.2, min_index=0, max_index=1)
    assert math.isclose(whole_path, 0.0, abs_tol=1e-9)
    assert math.isclose(active_branch, 0.2, abs_tol=1e-9)


# --- compute_cmd_vel ----------------------------------------------------


def test_compute_cmd_vel_target_straight_ahead():
    linear_x, angular_z = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=1.0, target_y=0.0, max_linear=0.2, max_angular=1.0
    )
    assert linear_x > 0.0
    assert math.isclose(angular_z, 0.0, abs_tol=1e-9)


def test_compute_cmd_vel_hard_zero_past_heading_cutoff():
    # 90 degrees off to the side: well past the 60deg cutoff.
    linear_x, angular_z = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=0.0, target_y=1.0, max_linear=0.2, max_angular=1.0
    )
    assert linear_x == 0.0
    assert angular_z > 0.0


def test_compute_cmd_vel_target_behind_turns_without_driving_forward():
    linear_x, angular_z = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=-1.0, target_y=0.0, max_linear=0.2, max_angular=1.0
    )
    assert linear_x == 0.0
    assert abs(angular_z) == 1.0


def test_compute_cmd_vel_clamps_to_max_linear():
    linear_x, _ = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=100.0, target_y=0.0, max_linear=0.2, max_angular=1.0
    )
    assert linear_x == 0.2


def test_compute_cmd_vel_within_cutoff_still_scales_down():
    linear_x, _ = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=1.0, target_y=1.0, max_linear=0.2, max_angular=10.0
    )
    # 45 degrees off: inside the 60deg cutoff, so some forward motion remains.
    assert 0.0 < linear_x < 0.2


def test_compute_cmd_vel_parameterized_heading_cutoff():
    linear_x, angular_z = compute_cmd_vel(
        0.0,
        0.0,
        0.0,
        target_x=1.0,
        target_y=1.0,
        max_linear=0.2,
        max_angular=1.0,
        linear_heading_threshold_rad=math.radians(30.0),
    )
    assert linear_x == 0.0
    assert angular_z > 0.0


def test_compute_cmd_vel_turns_right_with_negative_angular_speed():
    linear_x, angular_z = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=1.0, target_y=-1.0,
        max_linear=0.2, max_angular=1.0,
    )
    assert linear_x > 0.0
    assert angular_z < 0.0


def test_compute_cmd_vel_slows_near_goal():
    far_linear, _ = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=1.0, target_y=0.0,
        max_linear=1.0, max_angular=1.0,
    )
    near_linear, _ = compute_cmd_vel(
        0.0, 0.0, 0.0, target_x=0.1, target_y=0.0,
        max_linear=1.0, max_angular=1.0,
    )
    assert 0.0 < near_linear < far_linear


# --- rate_limit -----------------------------------------------------------


def test_rate_limit_allows_small_change():
    assert math.isclose(rate_limit(0.0, 0.05, max_delta=0.1), 0.05, abs_tol=1e-9)


def test_rate_limit_caps_large_increase():
    assert math.isclose(rate_limit(0.0, 1.0, max_delta=0.1), 0.1, abs_tol=1e-9)


def test_rate_limit_caps_large_decrease():
    assert math.isclose(rate_limit(0.5, -1.0, max_delta=0.2), 0.3, abs_tol=1e-9)


def test_rate_limit_disabled_when_max_delta_non_positive():
    assert rate_limit(0.0, 5.0, max_delta=0.0) == 5.0


# --- integrate_odom_delta --------------------------------------------------


def test_integrate_odom_delta_no_motion_returns_anchor():
    anchor = (1.0, 2.0, 0.5)
    odom = (0.0, 0.0, 0.0)
    assert integrate_odom_delta(anchor, odom, odom) == anchor


def test_integrate_odom_delta_straight_motion_zero_yaw():
    anchor = (0.0, 0.0, 0.0)
    x, y, yaw = integrate_odom_delta(anchor, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_integrate_odom_delta_rotated_anchor_reprojects_motion():
    anchor = (0.0, 0.0, math.pi / 2)
    x, y, yaw = integrate_odom_delta(anchor, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 1.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9)


def test_integrate_odom_delta_yaw_wraps_across_pi():
    anchor = (0.0, 0.0, math.pi - 0.1)
    x, y, yaw = integrate_odom_delta(anchor, (0.0, 0.0, 0.0), (0.0, 0.0, math.pi))
    # anchor_yaw + dyaw crosses the +-pi boundary; result must stay wrapped.
    assert -math.pi <= yaw <= math.pi
    assert math.isclose(normalize_angle(yaw - (2 * math.pi - 0.1)), 0.0, abs_tol=1e-6)


# --- is_stale / time_regressed ---------------------------------------------


def test_is_stale_true_when_missing():
    assert is_stale(None, now=10.0, timeout_sec=1.0) is True


def test_is_stale_false_within_timeout():
    assert is_stale(9.5, now=10.0, timeout_sec=1.0) is False


def test_is_stale_true_past_timeout():
    assert is_stale(8.0, now=10.0, timeout_sec=1.0) is True


def test_time_regressed_false_when_no_previous():
    assert time_regressed(None, 5.0) is False


def test_time_regressed_true_on_backward_jump():
    assert time_regressed(10.0, 9.0) is True


def test_time_regressed_false_when_moving_forward():
    assert time_regressed(9.0, 10.0) is False


# --- evaluate_depth_safety / worst_depth_result -----------------------------


def _kwargs(**overrides):
    base = dict(
        depth_timeout_sec=0.5,
        min_obstacle_distance_m=0.5,
        obstacle_pixel_ratio=0.03,
        min_valid_pixel_ratio=0.20,
        noise_valid_pixel_ratio=0.60,
        roi=None,
    )
    base.update(overrides)
    return base


def test_evaluate_depth_safety_missing_image_is_insufficient():
    result = evaluate_depth_safety(None, None, now=1.0, **_kwargs())
    assert result == DepthSafetyResult.INSUFFICIENT_DATA


def test_evaluate_depth_safety_stale_timestamp():
    depth = np.full((10, 10), 2000.0, dtype=np.float32)
    result = evaluate_depth_safety(depth, image_timestamp=0.0, now=1.0, **_kwargs())
    assert result == DepthSafetyResult.STALE


def test_evaluate_depth_safety_mostly_invalid_pixels_is_insufficient():
    depth = np.zeros((10, 10), dtype=np.float32)  # all invalid (zero)
    result = evaluate_depth_safety(depth, image_timestamp=1.0, now=1.1, **_kwargs())
    assert result == DepthSafetyResult.INSUFFICIENT_DATA


def test_evaluate_depth_safety_partially_collapsed_pixels_are_noisy():
    depth = np.zeros((10, 10), dtype=np.float32)
    depth[:5, :] = 2000.0  # 50% valid: measurable, but matches noisy close-range video
    result = evaluate_depth_safety(depth, image_timestamp=1.0, now=1.1, **_kwargs())
    assert result == DepthSafetyResult.NOISY_DEPTH


def test_evaluate_depth_safety_single_noisy_pixel_does_not_trigger_obstacle():
    depth = np.full((10, 10), 2000.0, dtype=np.float32)
    depth[0, 0] = 50.0  # one stray close pixel out of 100 (1%) < 3% threshold
    result = evaluate_depth_safety(depth, image_timestamp=1.0, now=1.1, **_kwargs())
    assert result == DepthSafetyResult.CLEAR


def test_evaluate_depth_safety_obstacle_when_enough_pixels_close():
    depth = np.full((10, 10), 2000.0, dtype=np.float32)
    depth[0:1, 0:5] = 200.0  # 5 of 100 pixels close = 5% >= 3% threshold
    result = evaluate_depth_safety(depth, image_timestamp=1.0, now=1.1, **_kwargs())
    assert result == DepthSafetyResult.OBSTACLE


def test_evaluate_depth_safety_camera_floor_is_obstacle_before_noise():
    depth = np.zeros((10, 10), dtype=np.float32)
    depth[:5, :] = 629.0
    result = evaluate_depth_safety(
        depth,
        image_timestamp=1.0,
        now=1.1,
        **_kwargs(min_obstacle_distance_m=0.65),
    )
    assert result == DepthSafetyResult.OBSTACLE


def test_evaluate_depth_safety_respects_roi_box():
    depth = np.full((10, 10), 2000.0, dtype=np.float32)
    depth[0, 0] = 100.0  # outside the roi below
    result = evaluate_depth_safety(
        depth, image_timestamp=1.0, now=1.1, **_kwargs(roi=(5, 5, 10, 10))
    )
    assert result == DepthSafetyResult.CLEAR


def test_worst_depth_result_obstacle_wins():
    results = [DepthSafetyResult.CLEAR, DepthSafetyResult.OBSTACLE, DepthSafetyResult.STALE]
    assert worst_depth_result(results) == DepthSafetyResult.OBSTACLE


def test_worst_depth_result_empty_is_insufficient_data():
    assert worst_depth_result([]) == DepthSafetyResult.INSUFFICIENT_DATA


def test_worst_depth_result_all_clear():
    assert worst_depth_result([DepthSafetyResult.CLEAR, DepthSafetyResult.CLEAR]) == (
        DepthSafetyResult.CLEAR
    )


# --- pose_error --------------------------------------------------------


def test_pose_error_identical_poses():
    pose = (1.0, 2.0, 0.3)
    distance, angle_deg = pose_error(pose, pose)
    assert math.isclose(distance, 0.0, abs_tol=1e-9)
    assert math.isclose(angle_deg, 0.0, abs_tol=1e-9)


def test_pose_error_known_offset():
    pose_a = (0.0, 0.0, 0.0)
    pose_b = (3.0, 4.0, math.pi / 2)
    distance, angle_deg = pose_error(pose_a, pose_b)
    assert math.isclose(distance, 5.0, abs_tol=1e-9)
    assert math.isclose(angle_deg, 90.0, abs_tol=1e-6)
