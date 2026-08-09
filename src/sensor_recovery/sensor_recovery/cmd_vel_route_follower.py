"""Follow a recorded map route using only odometry feedback and cmd_vel."""

import math
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
import yaml
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

from sensor_recovery.depth_decode import decode_compressed_depth
from sensor_recovery.depth_metrics import (
    compute_depth_region_metrics,
    format_distance,
)
from sensor_recovery.grid_path_planner import (
    OccupancyGridData,
    compute_clearance_field,
    path_segment_is_safe,
)
from sensor_recovery.path_follow_control import (
    DepthSafetyResult,
    Pose2D,
    compute_cmd_vel,
    evaluate_depth_safety,
    heading_error_to_target,
    integrate_odom_delta,
    is_stale,
    normalize_angle,
    path_deviation_m,
    rate_limit,
    update_path_progress,
    worst_depth_result,
)
from sensor_recovery.route_corner_control import (
    build_route_geometry,
    corner_speed_limit,
    heading_after_index,
    interior_corner_indices,
    remaining_path_distance,
    select_target_before_index,
)


_LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


class CmdVelRouteFollower(Node):
    """Track dense route points and stop-turn at detected hard corners."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_route_follower")
        self._declare_parameters()
        self._read_parameters()
        route_data = self._load_route(self.route_file)
        self.start_yaw = math.radians(float(route_data["start_yaw_deg"]))
        goal_yaw = route_data.get("goal_yaw_deg")
        self.goal_yaw = None if goal_yaw is None else math.radians(float(goal_yaw))
        self.geometry = build_route_geometry(
            route_data["points"],
            math.radians(self.corner_angle_threshold_deg),
            self.corner_sample_distance_m,
            self.corner_cluster_distance_m,
        )
        endpoint_guard = max(
            self.arrival_tolerance_m, self.corner_sample_distance_m
        )
        self.corner_indices = interior_corner_indices(
            self.geometry,
            self.corner_sample_distance_m,
            endpoint_guard,
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.last_odom_received: Optional[float] = None
        self.depth_stats_started_at = time.monotonic()
        self.depth_frames_in_window = 0
        self.last_depth_policy_warning_at: Optional[float] = None
        self.depth_clear_since: Optional[float] = None
        self.last_amcl_received_at: Optional[float] = None
        self.last_amcl_update_request: Optional[float] = None
        self.amcl_announced = False
        self.depth_announced = False
        self.last_depth_result = DepthSafetyResult.INSUFFICIENT_DATA
        self.last_depth_diagnostic = "unavailable"
        self.map_valid: Optional[bool] = None
        self.odom_start: Optional[Pose2D] = None
        self.route_anchor: Pose2D = (
            self.geometry.points[0][0],
            self.geometry.points[0][1],
            self.start_yaw,
        )
        self.anchor: Pose2D = self.route_anchor
        self.started_at: Optional[float] = None
        self.closest_index = 0
        self.target_index = 0
        self.corner_cursor = 0
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.last_log_at: Optional[float] = None
        self.latest_estimated_pose: Optional[Pose2D] = None
        self.result_due_at: Optional[float] = None
        self.result_started_at: Optional[float] = None
        self.result_logged = False
        self.resume_state: Optional[str] = None
        self.state = "WAITING"

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(
            String, "cmd_vel_route/state", _LATCHED_QOS
        )
        self.path_pub = self.create_publisher(
            PathMessage, "cmd_vel_route/path", _LATCHED_QOS
        )
        self.target_pub = self.create_publisher(
            PoseStamped, "cmd_vel_route/target", 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, "cmd_vel_route/estimated_pose", 10
        )
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self._on_amcl,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            self.depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(OccupancyGrid, "map", self._on_map, _MAP_QOS)
        self.cancel_client = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )
        self.amcl_update_client = self.create_client(
            Empty, "request_nomotion_update"
        )
        self.create_service(Trigger, "cmd_vel_route/start", self._on_start)
        self.create_service(Trigger, "cmd_vel_route/stop", self._on_stop)
        self.create_timer(self.control_period_sec, self._on_tick)
        self._publish_path()
        self._set_state("WAITING")
        self.get_logger().info(
            f"Loaded route: {len(self.geometry.points)} points, "
            f"length={self.geometry.cumulative_m[-1]:.2f}m, "
            f"hard_corners={list(self.corner_indices)}, "
            f"start_yaw={math.degrees(self.start_yaw):.1f}deg"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "route_file": "",
            "max_linear_speed": 0.07,
            "max_angular_speed": 0.25,
            "max_linear_accel": 0.10,
            "max_angular_accel": 0.40,
            "control_period_sec": 0.05,
            "nav2_stop_delay_sec": 1.0,
            "odom_timeout_sec": 1.0,
            "depth_clear_hold_sec": 0.5,
            "depth_center_roi_size_px": 80,
            "min_obstacle_distance_m": 0.65,
            "obstacle_pixel_ratio": 0.03,
            "min_valid_pixel_ratio": 0.20,
            "noise_valid_pixel_ratio": 0.60,
            "lookahead_m": 0.20,
            "max_path_deviation_m": 0.30,
            "arrival_tolerance_m": 0.06,
            "heading_tolerance_deg": 4.0,
            "linear_heading_threshold_deg": 30.0,
            "corner_angle_threshold_deg": 35.0,
            "corner_sample_distance_m": 0.25,
            "corner_cluster_distance_m": 0.35,
            "corner_slowdown_distance_m": 0.35,
            "corner_position_tolerance_m": 0.06,
            "start_position_tolerance_m": 0.20,
            "start_yaw_tolerance_deg": 20.0,
            "require_nav2_cancel": True,
            "require_static_map": True,
            "stop_on_depth_insufficient_data": True,
            "robot_radius_m": 0.20,
            "hard_margin_m": 0.05,
            "occupied_threshold": 50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("cmd_vel_topic", "cmd_vel_nav")
        self.declare_parameter(
            "depth_topic", "oakd/stereo/image_raw/compressedDepth"
        )

    def _read_parameters(self) -> None:
        self.route_file = str(self.get_parameter("route_file").value)
        if not self.route_file:
            raise ValueError("route_file is required")
        numeric = (
            "max_linear_speed",
            "max_angular_speed",
            "max_linear_accel",
            "max_angular_accel",
            "control_period_sec",
            "nav2_stop_delay_sec",
            "odom_timeout_sec",
            "depth_clear_hold_sec",
            "min_obstacle_distance_m",
            "obstacle_pixel_ratio",
            "min_valid_pixel_ratio",
            "noise_valid_pixel_ratio",
            "lookahead_m",
            "max_path_deviation_m",
            "arrival_tolerance_m",
            "heading_tolerance_deg",
            "linear_heading_threshold_deg",
            "corner_angle_threshold_deg",
            "corner_sample_distance_m",
            "corner_cluster_distance_m",
            "corner_slowdown_distance_m",
            "corner_position_tolerance_m",
            "start_position_tolerance_m",
            "start_yaw_tolerance_deg",
            "robot_radius_m",
            "hard_margin_m",
        )
        for name in numeric:
            value = float(self.get_parameter(name).value)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            setattr(self, name, value)
        self.require_nav2_cancel = bool(
            self.get_parameter("require_nav2_cancel").value
        )
        self.require_static_map = bool(
            self.get_parameter("require_static_map").value
        )
        self.stop_on_depth_insufficient_data = bool(
            self.get_parameter("stop_on_depth_insufficient_data").value
        )
        self.depth_center_roi_size_px = int(
            self.get_parameter("depth_center_roi_size_px").value
        )
        if self.depth_center_roi_size_px <= 0:
            raise ValueError("depth_center_roi_size_px must be positive")
        self.occupied_threshold = int(
            self.get_parameter("occupied_threshold").value
        )
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)

    @staticmethod
    def _load_route(route_file: str) -> dict:
        with Path(route_file).open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        route = document.get("route", document)
        if not route.get("ready", False):
            raise ValueError("route file is not ready; capture a Nav2 path first")
        if "start_yaw_deg" not in route or len(route.get("points", [])) < 2:
            raise ValueError("route needs start_yaw_deg and at least two points")
        return route

    def _on_odom(self, message: Odometry) -> None:
        self.latest_odom = message
        self.last_odom_received = self._now()

    def _on_amcl(self, message: PoseWithCovarianceStamped) -> None:
        self.latest_amcl = message
        self.last_amcl_received_at = time.monotonic()
        if not self.amcl_announced:
            self.amcl_announced = True
            pose = message.pose.pose
            yaw_deg = math.degrees(_yaw_from_quaternion(pose.orientation))
            self.get_logger().info(
                "AMCL pose active: "
                f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, "
                f"yaw={yaw_deg:.1f}deg"
            )

    def _on_depth(self, message: CompressedImage) -> None:
        try:
            self.latest_depth = decode_compressed_depth(message.data)
            received_at = self._now()
            header_age = received_at - _stamp_to_sec(message.header.stamp)
            self.depth_frames_in_window += 1
            if not self.depth_announced:
                self.depth_announced = True
                self.get_logger().info(
                    f"Depth stream active: topic={self.depth_topic}, "
                    f"shape={self.latest_depth.shape}, "
                    f"dtype={self.latest_depth.dtype}"
                )
            stats_now = time.monotonic()
            stats_elapsed = stats_now - self.depth_stats_started_at
            if stats_elapsed >= 5.0:
                rate_hz = self.depth_frames_in_window / stats_elapsed
                self.get_logger().info(
                    f"Depth receive: rate={rate_hz:.1f}Hz, "
                    f"header_age={header_age:.2f}s"
                )
                self.depth_stats_started_at = stats_now
                self.depth_frames_in_window = 0
        except Exception as error:
            self.get_logger().error(f"compressed depth conversion failed: {error}")

    def _on_map(self, message: OccupancyGrid) -> None:
        info = message.info
        grid = OccupancyGridData(
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            message.data,
        )
        clearance = compute_clearance_field(grid, self.occupied_threshold)
        segment_results = [
            path_segment_is_safe(
                grid,
                start,
                end,
                clearance,
                self.robot_radius_m,
                self.hard_margin_m,
                allow_unknown=False,
                occupied_threshold=self.occupied_threshold,
                allow_start_inside_margin=index == 0,
            )
            for index, (start, end) in enumerate(
                zip(self.geometry.points, self.geometry.points[1:])
            )
        ]
        self.map_valid = all(segment_results)
        corner_clearances = []
        for corner_index in self.corner_indices:
            cell = grid.world_to_cell(*self.geometry.points[corner_index])
            value = (
                clearance[grid.index(*cell)] if grid.in_bounds(*cell) else 0.0
            )
            corner_clearances.append(round(value, 3))
        if self.map_valid:
            self.get_logger().info(
                "Static-map route validation passed; corner_clearance_m="
                f"{corner_clearances}"
            )
        else:
            self.get_logger().error(
                "Static-map route validation failed: a route/corner cell is "
                f"within {self.robot_radius_m + self.hard_margin_m:.2f}m of a wall"
            )

    def _on_start(self, request, response):
        del request
        reason = self._odom_failure()
        if reason:
            response.success = False
            response.message = reason
            return response
        start_error = self._start_pose_failure()
        if start_error:
            response.success = False
            response.message = start_error
            return response
        depth_result = self._check_depth_safety()
        if self._depth_requires_stop(depth_result):
            response.success = False
            response.message = (
                f"depth safety is {depth_result.value}; "
                f"{self.last_depth_diagnostic}"
            )
            return response
        self._warn_depth_fail_open(depth_result)
        if self.require_static_map and self.map_valid is not True:
            response.success = False
            response.message = (
                "static map not received"
                if self.map_valid is None
                else "route is unsafe on static map"
            )
            return response
        if self.require_nav2_cancel and not self.cancel_client.service_is_ready():
            response.success = False
            response.message = "Nav2 cancel service unavailable"
            return response
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())
        self._publish_stop()
        self.anchor = self._amcl_pose(self.latest_amcl)
        position_correction = math.hypot(
            self.anchor[0] - self.route_anchor[0],
            self.anchor[1] - self.route_anchor[1],
        )
        yaw_correction = math.degrees(
            normalize_angle(self.anchor[2] - self.route_anchor[2])
        )
        self.get_logger().info(
            "Applied AMCL start-anchor correction: "
            f"position={position_correction:.3f}m, "
            f"yaw={yaw_correction:+.1f}deg"
        )
        self.odom_start = self._odom_pose(self.latest_odom)
        self.started_at = time.monotonic()
        self.closest_index = 0
        self.target_index = 0
        self.corner_cursor = 0
        self.last_log_at = None
        self.latest_estimated_pose = None
        self.result_due_at = None
        self.result_started_at = None
        self.result_logged = False
        self.resume_state = None
        self.depth_clear_since = None
        self._set_state("STARTING")
        response.success = True
        response.message = "route follower starting"
        return response

    def _on_stop(self, request, response):
        del request
        self._publish_stop()
        self._set_state("STOPPED")
        response.success = True
        response.message = "route follower stopped"
        return response

    def _on_tick(self) -> None:
        if self.state == "WAITING" and self.latest_amcl is None:
            self._request_fresh_amcl()
        if self.state in ("SUCCEEDED", "FAILED"):
            if (
                not self.result_logged
                and self.result_started_at is not None
                and (
                    self.last_amcl_received_at is None
                    or self.last_amcl_received_at <= self.result_started_at
                )
            ):
                self._request_fresh_amcl()
            if (
                not self.result_logged
                and self.result_due_at is not None
                and time.monotonic() >= self.result_due_at
            ):
                self._log_result()
            return

        if self.state not in (
            "STARTING",
            "ALIGNING",
            "DRIVING",
            "CORNER_ALIGN",
            "GOAL_ALIGN",
            "BLOCKED",
        ):
            return
        if self.state == "BLOCKED":
            depth_result = self._check_depth_safety()
            if self._depth_requires_stop(depth_result):
                self.depth_clear_since = None
                self._publish_stop()
                return
            self._warn_depth_fail_open(depth_result)
            clear_now = time.monotonic()
            if self.depth_clear_since is None:
                self.depth_clear_since = clear_now
                self.get_logger().info(
                    "Depth clear; holding stop for "
                    f"{self.depth_clear_hold_sec:.2f}s before resume"
                )
                self._publish_stop()
                return
            clear_elapsed = clear_now - self.depth_clear_since
            if clear_elapsed < self.depth_clear_hold_sec:
                self._publish_stop()
                return
            restored = self.resume_state or "DRIVING"
            self.resume_state = None
            self.depth_clear_since = None
            self.get_logger().info(
                f"Depth clear stable for {clear_elapsed:.2f}s; resuming"
            )
            self._set_state(restored)
        if self.state == "STARTING":
            self._publish_stop()
            if time.monotonic() - self.started_at < self.nav2_stop_delay_sec:
                return
            self._set_state("ALIGNING")

        reason = self._odom_failure()
        if reason:
            self._fail(reason)
            return
        current = integrate_odom_delta(
            self.anchor, self.odom_start, self._odom_pose(self.latest_odom)
        )
        self.latest_estimated_pose = current
        self._publish_pose(current)

        if self.state in ("ALIGNING", "CORNER_ALIGN", "GOAL_ALIGN"):
            self._tick_alignment(current)
            return

        progress = update_path_progress(
            list(self.geometry.points),
            current[0],
            current[1],
            self.closest_index,
            self.target_index,
            self.lookahead_m,
            0.8,
            0.15,
            0.4,
        )
        self.closest_index = progress.closest_index
        if progress.closest_distance_m > self.max_path_deviation_m:
            self._fail(f"path deviation {progress.closest_distance_m:.2f}m")
            return

        corner_index = self._current_corner_index()
        if corner_index is not None:
            corner = self.geometry.points[corner_index]
            if math.hypot(current[0] - corner[0], current[1] - corner[1]) <= (
                self.corner_position_tolerance_m
            ):
                self._publish_stop()
                self.closest_index = corner_index
                self.target_index = corner_index
                self._set_state("CORNER_ALIGN")
                return
            stop_index = corner_index
        else:
            goal = self.geometry.points[-1]
            if math.hypot(current[0] - goal[0], current[1] - goal[1]) <= (
                self.arrival_tolerance_m
            ):
                self._publish_stop()
                self._set_state("GOAL_ALIGN" if self.goal_yaw is not None else "SUCCEEDED")
                return
            stop_index = len(self.geometry.points) - 1

        target, self.target_index = select_target_before_index(
            self.geometry, self.closest_index, self.lookahead_m, stop_index
        )
        linear, angular = compute_cmd_vel(
            current[0],
            current[1],
            current[2],
            target[0],
            target[1],
            self.max_linear_speed,
            self.max_angular_speed,
            math.radians(self.linear_heading_threshold_deg),
        )
        if corner_index is not None:
            remaining = remaining_path_distance(
                self.geometry, self.closest_index, corner_index
            )
            linear = min(
                linear,
                corner_speed_limit(
                    self.max_linear_speed,
                    remaining,
                    self.corner_slowdown_distance_m,
                ),
            )
        self._publish_command(linear, angular)
        self._publish_target(target)
        self._log_status(current, target, corner_index)

    def _request_fresh_amcl(self) -> None:
        """Ask AMCL to publish while the robot is stationary."""
        now = time.monotonic()
        if (
            self.last_amcl_update_request is not None
            and now - self.last_amcl_update_request < 2.0
        ):
            return
        if not self.amcl_update_client.service_is_ready():
            return
        self.last_amcl_update_request = now
        self.amcl_update_client.call_async(Empty.Request())
        self.get_logger().info(
            "Requested fresh AMCL pose for stationary start validation"
        )

    def _tick_alignment(self, current: Pose2D) -> None:
        if self.state == "ALIGNING":
            desired = heading_after_index(
                self.geometry, 0, self.corner_sample_distance_m
            )
            next_state = "DRIVING"
        elif self.state == "CORNER_ALIGN":
            corner = self.corner_indices[self.corner_cursor]
            desired = heading_after_index(
                self.geometry, corner, self.corner_sample_distance_m
            )
            next_state = "DRIVING"
        else:
            desired = self.goal_yaw
            next_state = "SUCCEEDED"
        error = normalize_angle(desired - current[2])
        if abs(math.degrees(error)) <= self.heading_tolerance_deg:
            self._publish_stop()
            if self.state == "CORNER_ALIGN":
                self.corner_cursor += 1
            self._set_state(next_state)
            return
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, error))
        self._publish_command(0.0, angular)

    def _current_corner_index(self) -> Optional[int]:
        if self.corner_cursor >= len(self.corner_indices):
            return None
        return self.corner_indices[self.corner_cursor]

    def _publish_command(self, linear: float, angular: float) -> None:
        depth_result = self._check_depth_safety()
        if self._depth_requires_stop(depth_result):
            if self.state != "BLOCKED":
                self.resume_state = self.state
                self.depth_clear_since = None
                self._set_state("BLOCKED")
                self.get_logger().warning(
                    f"Depth safety stop: {depth_result.value}; "
                    f"{self.last_depth_diagnostic}"
                )
            self._publish_stop()
            return
        self._warn_depth_fail_open(depth_result)
        linear = rate_limit(
            self.previous_linear,
            linear,
            self.max_linear_accel * self.control_period_sec,
        )
        angular = rate_limit(
            self.previous_angular,
            angular,
            self.max_angular_accel * self.control_period_sec,
        )
        self.previous_linear = linear
        self.previous_angular = angular
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd_pub.publish(command)

    def _publish_stop(self) -> None:
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.cmd_pub.publish(Twist())

    def _odom_failure(self) -> str:
        if self.latest_odom is None:
            return "odom missing"
        if is_stale(self.last_odom_received, self._now(), self.odom_timeout_sec):
            return "odom stale"
        return ""

    def _start_pose_failure(self) -> str:
        if self.latest_amcl is None:
            return "AMCL pose missing"
        pose = self.latest_amcl.pose.pose
        current_yaw = _yaw_from_quaternion(pose.orientation)
        distance = math.hypot(
            pose.position.x - self.route_anchor[0],
            pose.position.y - self.route_anchor[1],
        )
        yaw_error_deg = abs(math.degrees(normalize_angle(current_yaw - self.start_yaw)))
        if distance > self.start_position_tolerance_m:
            return (
                f"start position error {distance:.2f}m exceeds "
                f"{self.start_position_tolerance_m:.2f}m"
            )
        if yaw_error_deg > self.start_yaw_tolerance_deg:
            return (
                f"start yaw error {yaw_error_deg:.1f}deg exceeds "
                f"{self.start_yaw_tolerance_deg:.1f}deg"
            )
        self.get_logger().info(
            f"Start pose validated: position_error={distance:.3f}m, "
            f"yaw_error={yaw_error_deg:.1f}deg"
        )
        return ""

    def _check_depth_safety(self) -> DepthSafetyResult:
        rois = self._forward_rois()
        if not rois:
            self.last_depth_result = DepthSafetyResult.INSUFFICIENT_DATA
            self.last_depth_diagnostic = "roi=unavailable,distance=n/a,valid=0.0%"
            return self.last_depth_result
        # The test policy intentionally measures only a small image-centre
        # patch. A single pixel would be too sensitive to stereo invalids.
        names = ["center"]
        results = []
        diagnostics = []
        for name in names:
            result = evaluate_depth_safety(
                self.latest_depth,
                self.min_obstacle_distance_m,
                self.obstacle_pixel_ratio,
                self.min_valid_pixel_ratio,
                rois[name],
                self.noise_valid_pixel_ratio,
            )
            metrics = compute_depth_region_metrics(
                self.latest_depth, rois[name], self.min_obstacle_distance_m
            )
            results.append(result)
            detected_distance = (
                metrics.close_median_m
                if result == DepthSafetyResult.OBSTACLE
                else metrics.p05_m
            )
            diagnostics.append(
                (
                    name,
                    result,
                    f"roi={name},distance={format_distance(detected_distance)},"
                    f"valid={metrics.valid_ratio * 100:.1f}%,"
                    f"under_{self.min_obstacle_distance_m:.2f}m="
                    f"{metrics.close_ratio * 100:.1f}%",
                )
            )
        self.last_depth_result = worst_depth_result(results)
        self.last_depth_diagnostic = next(
            text
            for _, result, text in diagnostics
            if result == self.last_depth_result
        )
        return self.last_depth_result

    def _depth_requires_stop(self, result: DepthSafetyResult) -> bool:
        if result == DepthSafetyResult.OBSTACLE:
            return True
        if result == DepthSafetyResult.NOISY_DEPTH:
            return True
        if result == DepthSafetyResult.INSUFFICIENT_DATA:
            return self.stop_on_depth_insufficient_data
        return False

    def _warn_depth_fail_open(self, result: DepthSafetyResult) -> None:
        if result == DepthSafetyResult.CLEAR:
            return
        now = time.monotonic()
        if (
            self.last_depth_policy_warning_at is not None
            and now - self.last_depth_policy_warning_at < 5.0
        ):
            return
        self.last_depth_policy_warning_at = now
        self.get_logger().warning(
            f"Depth safety {result.value} ignored by test configuration; "
            "OBSTACLE detection remains active"
        )

    def _forward_rois(self):
        if self.latest_depth is None:
            return {}
        height, width = self.latest_depth.shape[:2]
        half = max(1, self.depth_center_roi_size_px // 2)
        center_x = width // 2
        center_y = height // 2
        return {
            "center": (
                max(0, center_x - half),
                max(0, center_y - half),
                min(width, center_x + half),
                min(height, center_y + half),
            ),
        }

    def _fail(self, reason: str) -> None:
        self._publish_stop()
        self._set_state("FAILED")
        self.get_logger().error(reason)

    def _set_state(self, state: str) -> None:
        if state != self.state:
            self.get_logger().warning(f"cmd_vel_route: {self.state} -> {state}")
        self.state = state
        self.state_pub.publish(String(data=state))
        if state in ("SUCCEEDED", "FAILED") and not self.result_logged:
            self.result_started_at = time.monotonic()
            self.result_due_at = self.result_started_at + 3.0
            self.last_amcl_update_request = None
            self._request_fresh_amcl()

    def _log_result(self) -> None:
        self.result_logged = True
        goal_x, goal_y = self.geometry.points[-1]
        estimated_text = "unavailable"
        if self.latest_estimated_pose is not None:
            estimated_text = (
                f"({self.latest_estimated_pose[0]:.3f},"
                f"{self.latest_estimated_pose[1]:.3f},"
                f"{math.degrees(self.latest_estimated_pose[2]):.1f}deg)"
            )
        amcl_text = "unavailable"
        error_text = "unavailable"
        if self.latest_amcl is not None:
            pose = self.latest_amcl.pose.pose
            actual_yaw = _yaw_from_quaternion(pose.orientation)
            position_error = math.hypot(
                pose.position.x - goal_x, pose.position.y - goal_y
            )
            amcl_text = (
                f"({pose.position.x:.3f},{pose.position.y:.3f},"
                f"{math.degrees(actual_yaw):.1f}deg)"
            )
            if self.goal_yaw is None:
                error_text = f"position={position_error:.3f}m"
            else:
                yaw_error = abs(
                    math.degrees(normalize_angle(actual_yaw - self.goal_yaw))
                )
                error_text = (
                    f"position={position_error:.3f}m,yaw={yaw_error:.1f}deg"
                )
        self.get_logger().warning(
            f"ROUTE_RESULT state={self.state} goal=({goal_x:.3f},{goal_y:.3f}) "
            f"estimated={estimated_text} amcl={amcl_text} error={error_text} "
            f"amcl_fresh={self._result_amcl_is_fresh()}"
        )

    def _result_amcl_is_fresh(self) -> bool:
        return (
            self.result_started_at is not None
            and self.last_amcl_received_at is not None
            and self.last_amcl_received_at > self.result_started_at
        )

    def _publish_path(self) -> None:
        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.geometry.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_target(self, target: Tuple[float, float]) -> None:
        self._publish_pose_message(self.target_pub, (target[0], target[1], 0.0))

    def _publish_pose(self, pose: Pose2D) -> None:
        self._publish_pose_message(self.pose_pub, pose)

    def _publish_pose_message(self, publisher, pose: Pose2D) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        message.pose.orientation.z = math.sin(pose[2] * 0.5)
        message.pose.orientation.w = math.cos(pose[2] * 0.5)
        publisher.publish(message)

    def _log_status(
        self, current: Pose2D, target: Tuple[float, float], corner: Optional[int]
    ) -> None:
        now = self._now()
        if self.last_log_at is not None and now - self.last_log_at < 1.0:
            return
        error = heading_error_to_target(*current, *target)
        deviation = path_deviation_m(
            list(self.geometry.points), current[0], current[1]
        )
        self.get_logger().info(
            f"pose=({current[0]:.2f},{current[1]:.2f},"
            f"{math.degrees(current[2]):.1f}deg) target={target} "
            f"heading_error={math.degrees(error):.1f}deg "
            f"deviation={deviation:.2f}m next_corner={corner}"
        )
        self.last_log_at = now

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _odom_pose(message: Odometry) -> Pose2D:
        pose = message.pose.pose
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)

    @staticmethod
    def _amcl_pose(message: PoseWithCovarianceStamped) -> Pose2D:
        pose = message.pose.pose
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
