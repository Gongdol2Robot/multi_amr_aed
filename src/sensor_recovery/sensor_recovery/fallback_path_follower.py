"""Pause Nav2 and follow a freshly map-planned path via odometry + depth
safety stop while LiDAR is FAULT/RECOVERING; report SUCCEEDED/FAILED
explicitly and request a replacement robot on FAILED. Resume Nav2 once
LiDAR is ALIVE again, but only if no replacement was already dispatched.

On FAULT, the last pre-fault Nav2 goal's destination is kept, but the path
to it is *replanned* — not by asking Nav2's planner_server, but with our
own A* over the latched static /map. Nav2's planner_server can't be
trusted here: its global costmap has an obstacle_layer sourced from /scan,
and AMCL's map->odom TF eventually goes stale too, so compute_path_to_pose
can fail or hang for the same underlying reason LiDAR died in the first
place (confirmed by real-robot testing: replanning through Nav2 while
LiDAR was actually off produced a lot of errors). A plan computed directly
from the static occupancy grid has no such dependency, and can bias away
from walls instead of chasing the literal shortest path — see
`grid_path_planner.py`.

[CODE REVIEW]
LiDAR FAULT를 받은 뒤 Nav2 속도 명령권을 안전하게 회수하고, 마지막 목표까지
static map/odom/depth 기반으로 주행하는 전체 fallback orchestration 노드다.
"""

import math
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger

from sensor_recovery.fallback_state_machine import (
    FallbackState,
    FallbackTickInputs,
    next_fallback_state,
)
from sensor_recovery.depth_decode import decode_compressed_depth
from sensor_recovery.grid_path_planner import (
    OccupancyGridData,
    compute_clearance_field,
    plan_path,
    simplify_path,
)
from sensor_recovery.lidar_state_machine import LidarState
from sensor_recovery.path_follow_control import (
    DepthSafetyResult,
    Pose2D,
    compute_cmd_vel,
    depth_result_blocks_motion,
    evaluate_depth_safety,
    goal_reached,
    heading_error_to_target,
    integrate_odom_delta,
    is_stale,
    path_deviation_m,
    pose_error,
    rate_limit,
    remaining_path_from_pose,
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

_STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class FallbackPathFollower(Node):
    """LiDAR-fault response: drive blind via odom + depth, report success/failure."""

    # [CODE REVIEW]
    # 설명 순서: FAULT 수신 -> Nav2 cancel 확인 -> 경로 snapshot/재계획
    # -> odom 기반 제어 + depth 정지 -> 성공/실패 -> 복구 또는 대체 요청.
    # 경로/제어 수학은 ROS-free 모듈로 분리하고 이 클래스는 lifecycle과 I/O를 묶는다.

    def __init__(self) -> None:
        super().__init__("fallback_path_follower")
        self._declare_parameters()
        self._read_parameters()

        self.bridge = CvBridge()
        self._latest_path: Optional[Path] = None
        self._latest_odom: Optional[Odometry] = None
        self._latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self._odom_at_latest_amcl: Optional[Pose2D] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._last_odom_time: Optional[float] = None
        self._odom_stale_active = False
        self._last_amcl_time: Optional[float] = None
        self._depth_stream_announced = False
        self._nav2_goal_active = False

        self._lidar_state = LidarState.STARTING
        self._fallback_state = FallbackState.IDLE
        self._fault_session_active = False
        self._fault_path_points = []
        self._saved_nav_path_points = []
        self._route_geometry = None
        self._corner_indices = ()
        self._corner_cursor = 0
        self._aligning_corner = False
        self._fault_goal_pose: Optional[PoseStamped] = None
        self._odom_anchor: Optional[Pose2D] = None
        self._odom_start: Optional[Pose2D] = None
        self._fault_amcl_pose: Optional[Pose2D] = None
        self._closest_index = 0
        self._target_index = 0
        self._blocked_since: Optional[float] = None
        self._depth_clear_since: Optional[float] = None
        self._last_depth_unavailable_warning_time: Optional[float] = None
        self._stuck_anchor: Optional[Tuple[float, float, float]] = None
        self._prev_linear_cmd = 0.0
        self._prev_angular_cmd = 0.0
        self._replacement_dispatched = False
        self._awaiting_reconvergence = False
        self._alive_triggered_time: Optional[float] = None
        self._recovery_mode: Optional[str] = None
        self._recovery_amcl_samples = []
        self._recovery_wait_warning_logged = False
        self._last_amcl_update_request: Optional[float] = None
        self._replan_started = False
        self._fault_triggered_time: Optional[float] = None
        self._cancel_future = None
        self._cancel_response_accepted = False
        self._cancel_requested_time: Optional[float] = None
        self._cancel_confirmed = False
        self._map_grid: Optional[OccupancyGridData] = None
        self._map_clearance: Optional[list] = None
        self._last_debug_log_time: Optional[float] = None
        self._depth_subscription = None

        # Feed the same velocity smoother input Nav2 uses.  After Nav2's goal
        # cancellation is confirmed this leaves exactly one publisher on the
        # physical cmd_vel output instead of racing the velocity smoother.
        self.cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.fallback_state_pub = self.create_publisher(String, "fallback_state", _STATUS_QOS)
        self.replacement_needed_pub = self.create_publisher(
            Bool, "replacement_needed", _STATUS_QOS
        )
        self.recovery_ready_pub = self.create_publisher(
            Bool, "recovery_ready", _STATUS_QOS
        )
        self.pending_goal_pub = self.create_publisher(PoseStamped, "pending_goal", _STATUS_QOS)
        self.debug_path_pub = self.create_publisher(
            Path, "fallback_debug/path", _STATUS_QOS
        )
        self.debug_target_pub = self.create_publisher(
            PoseStamped, "fallback_debug/target", 10
        )
        self.debug_pose_pub = self.create_publisher(
            PoseStamped, "fallback_debug/estimated_pose", 10
        )

        self.create_subscription(String, "lidar_state", self._on_lidar_state, 10)
        self.create_subscription(Path, "plan", self._on_plan, 10)
        self.create_subscription(
            GoalStatusArray,
            f"{self._navigate_action}/_action/status",
            self._on_nav_status,
            10,
        )
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, 10
        )
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, _MAP_QOS)

        if self.enable_manual_trigger:
            self.create_service(
                Trigger, "fallback/manual_start", self._on_manual_start
            )
            self.create_service(
                Trigger, "fallback/manual_stop", self._on_manual_stop
            )

        self._cancel_client = self.create_client(
            CancelGoal, f"{self._navigate_action}/_action/cancel_goal"
        )
        self._amcl_update_client = self.create_client(
            Empty, "request_nomotion_update"
        )

        self.replacement_needed_pub.publish(Bool(data=False))
        self.recovery_ready_pub.publish(Bool(data=True))
        self._publish_fallback_state()

        self._control_timer = self.create_timer(
            self._fallback_control_period_sec, self._on_control_tick
        )
        if self.allow_insufficient_depth_motion:
            self.get_logger().warning(
                "Depth fail-open enabled: INSUFFICIENT_DATA will be logged "
                "but will not stop fallback; OBSTACLE/NOISY_DEPTH still stop"
            )

    # -- setup -----------------------------------------------------------

    def _declare_parameters(self) -> None:
        defaults = {
            # Match the normal Nav2 RPP command limits. Depth stop, corner
            # slowdown and this controller's acceleration limits still apply.
            "max_linear_speed": 0.20,
            "max_angular_speed": 0.60,
            "max_linear_accel": 0.15,
            "max_angular_accel": 0.5,
            "lookahead_m": 0.3,
            "closest_search_ahead_m": 1.0,
            "closest_search_backtrack_m": 0.3,
            "path_reacquire_distance_m": 0.5,
            "linear_heading_threshold_deg": 60.0,
            "arrival_tolerance_m": 0.15,
            "min_obstacle_distance_m": 0.65,
            "obstacle_pixel_ratio": 0.03,
            "min_valid_pixel_ratio": 0.20,
            "noise_valid_pixel_ratio": 0.60,
            "fallback_control_period_sec": 0.1,
            "odom_timeout_sec": 2.0,
            "depth_clear_hold_sec": 0.5,
            "blocked_timeout_sec": 5.0,
            "stuck_timeout_sec": 3.0,
            "stuck_distance_m": 0.03,
            "max_path_deviation_m": 0.7,
            "reconvergence_timeout_sec": 5.0,
            "recovery_amcl_required_samples": 3,
            "recovery_amcl_stability_distance_m": 0.15,
            "recovery_amcl_stability_angle_deg": 15.0,
            "recovery_amcl_update_period_sec": 1.0,
            "pre_replan_delay_sec": 1.0,
            "nav2_cancel_timeout_sec": 3.0,
            "corner_angle_threshold_deg": 35.0,
            "corner_sample_distance_m": 0.25,
            "corner_cluster_distance_m": 0.35,
            "corner_slowdown_distance_m": 0.35,
            "corner_position_tolerance_m": 0.08,
            "corner_heading_tolerance_deg": 5.0,
            # Robot footprint radius (matches nav2_aed.yaml's robot_radius)
            # plus a fixed safety margin: cells any closer to a known wall
            # are outright blocked during self-planning.
            "robot_radius_m": 0.20,
            "hard_margin_m": 0.05,
            # Cells with less than this much clearance from a wall get a
            # cost penalty (scaled by wall_clearance_weight) so the planner
            # prefers open space over the literal shortest path.
            "soft_clearance_m": 0.4,
            "wall_clearance_weight": 2.0,
            "occupied_threshold": 50,
            "debug_log_period_sec": 1.0,
            # Neutral defaults. Robot-specific values are supplied by the
            # launch/test wrapper after a measured odom-vs-AMCL comparison.
            "odom_translation_scale": 1.0,
            "odom_translation_heading_correction_deg": 0.0,
            "odom_yaw_delta_scale": 1.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_nav")
        self.declare_parameter(
            "depth_topic", "oakd/stereo/image_raw/compressedDepth"
        )
        self.declare_parameter("depth_compressed", True)
        self.declare_parameter("allow_insufficient_depth_motion", False)
        self.declare_parameter("require_active_nav_goal", True)
        self.declare_parameter("prefer_saved_nav2_path", True)
        self.declare_parameter("resume_nav2_after_failure", True)
        self.declare_parameter("allow_unknown_cells", False)
        self.declare_parameter("debug_enabled", False)
        self.declare_parameter("enable_manual_trigger", False)

    def _read_parameters(self) -> None:
        self.max_linear = self._positive_param("max_linear_speed", 0.20)
        self.max_angular = self._positive_param("max_angular_speed", 0.60)
        self.max_linear_accel = self._positive_param("max_linear_accel", 0.15)
        self.max_angular_accel = self._positive_param("max_angular_accel", 0.5)
        self.lookahead_m = self._positive_param("lookahead_m", 0.3)
        self.closest_search_ahead_m = self._positive_param(
            "closest_search_ahead_m", 1.0
        )
        self.closest_search_backtrack_m = self._nonnegative_param(
            "closest_search_backtrack_m", 0.3
        )
        self.path_reacquire_distance_m = self._positive_param(
            "path_reacquire_distance_m", 0.5
        )
        heading_threshold_deg = self._positive_param(
            "linear_heading_threshold_deg", 60.0
        )
        if heading_threshold_deg > 180.0:
            self.get_logger().error(
                "Parameter 'linear_heading_threshold_deg' must be <= 180; "
                "using default 60.0"
            )
            heading_threshold_deg = 60.0
        self.linear_heading_threshold_rad = math.radians(heading_threshold_deg)
        self.arrival_tolerance_m = self._positive_param("arrival_tolerance_m", 0.15)
        self.min_obstacle_distance_m = self._positive_param("min_obstacle_distance_m", 0.65)
        self.obstacle_pixel_ratio = self._positive_param("obstacle_pixel_ratio", 0.03)
        self.min_valid_pixel_ratio = self._positive_param("min_valid_pixel_ratio", 0.20)
        self.noise_valid_pixel_ratio = self._positive_param(
            "noise_valid_pixel_ratio", 0.60
        )
        self._fallback_control_period_sec = self._positive_param(
            "fallback_control_period_sec", 0.1
        )
        self.odom_timeout_sec = self._positive_param("odom_timeout_sec", 2.0)
        self.depth_clear_hold_sec = self._nonnegative_param(
            "depth_clear_hold_sec", 0.5
        )
        self.blocked_timeout_sec = self._positive_param("blocked_timeout_sec", 5.0)
        self.stuck_timeout_sec = self._positive_param("stuck_timeout_sec", 3.0)
        self.stuck_distance_m = self._positive_param("stuck_distance_m", 0.03)
        self.max_path_deviation_m = self._positive_param("max_path_deviation_m", 0.7)
        self.reconvergence_timeout_sec = self._positive_param("reconvergence_timeout_sec", 5.0)
        self.recovery_amcl_required_samples = int(
            self.get_parameter("recovery_amcl_required_samples").value
        )
        if self.recovery_amcl_required_samples < 1:
            self.get_logger().error(
                "Parameter 'recovery_amcl_required_samples' must be >= 1; using 3"
            )
            self.recovery_amcl_required_samples = 3
        self.recovery_amcl_stability_distance_m = self._positive_param(
            "recovery_amcl_stability_distance_m", 0.15
        )
        recovery_stability_angle_deg = self._positive_param(
            "recovery_amcl_stability_angle_deg", 15.0
        )
        self.recovery_amcl_stability_angle_rad = math.radians(
            recovery_stability_angle_deg
        )
        self.recovery_amcl_update_period_sec = self._positive_param(
            "recovery_amcl_update_period_sec", 1.0
        )
        self.pre_replan_delay_sec = self._positive_param("pre_replan_delay_sec", 1.0)
        self.nav2_cancel_timeout_sec = self._positive_param(
            "nav2_cancel_timeout_sec", 3.0
        )
        self.corner_angle_threshold_deg = self._positive_param(
            "corner_angle_threshold_deg", 35.0
        )
        self.corner_sample_distance_m = self._positive_param(
            "corner_sample_distance_m", 0.25
        )
        self.corner_cluster_distance_m = self._positive_param(
            "corner_cluster_distance_m", 0.35
        )
        self.corner_slowdown_distance_m = self._positive_param(
            "corner_slowdown_distance_m", 0.35
        )
        self.corner_position_tolerance_m = self._positive_param(
            "corner_position_tolerance_m", 0.08
        )
        self.corner_heading_tolerance_deg = self._positive_param(
            "corner_heading_tolerance_deg", 5.0
        )
        self.robot_radius_m = self._positive_param("robot_radius_m", 0.20)
        self.hard_margin_m = float(self.get_parameter("hard_margin_m").value)
        self.soft_clearance_m = self._positive_param("soft_clearance_m", 0.4)
        self.wall_clearance_weight = float(self.get_parameter("wall_clearance_weight").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.debug_log_period_sec = self._positive_param(
            "debug_log_period_sec", 1.0
        )
        self.odom_translation_scale = self._positive_param(
            "odom_translation_scale", 1.0
        )
        translation_heading_correction_deg = float(
            self.get_parameter("odom_translation_heading_correction_deg").value
        )
        if abs(translation_heading_correction_deg) > 45.0:
            self.get_logger().error(
                "Parameter 'odom_translation_heading_correction_deg' must be "
                "between -45 and 45; using 0.0"
            )
            translation_heading_correction_deg = 0.0
        self.odom_translation_heading_correction_rad = math.radians(
            translation_heading_correction_deg
        )
        self.odom_yaw_delta_scale = self._positive_param(
            "odom_yaw_delta_scale", 1.0
        )
        self._navigate_action = str(self.get_parameter("navigate_action").value)
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._depth_compressed = bool(
            self.get_parameter("depth_compressed").value
        )
        self.allow_insufficient_depth_motion = bool(
            self.get_parameter("allow_insufficient_depth_motion").value
        )
        self.require_active_nav_goal = bool(
            self.get_parameter("require_active_nav_goal").value
        )
        self.prefer_saved_nav2_path = bool(
            self.get_parameter("prefer_saved_nav2_path").value
        )
        self.resume_nav2_after_failure = bool(
            self.get_parameter("resume_nav2_after_failure").value
        )
        self.allow_unknown_cells = bool(self.get_parameter("allow_unknown_cells").value)
        self.debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self.enable_manual_trigger = bool(
            self.get_parameter("enable_manual_trigger").value
        )

    def _positive_param(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().error(
                f"Parameter '{name}'={value} must be positive; using default {default}"
            )
            return default
        return value

    def _nonnegative_param(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            self.get_logger().error(
                f"Parameter '{name}'={value} must be nonnegative; using default {default}"
            )
            return default
        return value

    # -- subscriptions -----------------------------------------------------

    def _on_plan(self, msg: Path) -> None:
        # Keep the last complete map-frame path.  Nav2 can briefly publish an
        # empty path while stopping; overwriting here would lose the mission
        # destination exactly when the LiDAR fault is handled.
        if msg.poses and msg.header.frame_id == "map":
            self._latest_path = msg

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        active_codes = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self._nav2_goal_active = any(
            status.status in active_codes for status in msg.status_list
        )
        if self._nav2_goal_active and self._awaiting_reconvergence:
            self.get_logger().error(
                "Nav2 goal detected before the recovery position check "
                "completed; keeping the robot stopped and canceling the goal"
            )
            self.cmd_vel_pub.publish(Twist())
            self._cancel_nav2_goal()
            return
        active_fallback_states = {
            FallbackState.STARTING,
            FallbackState.ACTIVE,
            FallbackState.BLOCKED,
            FallbackState.RECOVERING,
        }
        if (
            self._nav2_goal_active
            and self._cancel_confirmed
            and self._fallback_state in active_fallback_states
        ):
            self.get_logger().error(
                "New Nav2 goal detected during fallback; stopping and "
                "canceling it before cmd_vel control continues"
            )
            self.cmd_vel_pub.publish(Twist())
            self._cancel_nav2_goal()

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self._map_grid = OccupancyGridData(
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            data=msg.data,
        )
        self._map_clearance = compute_clearance_field(
            self._map_grid, self.occupied_threshold
        )
        self.get_logger().info(
            f"Static map received: {info.width}x{info.height} @ {info.resolution:.3f} m/cell "
            "(cached for LiDAR-fault self-planning)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg
        # Compare freshness using the callback receipt time on this computer.
        # The robot header stamp can differ from the laptop clock and includes
        # network transport delay, which previously caused false 0.5 s stale
        # failures even while odom packets were still arriving.
        self._last_odom_time = self._now_sec()

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_amcl_pose = msg
        self._last_amcl_time = _stamp_to_sec(msg.header.stamp)
        if self._latest_odom is not None:
            self._odom_at_latest_amcl = self._pose2d_from_odom(self._latest_odom)
        if self._awaiting_reconvergence:
            msg_time = _stamp_to_sec(msg.header.stamp)
            if self._alive_triggered_time is None or msg_time <= self._alive_triggered_time:
                return
            pose = self._pose2d_from_amcl(msg)
            if self._recovery_amcl_samples:
                previous = self._recovery_amcl_samples[-1]
                distance_m, angle_deg = pose_error(previous, pose)
                if (
                    distance_m > self.recovery_amcl_stability_distance_m
                    or math.radians(angle_deg)
                    > self.recovery_amcl_stability_angle_rad
                ):
                    self.get_logger().warning(
                        "AMCL recovery sample jumped "
                        f"{distance_m:.2f}m/{angle_deg:.1f}deg; restarting "
                        "the stability count"
                    )
                    self._recovery_amcl_samples = [pose]
                    return
            self._recovery_amcl_samples.append(pose)
            self.get_logger().info(
                "Fresh stable AMCL recovery sample "
                f"{len(self._recovery_amcl_samples)}/"
                f"{self.recovery_amcl_required_samples}: "
                f"pose=({pose[0]:.3f},{pose[1]:.3f},"
                f"{math.degrees(pose[2]):.1f}deg)"
            )
            if (
                len(self._recovery_amcl_samples)
                >= self.recovery_amcl_required_samples
            ):
                self._on_reconverged()

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self._store_depth(depth)
        except Exception as error:
            self.get_logger().error(f"depth conversion failed: {error}")

    def _on_compressed_depth(self, msg: CompressedImage) -> None:
        try:
            # compressed_depth_image_transport prefixes the PNG payload with
            # a small transport header.  Searching for the PNG signature is
            # robust across its 16UC1 header variants.
            depth = decode_compressed_depth(msg.data)
            self._store_depth(depth)
        except Exception as error:
            self.get_logger().error(f"compressed depth conversion failed: {error}")

    def _store_depth(self, depth: np.ndarray) -> None:
        self._latest_depth = depth
        if not self._depth_stream_announced:
            self._depth_stream_announced = True
            self.get_logger().info(
                f"Depth stream active: topic={self._depth_topic}, "
                f"shape={depth.shape}, dtype={depth.dtype}"
            )

    def _start_depth_subscription(self) -> None:
        """Subscribe only while fallback needs depth obstacle checks."""
        if self._depth_subscription is not None:
            return
        self._latest_depth = None
        if self._depth_compressed:
            self._depth_subscription = self.create_subscription(
                CompressedImage,
                self._depth_topic,
                self._on_compressed_depth,
                qos_profile_sensor_data,
            )
        else:
            self._depth_subscription = self.create_subscription(
                Image,
                self._depth_topic,
                self._on_depth,
                qos_profile_sensor_data,
            )
        self.get_logger().info(
            f"Depth subscription enabled for fallback: {self._depth_topic}"
        )

    def _stop_depth_subscription(self) -> None:
        """Release the DDS reader so normal driving carries no depth stream."""
        subscription = self._depth_subscription
        if subscription is None:
            return
        self._depth_subscription = None
        self.destroy_subscription(subscription)
        self._latest_depth = None
        self.get_logger().info("Depth subscription disabled")

    def _on_lidar_state(self, msg: String) -> None:
        # [CODE REVIEW] FAULT edge에서 한 번만 fallback을 시작하고,
        # FAULT/RECOVERING 뒤 ALIVE가 되었을 때만 위치 재수렴과 Nav2 복귀를 검토한다.
        try:
            new_state = LidarState(msg.data)
        except ValueError:
            return
        previous = self._lidar_state
        self._lidar_state = new_state
        if new_state == LidarState.FAULT and previous != LidarState.FAULT:
            self._start_fallback()
        elif new_state == LidarState.ALIVE and previous in (
            LidarState.FAULT,
            LidarState.RECOVERING,
        ) and self._fault_session_active:
            self._on_lidar_recovered()

    # -- fallback lifecycle --------------------------------------------------

    def _manual_start_failure(self) -> str:
        if self._fallback_state in (
            FallbackState.STARTING,
            FallbackState.ACTIVE,
            FallbackState.BLOCKED,
        ):
            return f"fallback already {self._fallback_state.value}"
        if self.require_active_nav_goal and not self._nav2_goal_active:
            return "active Nav2 goal missing"
        if self._latest_path is None or len(self._latest_path.poses) < 2:
            return "Nav2 /plan missing"
        if self._latest_path.header.frame_id != "map":
            return f"Nav2 /plan frame is {self._latest_path.header.frame_id!r}, not 'map'"
        if self._latest_amcl_pose is None:
            return "AMCL pose missing"
        if self._latest_odom is None:
            return "odom missing"
        if is_stale(self._last_odom_time, self._now_sec(), self.odom_timeout_sec):
            return "odom stale"
        if self._latest_depth is None:
            return "compressed depth missing"
        if self._map_grid is None:
            return "static map missing"
        if not self._cancel_client.service_is_ready():
            return "Nav2 cancel service unavailable"
        return ""

    def _on_manual_start(self, request, response):
        del request
        self._start_depth_subscription()
        reason = self._manual_start_failure()
        if reason:
            response.success = False
            response.message = reason
            return response
        self._lidar_state = LidarState.FAULT
        started = self._start_fallback("MANUAL TAKEOVER")
        response.success = started
        response.message = (
            "manual Nav2-to-cmd_vel takeover accepted"
            if started
            else "manual takeover rejected"
        )
        return response

    def _on_manual_stop(self, request, response):
        del request
        self.cmd_vel_pub.publish(Twist())
        self._prev_linear_cmd = 0.0
        self._prev_angular_cmd = 0.0
        self._fault_session_active = False
        self._stop_depth_subscription()
        self._awaiting_reconvergence = False
        self._recovery_mode = None
        self._recovery_amcl_samples = []
        self._fallback_state = FallbackState.IDLE
        self._publish_fallback_state()
        response.success = True
        response.message = "manual fallback stopped; Nav2 remains canceled"
        return response

    def _start_fallback(self, trigger_label: str = "LiDAR FAULT") -> bool:
        # [CODE REVIEW] 장애 순간의 마지막 map pose, odom, Nav2 plan endpoint를 snapshot한다.
        # 이후 AMCL/scan이 없어도 같은 목표와 상대 odom 이동량으로 주행할 기준점이 된다.
        if self.require_active_nav_goal and not self._nav2_goal_active:
            self.get_logger().info(
                "LiDAR FAULT while Nav2 is idle; fallback drive will not start"
            )
            return False

        self._start_depth_subscription()
        self._fault_session_active = True
        self.recovery_ready_pub.publish(Bool(data=False))
        self.cmd_vel_pub.publish(Twist())
        self._prev_linear_cmd = 0.0
        self._prev_angular_cmd = 0.0
        self._closest_index = 0
        self._target_index = 0
        self._last_debug_log_time = None
        self._blocked_since = None
        self._depth_clear_since = None
        self._last_depth_unavailable_warning_time = None
        self._odom_stale_active = False
        self._stuck_anchor = None
        self._awaiting_reconvergence = False
        self._alive_triggered_time = None
        self._recovery_mode = None
        self._recovery_amcl_samples = []
        self._recovery_wait_warning_logged = False
        self._last_amcl_update_request = None
        self._replacement_dispatched = False
        self._fault_path_points = []
        self._saved_nav_path_points = []
        self._route_geometry = None
        self._corner_indices = ()
        self._corner_cursor = 0
        self._aligning_corner = False
        self._replan_started = False
        self._fault_triggered_time = self._now_sec()

        if self._latest_amcl_pose is not None:
            last_amcl = self._pose2d_from_amcl(self._latest_amcl_pose)
            if self._odom_at_latest_amcl is not None and self._latest_odom is not None:
                self._fault_amcl_pose = integrate_odom_delta(
                    last_amcl,
                    self._odom_at_latest_amcl,
                    self._pose2d_from_odom(self._latest_odom),
                )
            else:
                self._fault_amcl_pose = last_amcl
        else:
            self._fault_amcl_pose = None
        self._odom_anchor = self._fault_amcl_pose
        self._odom_start = (
            self._pose2d_from_odom(self._latest_odom) if self._latest_odom is not None else None
        )

        self._fault_goal_pose = self._extract_goal_pose()
        if self._fault_amcl_pose is not None and self._latest_path is not None:
            plan_points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in self._latest_path.poses
            ]
            self._saved_nav_path_points = remaining_path_from_pose(
                plan_points,
                self._fault_amcl_pose[0],
                self._fault_amcl_pose[1],
            )

        # STARTING을 먼저 발행해야 Mission Executor가 이어서 도착할 Nav2
        # CANCELED 결과를 일반 주행 실패로 보고하지 않는다.
        self._fallback_state = FallbackState.STARTING
        self._publish_fallback_state()
        # Nav2와 fallback이 같은 cmd_vel 입력을 동시에 내보내지 않도록 취소한다.
        # 실제 non-zero 명령은 _nav2_cancel_ready() 확인 뒤에만 발행된다.
        self._cancel_nav2_goal()
        if self._fault_amcl_pose is not None and self._fault_goal_pose is not None:
            goal = self._fault_goal_pose.pose.position
            self.get_logger().warning(
                "FALLBACK_SNAPSHOT "
                f"pose=({self._fault_amcl_pose[0]:.3f},"
                f"{self._fault_amcl_pose[1]:.3f},"
                f"{math.degrees(self._fault_amcl_pose[2]):.1f}deg) "
                f"goal=({goal.x:.3f},{goal.y:.3f}) "
                f"saved_path_points={len(self._saved_nav_path_points)}"
            )
        self.get_logger().warning(
            f"{trigger_label}: pausing Nav2 and stopping for "
            f"{self.pre_replan_delay_sec:.1f}s before following the saved route"
        )
        return True

    def _extract_goal_pose(self) -> Optional[PoseStamped]:
        """Best-known destination, taken from the last pre-fault /plan's endpoint."""
        if self._latest_path is None or not self._latest_path.poses:
            self.get_logger().warning("LiDAR FAULT but no /plan available; no destination known")
            return None
        if self._latest_path.header.frame_id != "map":
            self.get_logger().error(
                f"/plan frame_id is '{self._latest_path.header.frame_id}', expected 'map'; "
                "discarding it (position estimate is map-frame)"
            )
            return None
        return self._latest_path.poses[-1]

    def _request_replan(self) -> None:
        # [CODE REVIEW] 기본은 장애 직전 Nav2 route의 남은 부분을 사용한다.
        # 필요하면 live scan/costmap 대신 latched static map만으로 독립 A*를 수행한다.
        if self._fault_amcl_pose is None or self._fault_goal_pose is None:
            self.get_logger().warning(
                "Cannot replan: missing last-known pose or destination"
            )
            return
        if self.prefer_saved_nav2_path and len(self._saved_nav_path_points) >= 2:
            self._set_fault_path(self._saved_nav_path_points)
            self._publish_debug_path()
            self.get_logger().warning(
                f"Using {len(self._fault_path_points)} points from the last "
                "Nav2 route; no Nav2 planner is used during fallback"
            )
            return
        if self._map_grid is None:
            self.get_logger().error(
                "No /map received yet; cannot self-plan without a static map"
            )
            return

        start_x, start_y, _ = self._fault_amcl_pose
        goal_x = self._fault_goal_pose.pose.position.x
        goal_y = self._fault_goal_pose.pose.position.y
        self.get_logger().info(
            f"Planning map-based path from ({start_x:.2f}, {start_y:.2f}) to "
            f"({goal_x:.2f}, {goal_y:.2f}) using the static map (no Nav2 involved)"
        )
        raw_path = plan_path(
            self._map_grid,
            (start_x, start_y),
            (goal_x, goal_y),
            robot_radius_m=self.robot_radius_m,
            hard_margin_m=self.hard_margin_m,
            soft_clearance_m=self.soft_clearance_m,
            wall_clearance_weight=self.wall_clearance_weight,
            allow_unknown=self.allow_unknown_cells,
            occupied_threshold=self.occupied_threshold,
            clearance=self._map_clearance,
        )
        if not raw_path:
            self.get_logger().error(
                "Map-based path planning found no wall-clear route to the goal"
            )
            return
        path = simplify_path(
            self._map_grid,
            raw_path,
            self._map_clearance,
            robot_radius_m=self.robot_radius_m,
            hard_margin_m=self.hard_margin_m,
            allow_unknown=self.allow_unknown_cells,
            occupied_threshold=self.occupied_threshold,
        )
        self._set_fault_path(path)
        self._publish_debug_path()
        self.get_logger().warning(
            f"Planned {len(raw_path)} grid points, simplified to {len(path)} "
            "wall-clear waypoints; "
            "starting fallback drive"
        )

    def _set_fault_path(self, points) -> None:
        self._fault_path_points = list(points)
        self._closest_index = 0
        self._target_index = 0
        self._corner_cursor = 0
        self._aligning_corner = False
        self._route_geometry = build_route_geometry(
            self._fault_path_points,
            math.radians(self.corner_angle_threshold_deg),
            self.corner_sample_distance_m,
            self.corner_cluster_distance_m,
        )
        total = self._route_geometry.cumulative_m[-1]
        endpoint_guard = max(self.arrival_tolerance_m, self.corner_sample_distance_m)
        self._corner_indices = interior_corner_indices(
            self._route_geometry,
            self.corner_sample_distance_m,
            endpoint_guard,
        )
        self.get_logger().info(
            f"Fallback route length={total:.2f}m, hard_corners="
            f"{list(self._corner_indices)}"
        )
        self.get_logger().info(
            "Odom calibration: translation_scale="
            f"{self.odom_translation_scale:.3f}, translation_heading_correction="
            f"{math.degrees(self.odom_translation_heading_correction_rad):+.1f}deg, "
            f"yaw_delta_scale={self.odom_yaw_delta_scale:.3f}"
        )

    def _current_corner_index(self) -> Optional[int]:
        if self._corner_cursor >= len(self._corner_indices):
            return None
        return self._corner_indices[self._corner_cursor]

    def _on_lidar_recovered(self) -> None:
        # [CODE REVIEW] LiDAR ALIVE 직후에는 바로 Nav2를 재시작하지 않는다.
        # 정지 상태에서 fresh/stable AMCL을 확인하고, 이미 도착했으면 goal을 다시 보내지 않는다.
        self.cmd_vel_pub.publish(Twist())
        self._stop_depth_subscription()
        self._prev_linear_cmd = 0.0
        self._prev_angular_cmd = 0.0

        fallback_completed = self._fallback_state == FallbackState.SUCCEEDED
        self._recovery_mode = "validate_only" if fallback_completed else "resume"

        if self._fallback_state not in (FallbackState.SUCCEEDED, FallbackState.FAILED):
            self._fallback_state = FallbackState.RECOVERING
        self._publish_fallback_state()

        self._awaiting_reconvergence = True
        self._alive_triggered_time = self._now_sec()
        self._recovery_amcl_samples = []
        self._recovery_wait_warning_logged = False
        self._last_amcl_update_request = None
        if fallback_completed:
            self.get_logger().warning(
                "LiDAR ALIVE after fallback arrival: staying stopped and waiting "
                "only for a fresh stable AMCL position check; the completed Nav2 "
                "goal will not be sent again"
            )
        else:
            self.get_logger().warning(
                "LiDAR ALIVE before fallback arrival: staying stopped until fresh "
                "stable AMCL is available before resuming Nav2"
            )
        self._request_amcl_update(self._alive_triggered_time)

    def _on_reconverged(self) -> None:
        self._awaiting_reconvergence = False
        self.get_logger().info("Fresh stable AMCL position confirmed")
        self._decide_post_recovery_action()

    def _decide_post_recovery_action(self) -> None:
        self._fault_session_active = False
        self.replacement_needed_pub.publish(Bool(data=False))
        if self._recovery_mode == "validate_only":
            self._log_recovery_position_check()
            self._recovery_mode = None
            self.recovery_ready_pub.publish(Bool(data=True))
            self.get_logger().warning(
                "Fallback had already reached the destination; staying stopped "
                "with Nav2 idle"
            )
            return
        if self._replacement_dispatched and not self.resume_nav2_after_failure:
            self._recovery_mode = None
            self.recovery_ready_pub.publish(Bool(data=True))
            self.get_logger().info(
                "A replacement was already requested for this mission; staying stopped "
                "and waiting for Mission Manager, even though LiDAR recovered"
            )
            return
        if self._replacement_dispatched:
            self.get_logger().warning(
                "LiDAR recovered after fallback failure; clearing replacement request "
                "and resuming the original Nav2 goal"
            )
            self._replacement_dispatched = False
        self.recovery_ready_pub.publish(Bool(data=True))
        if self._fault_goal_pose is not None:
            # Mission Executor가 원래 assignment 식별자와 version을 유지한 채
            # Nav2 goal을 다시 보내도록 명시적으로 hand-off한다.
            self._fallback_state = FallbackState.RESUMED
            self._publish_fallback_state()
        else:
            self._fallback_state = FallbackState.FAILED
            self._publish_fallback_state()
            self.get_logger().error(
                "LiDAR recovered but there was no pending goal to resume"
            )
        self._recovery_mode = None

    def _log_recovery_position_check(self) -> None:
        if self._latest_amcl_pose is None or self._fault_goal_pose is None:
            self.get_logger().warning(
                "RECOVERY_POSITION_CHECK unavailable: missing AMCL pose or goal"
            )
            return
        amcl_pose = self._pose2d_from_amcl(self._latest_amcl_pose)
        goal = self._fault_goal_pose.pose.position
        goal_error = math.hypot(goal.x - amcl_pose[0], goal.y - amcl_pose[1])
        self.get_logger().warning(
            "RECOVERY_POSITION_CHECK "
            f"goal=({goal.x:.3f},{goal.y:.3f}) "
            f"amcl=({amcl_pose[0]:.3f},{amcl_pose[1]:.3f},"
            f"{math.degrees(amcl_pose[2]):.1f}deg) "
            f"goal_error={goal_error:.3f}m result="
            f"{'PASS' if goal_error <= self.arrival_tolerance_m else 'OUTSIDE_TOLERANCE'}"
        )

    def _request_amcl_update(self, now: float) -> None:
        if (
            self._last_amcl_update_request is not None
            and now - self._last_amcl_update_request
            < self.recovery_amcl_update_period_sec
        ):
            return
        self._last_amcl_update_request = now
        if self._amcl_update_client.service_is_ready():
            self._amcl_update_client.call_async(Empty.Request())

    # -- control loop --------------------------------------------------------

    def _on_control_tick(self) -> None:
        # [CODE REVIEW] 매 tick의 책임은 위치 추정 -> 경로 진행률 -> depth 안전 판정
        # -> 순수 상태 머신 -> 상태가 ACTIVE일 때만 cmd_vel 발행 순서다.
        now = self._now_sec()

        if self._awaiting_reconvergence:
            self.cmd_vel_pub.publish(Twist())
            self._request_amcl_update(now)
            if (
                self._alive_triggered_time is not None
                and (now - self._alive_triggered_time) > self.reconvergence_timeout_sec
                and not self._recovery_wait_warning_logged
            ):
                self.get_logger().warning(
                    "Fresh stable AMCL position is still unavailable after "
                    f"{self.reconvergence_timeout_sec:.1f}s; staying stopped "
                    "instead of resuming Nav2"
                )
                self._recovery_wait_warning_logged = True
            return

        active_states = (
            FallbackState.STARTING,
            FallbackState.ACTIVE,
            FallbackState.BLOCKED,
        )
        if self._fallback_state not in active_states:
            return

        # Never publish a non-zero fallback command until Nav2 confirms its
        # active goal was canceled.  Both controllers use cmd_vel_nav, so this
        # is the ownership hand-off that prevents competing speed commands.
        # 이 확인이 Nav2에서 fallback으로 넘어가는 cmd_vel ownership hand-off다.
        if not self._nav2_cancel_ready(now):
            self.cmd_vel_pub.publish(Twist())
            if (
                self._cancel_requested_time is not None
                and now - self._cancel_requested_time > self.nav2_cancel_timeout_sec
            ):
                self._fail_fallback("Nav2 goal cancellation timed out")
            return

        # Stay stopped for pre_replan_delay_sec after FAULT before even
        # asking for a path — gives Nav2's own stop and sensors a moment to
        # settle instead of immediately taking over.
        if not self._replan_started:
            if (
                self._fault_triggered_time is not None
                and (now - self._fault_triggered_time) < self.pre_replan_delay_sec
            ):
                self.cmd_vel_pub.publish(Twist())
                return
            self._replan_started = True
            self._request_replan()

        current_pose = self._estimate_current_pose(now)
        odom_stale = is_stale(self._last_odom_time, now, self.odom_timeout_sec)
        odom_age = (
            math.inf
            if self._last_odom_time is None
            else max(0.0, now - self._last_odom_time)
        )
        was_odom_stale = self._odom_stale_active
        self._odom_stale_active = odom_stale
        if odom_stale and not was_odom_stale:
            self.get_logger().warning(
                "ODOM_STALE: no local odom receipt for "
                f"{odom_age:.2f}s (limit={self.odom_timeout_sec:.2f}s); "
                "stopping fallback and waiting for fresh odom"
            )
        elif was_odom_stale and not odom_stale:
            self.get_logger().warning(
                "ODOM_RECOVERED: fresh odom received; fallback may resume"
            )

        target = None
        linear_x = angular_z = 0.0
        heading_error = 0.0
        arrived = False
        deviation = 0.0
        goal_distance = math.inf
        progress = None

        if current_pose is not None and not odom_stale and self._fault_path_points:
            current_x, current_y, current_yaw = current_pose
            progress = update_path_progress(
                self._fault_path_points,
                current_x,
                current_y,
                self._closest_index,
                self._target_index,
                self.lookahead_m,
                self.closest_search_ahead_m,
                self.closest_search_backtrack_m,
                self.path_reacquire_distance_m,
            )
            self._closest_index = progress.closest_index
            self._target_index = progress.target_index
            deviation = path_deviation_m(
                self._fault_path_points,
                current_x,
                current_y,
                progress.search_start_index,
                progress.search_end_index,
            )
            last_x, last_y = self._fault_path_points[-1]
            goal_distance = math.hypot(last_x - current_x, last_y - current_y)
            arrived = goal_reached(
                self._fault_path_points,
                current_x,
                current_y,
                self.arrival_tolerance_m,
            )
            if not arrived:
                corner_index = self._current_corner_index()
                if self._aligning_corner and corner_index is not None:
                    desired = heading_after_index(
                        self._route_geometry,
                        corner_index,
                        self.corner_sample_distance_m,
                    )
                    heading_error = math.atan2(
                        math.sin(desired - current_yaw),
                        math.cos(desired - current_yaw),
                    )
                    if abs(math.degrees(heading_error)) <= (
                        self.corner_heading_tolerance_deg
                    ):
                        self._aligning_corner = False
                        self._corner_cursor += 1
                    else:
                        angular_z = max(
                            -self.max_angular,
                            min(self.max_angular, heading_error),
                        )
                else:
                    stop_index = len(self._fault_path_points) - 1
                    if corner_index is not None:
                        corner = self._fault_path_points[corner_index]
                        corner_distance = math.hypot(
                            current_x - corner[0], current_y - corner[1]
                        )
                        if corner_distance <= self.corner_position_tolerance_m:
                            self._aligning_corner = True
                            target = corner
                        else:
                            stop_index = corner_index
                    if not self._aligning_corner:
                        target, self._target_index = select_target_before_index(
                            self._route_geometry,
                            self._closest_index,
                            self.lookahead_m,
                            stop_index,
                        )
                        heading_error = heading_error_to_target(
                            current_x,
                            current_y,
                            current_yaw,
                            target[0],
                            target[1],
                        )
                        linear_x, angular_z = compute_cmd_vel(
                            current_x,
                            current_y,
                            current_yaw,
                            target[0],
                            target[1],
                            self.max_linear,
                            self.max_angular,
                            self.linear_heading_threshold_rad,
                        )
                        if corner_index is not None:
                            remaining = remaining_path_distance(
                                self._route_geometry,
                                self._closest_index,
                                corner_index,
                            )
                            linear_x = min(
                                linear_x,
                                corner_speed_limit(
                                    self.max_linear,
                                    remaining,
                                    self.corner_slowdown_distance_m,
                                ),
                            )

        # Once inside goal tolerance the command is already zero, so missing
        # depth must not turn a completed fallback into BLOCKED/FAILED.
        depth_result = (
            DepthSafetyResult.CLEAR
            if arrived
            else self._check_depth_safety(angular_z)
        )
        raw_depth_blocked = depth_result_blocks_motion(
            depth_result,
            allow_insufficient=self.allow_insufficient_depth_motion,
        )
        depth_unavailable = depth_result == DepthSafetyResult.INSUFFICIENT_DATA
        if depth_unavailable and not raw_depth_blocked:
            if (
                self._last_depth_unavailable_warning_time is None
                or (now - self._last_depth_unavailable_warning_time) >= 5.0
            ):
                self._last_depth_unavailable_warning_time = now
                self.get_logger().warning(
                    f"Depth {depth_result.value}; continuing fallback because "
                    "allow_insufficient_depth_motion=true"
                )
        elif not depth_unavailable:
            self._last_depth_unavailable_warning_time = None
        depth_blocked = raw_depth_blocked
        if arrived:
            self._depth_clear_since = None
        elif raw_depth_blocked:
            self._depth_clear_since = None
        elif (
            self._fallback_state == FallbackState.BLOCKED
            and self.depth_clear_hold_sec > 0.0
        ):
            if self._depth_clear_since is None:
                self._depth_clear_since = now
                self.get_logger().info(
                    "Depth clear; holding stop for "
                    f"{self.depth_clear_hold_sec:.2f}s before resume"
                )
            clear_elapsed = now - self._depth_clear_since
            depth_blocked = clear_elapsed < self.depth_clear_hold_sec
            if not depth_blocked:
                self.get_logger().info(
                    f"Depth clear stable for {clear_elapsed:.2f}s; resuming"
                )
                self._depth_clear_since = None

        commanded_nonzero = abs(linear_x) > 1e-3 or abs(angular_z) > 1e-3
        stuck = self._update_stuck_tracking(
            now, current_pose, commanded_nonzero and not depth_blocked
        )

        if raw_depth_blocked:
            if self._blocked_since is None:
                self._blocked_since = now
        else:
            self._blocked_since = None
        blocked_duration = (now - self._blocked_since) if self._blocked_since is not None else 0.0

        # ROS/센서 값을 한 tick snapshot으로 변환한 뒤 순수 상태 머신에 넘긴다.
        # BLOCKED는 0속도로 회복을 기다리고, FAILED는 아래에서 대체 로봇 요청으로 이어진다.
        inputs = FallbackTickInputs(
            has_plan=bool(self._fault_path_points),
            has_anchor=self._odom_anchor is not None and self._odom_start is not None,
            odom_stale=odom_stale,
            depth_blocked=depth_blocked,
            blocked_duration_sec=blocked_duration,
            blocked_timeout_sec=self.blocked_timeout_sec,
            stuck=stuck,
            path_deviation_m=deviation,
            max_path_deviation_m=self.max_path_deviation_m,
            arrived=arrived,
        )
        previous_state = self._fallback_state
        self._fallback_state = next_fallback_state(self._fallback_state, inputs)
        if self._fallback_state != previous_state:
            transition_reason = self._fallback_transition_reason(
                inputs, depth_result, odom_age, was_odom_stale
            )
            self.get_logger().warning(
                f"fallback_state: {previous_state.value} -> "
                f"{self._fallback_state.value} reason={transition_reason}"
            )
            self._publish_fallback_state()
            if self._fallback_state == FallbackState.SUCCEEDED:
                self._stop_depth_subscription()
                self._log_arrival_validation(now, current_pose)

        if self._fallback_state == FallbackState.ACTIVE and not depth_blocked:
            linear_x = rate_limit(
                self._prev_linear_cmd, linear_x,
                self.max_linear_accel * self._fallback_control_period_sec,
            )
            angular_z = rate_limit(
                self._prev_angular_cmd, angular_z,
                self.max_angular_accel * self._fallback_control_period_sec,
            )
        else:
            linear_x = angular_z = 0.0

        self._prev_linear_cmd = linear_x
        self._prev_angular_cmd = angular_z
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

        self._publish_debug(
            now,
            current_pose,
            target,
            goal_distance,
            deviation,
            heading_error,
            linear_x,
            angular_z,
            depth_result,
            progress.reacquired if progress is not None else False,
        )

        if self._fallback_state == FallbackState.FAILED and not self._replacement_dispatched:
            self._request_replacement()

    def _fallback_transition_reason(
        self,
        inputs: FallbackTickInputs,
        depth_result: DepthSafetyResult,
        odom_age: float,
        was_odom_stale: bool,
    ) -> str:
        """Return the exact condition responsible for a state transition."""
        if self._fallback_state == FallbackState.FAILED:
            if not inputs.has_plan:
                return "MISSING_PLAN"
            if not inputs.has_anchor:
                return "MISSING_ODOM_ANCHOR"
            if inputs.stuck:
                return "STUCK"
            if inputs.path_deviation_m > inputs.max_path_deviation_m:
                return (
                    f"PATH_DEVIATION({inputs.path_deviation_m:.3f}m>"
                    f"{inputs.max_path_deviation_m:.3f}m)"
                )
            if (
                inputs.depth_blocked
                and inputs.blocked_duration_sec > inputs.blocked_timeout_sec
            ):
                return (
                    f"DEPTH_TIMEOUT({depth_result.value},"
                    f"{inputs.blocked_duration_sec:.2f}s>"
                    f"{inputs.blocked_timeout_sec:.2f}s)"
                )
            return "UNKNOWN_FAILURE"
        if inputs.odom_stale:
            return (
                f"ODOM_STALE(age={odom_age:.2f}s,"
                f"limit={self.odom_timeout_sec:.2f}s)"
            )
        if inputs.depth_blocked:
            return f"DEPTH_{depth_result.value}"
        if self._fallback_state == FallbackState.SUCCEEDED:
            return "GOAL_REACHED"
        if was_odom_stale:
            return "ODOM_RECOVERED"
        return "SAFETY_CLEAR"

    def _estimate_current_pose(self, now: float) -> Optional[Pose2D]:
        if (
            self._odom_anchor is None
            or self._odom_start is None
            or self._latest_odom is None
            or is_stale(self._last_odom_time, now, self.odom_timeout_sec)
        ):
            return None
        return integrate_odom_delta(
            self._odom_anchor,
            self._odom_start,
            self._pose2d_from_odom(self._latest_odom),
            translation_scale=self.odom_translation_scale,
            translation_heading_correction_rad=(
                self.odom_translation_heading_correction_rad
            ),
            yaw_delta_scale=self.odom_yaw_delta_scale,
        )

    def _log_arrival_validation(
        self, now: float, calibrated_pose: Optional[Pose2D]
    ) -> None:
        if calibrated_pose is None or not self._fault_path_points:
            return
        goal_x, goal_y = self._fault_path_points[-1]
        calibrated_error = math.hypot(
            goal_x - calibrated_pose[0], goal_y - calibrated_pose[1]
        )
        raw_pose = integrate_odom_delta(
            self._odom_anchor,
            self._odom_start,
            self._pose2d_from_odom(self._latest_odom),
        )
        raw_error = math.hypot(goal_x - raw_pose[0], goal_y - raw_pose[1])
        result = (
            "FALLBACK_RESULT "
            f"goal=({goal_x:.3f},{goal_y:.3f}) "
            f"calibrated_pose=({calibrated_pose[0]:.3f},"
            f"{calibrated_pose[1]:.3f}) calibrated_error={calibrated_error:.3f}m "
            f"raw_odom_pose=({raw_pose[0]:.3f},{raw_pose[1]:.3f}) "
            f"raw_odom_error={raw_error:.3f}m"
        )
        if self._latest_amcl_pose is not None and self._last_amcl_time is not None:
            amcl_pose = self._pose2d_from_amcl(self._latest_amcl_pose)
            amcl_age = max(0.0, now - self._last_amcl_time)
            amcl_goal_error = math.hypot(
                goal_x - amcl_pose[0], goal_y - amcl_pose[1]
            )
            estimate_amcl_error = math.hypot(
                calibrated_pose[0] - amcl_pose[0],
                calibrated_pose[1] - amcl_pose[1],
            )
            result += (
                f" latest_amcl=({amcl_pose[0]:.3f},{amcl_pose[1]:.3f}) "
                f"amcl_age={amcl_age:.2f}s amcl_goal_error={amcl_goal_error:.3f}m "
                f"estimate_amcl_error={estimate_amcl_error:.3f}m"
            )
        self.get_logger().warning(result)

    def _check_depth_safety(self, angular_z: float) -> DepthSafetyResult:
        rois = self._forward_rois()
        if not rois:
            return DepthSafetyResult.INSUFFICIENT_DATA
        names = ["center"]
        turn_threshold = 0.05
        if angular_z > turn_threshold:
            names.append("left")
        elif angular_z < -turn_threshold:
            names.append("right")
        results = [
            evaluate_depth_safety(
                self._latest_depth, self.min_obstacle_distance_m,
                self.obstacle_pixel_ratio, self.min_valid_pixel_ratio, rois[name],
                self.noise_valid_pixel_ratio,
            )
            for name in names
        ]
        return worst_depth_result(results)

    def _forward_rois(self) -> Dict[str, Tuple[int, int, int, int]]:
        if self._latest_depth is None:
            return {}
        height, width = self._latest_depth.shape[:2]
        third = max(1, width // 3)
        y0, y1 = int(height * 0.3), int(height * 0.7)
        return {
            "left": (0, y0, third, y1),
            "center": (third, y0, min(width, 2 * third), y1),
            "right": (min(width, 2 * third), y0, width, y1),
        }

    def _update_stuck_tracking(
        self, now: float, current_pose: Optional[Pose2D], trying_to_move: bool
    ) -> bool:
        if current_pose is None:
            self._stuck_anchor = None
            return False
        current_x, current_y, _ = current_pose
        if self._stuck_anchor is None:
            self._stuck_anchor = (current_x, current_y, now)
            return False
        anchor_x, anchor_y, anchor_time = self._stuck_anchor
        moved = math.hypot(current_x - anchor_x, current_y - anchor_y)
        if moved > self.stuck_distance_m or not trying_to_move:
            self._stuck_anchor = (current_x, current_y, now)
            return False
        return (now - anchor_time) > self.stuck_timeout_sec

    def _publish_debug_path(self) -> None:
        if not self.debug_enabled or not self._fault_path_points:
            return
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in self._fault_path_points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.debug_path_pub.publish(message)

    def _publish_debug(
        self,
        now: float,
        current_pose: Optional[Pose2D],
        target: Optional[Tuple[float, float]],
        goal_distance: float,
        deviation: float,
        heading_error: float,
        linear_x: float,
        angular_z: float,
        depth_result: DepthSafetyResult,
        reacquired: bool,
    ) -> None:
        if not self.debug_enabled:
            return
        if (
            self._last_debug_log_time is not None
            and (now - self._last_debug_log_time) < self.debug_log_period_sec
        ):
            return
        self._last_debug_log_time = now

        pose_text = "unavailable"
        if current_pose is not None:
            current_x, current_y, current_yaw = current_pose
            pose_text = f"({current_x:.2f}, {current_y:.2f}, {current_yaw:.2f})"
            pose_message = self._debug_pose_message(
                current_x, current_y, current_yaw
            )
            self.debug_pose_pub.publish(pose_message)

        target_text = "none"
        if target is not None:
            target_text = f"({target[0]:.2f}, {target[1]:.2f})"
            target_message = self._debug_pose_message(target[0], target[1], 0.0)
            self.debug_target_pub.publish(target_message)

        self.get_logger().info(
            "fallback_debug: "
            f"state={self._fallback_state.value} pose={pose_text} "
            f"closest={self._closest_index} target={self._target_index} "
            f"target_point={target_text} goal_dist={goal_distance:.3f}m "
            f"lateral_error={deviation:.3f}m "
            f"heading_error={math.degrees(heading_error):.1f}deg "
            f"cmd=({linear_x:.3f}m/s,{angular_z:.3f}rad/s) "
            f"depth={depth_result.value} reacquired={reacquired}"
        )

    def _debug_pose_message(self, x: float, y: float, yaw: float) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.orientation.w = math.cos(yaw * 0.5)
        return message

    def _request_replacement(self) -> None:
        # [CODE REVIEW] fallback이 안전하게 계속 갈 수 없을 때 실패 상태와
        # 원래 목적지를 latched Topic으로 함께 발행해 상위 mission 계층에 전달한다.
        self._replacement_dispatched = True
        self.replacement_needed_pub.publish(Bool(data=True))
        if self._fault_goal_pose is not None:
            self.pending_goal_pub.publish(self._fault_goal_pose)
        self.get_logger().error(
            "fallback FAILED: requested a replacement robot"
            + (" (destination published)" if self._fault_goal_pose is not None else "")
        )

    def _publish_fallback_state(self) -> None:
        self.fallback_state_pub.publish(String(data=self._fallback_state.value))

    # -- Nav2 -------------------------------------------------------------

    def _cancel_nav2_goal(self) -> None:
        self._cancel_future = None
        self._cancel_response_accepted = False
        self._cancel_confirmed = False
        self._cancel_requested_time = self._now_sec()
        if not self._cancel_client.service_is_ready():
            self.get_logger().error(
                "navigate_to_pose cancel service unavailable; fallback remains stopped"
            )
            return
        self._cancel_future = self._cancel_client.call_async(CancelGoal.Request())

    def _nav2_cancel_ready(self, now: float) -> bool:
        # [CODE REVIEW] cancel service 응답 성공만으로는 goal이 완전히 멈췄다고 볼 수 없다.
        # Action status가 active/canceling을 벗어난 것까지 확인해야 cmd_vel 명령권을 넘긴다.
        del now
        if self._cancel_confirmed:
            return True
        if not self._cancel_response_accepted:
            if self._cancel_future is None or not self._cancel_future.done():
                return False
            try:
                response = self._cancel_future.result()
            except Exception as error:
                self.get_logger().error(f"Nav2 cancel request failed: {error}")
                return False
            if response.return_code != CancelGoal.Response.ERROR_NONE:
                self.get_logger().error(
                    f"Nav2 rejected cancel request: return_code={response.return_code}"
                )
                return False
            self._cancel_response_accepted = True
            self.get_logger().warning(
                "Nav2 accepted the cancel request; waiting for the action "
                "status to leave active/canceling"
            )
        if self._nav2_goal_active:
            return False
        self._cancel_confirmed = True
        self.get_logger().warning(
            "Nav2 goal cancellation confirmed; cmd_vel ownership transferred"
        )
        return True

    def _fail_fallback(self, reason: str) -> None:
        self.cmd_vel_pub.publish(Twist())
        self._stop_depth_subscription()
        self._fallback_state = FallbackState.FAILED
        self._publish_fallback_state()
        self.get_logger().error(f"fallback FAILED: {reason}")
        if not self._replacement_dispatched:
            self._request_replacement()

    # -- helpers ------------------------------------------------------------

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _pose2d_from_amcl(msg: PoseWithCovarianceStamped) -> Pose2D:
        p = msg.pose.pose
        return (p.position.x, p.position.y, _yaw_from_quaternion(p.orientation))

    @staticmethod
    def _pose2d_from_odom(msg: Odometry) -> Pose2D:
        p = msg.pose.pose
        return (p.position.x, p.position.y, _yaw_from_quaternion(p.orientation))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FallbackPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
