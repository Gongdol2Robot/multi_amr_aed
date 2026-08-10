"""Drive a predefined map route with fallback control while LiDAR stays on.

[CODE REVIEW]
실제 LiDAR FAULT 통합 흐름을 만들기 전에 저속 경로 추종만 분리 검증하려고 만든
대안 시험 노드다. 현재 운영 launch와 fault-cycle 시험에서는 사용하지 않으며,
이전 실기 시험을 재현할 때만 수동 실행하도록 보존한다.

This is a deliberately isolated real-robot test tool. Nav2 may be used to
stage the robot at the route start, but after ``start_test`` the node cancels
Nav2 and publishes the same low-speed odom-based control used by the LiDAR
fallback. LiDAR is retained only as an emergency forward obstacle stop; it is
not used for localization, path generation, or path progress.
"""

import math
from typing import Optional, Tuple

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from sensor_recovery.grid_path_planner import (
    OccupancyGridData,
    compute_clearance_field,
    path_segment_is_safe,
)
from sensor_recovery.path_follow_control import (
    Pose2D,
    compute_cmd_vel,
    goal_reached,
    heading_error_to_target,
    integrate_odom_delta,
    is_stale,
    path_deviation_m,
    rate_limit,
    update_path_progress,
)
from sensor_recovery.route_test_support import (
    densify_route,
    minimum_range_in_sector,
    parse_flat_route,
)


_LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class FallbackRouteTest(Node):
    """Explicitly triggered low-speed route test with conservative stops."""

    def __init__(self) -> None:
        super().__init__("fallback_route_test")
        self._declare_parameters()
        self._read_parameters()

        route_values = self.get_parameter(f"{self.route_name}_route").value
        self.vertices = parse_flat_route(route_values)
        self.path_points = densify_route(self.vertices, self.route_spacing_m)

        self.latest_odom: Optional[Odometry] = None
        self.latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self.latest_scan: Optional[LaserScan] = None
        self.last_odom_time: Optional[float] = None
        self.last_scan_time: Optional[float] = None
        self.map_grid: Optional[OccupancyGridData] = None
        self.map_valid = False

        self.odom_anchor: Optional[Pose2D] = None
        self.odom_start: Optional[Pose2D] = None
        self.closest_index = 0
        self.target_index = 0
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.started_at: Optional[float] = None
        self.blocked_since: Optional[float] = None
        self.last_log_time: Optional[float] = None
        self.state = "WAITING"

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "fallback_test/state", _LATCHED_QOS)
        self.path_pub = self.create_publisher(Path, "fallback_test/path", _LATCHED_QOS)
        self.target_pub = self.create_publisher(PoseStamped, "fallback_test/target", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl, 10
        )
        self.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(OccupancyGrid, "map", self._on_map, _MAP_QOS)
        self.cancel_client = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )
        self.create_service(Trigger, "fallback_test/start", self._on_start)
        self.create_service(Trigger, "fallback_test/stop", self._on_stop)
        self.create_timer(self.control_period_sec, self._on_tick)

        self._set_state("WAITING")
        self.get_logger().info(
            f"Loaded '{self.route_name}' route with {len(self.vertices)} vertices: "
            f"{self.vertices}. Move robot to {self.vertices[0]}, then call "
            "fallback_test/start"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("route_name", "straight")
        self.declare_parameter("straight_route", [0.8, 0.2, 0.8, 2.8])
        self.declare_parameter(
            "wall_corner_route", [-2.4, 0.5, -1.45, 0.5, -1.45, 1.8]
        )
        defaults = {
            "route_spacing_m": 0.05,
            "max_linear_speed": 0.05,
            "max_angular_speed": 0.2,
            "max_linear_accel": 0.15,
            "max_angular_accel": 0.5,
            "lookahead_m": 0.3,
            "arrival_tolerance_m": 0.15,
            "start_tolerance_m": 0.2,
            "max_path_deviation_m": 0.5,
            "odom_timeout_sec": 1.0,
            "scan_timeout_sec": 1.0,
            "minimum_scan_distance_m": 0.35,
            "scan_half_angle_deg": 35.0,
            "blocked_timeout_sec": 5.0,
            "nav2_stop_delay_sec": 1.0,
            "control_period_sec": 0.1,
            "robot_radius_m": 0.2,
            "hard_margin_m": 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        self.route_name = str(self.get_parameter("route_name").value)
        if self.route_name not in ("straight", "wall_corner"):
            raise ValueError("route_name must be 'straight' or 'wall_corner'")
        for name in ("route_spacing_m", "max_linear_speed", "max_angular_speed",
                     "max_linear_accel", "max_angular_accel", "lookahead_m",
                     "arrival_tolerance_m", "start_tolerance_m",
                     "max_path_deviation_m", "odom_timeout_sec", "scan_timeout_sec",
                     "minimum_scan_distance_m", "scan_half_angle_deg",
                     "blocked_timeout_sec", "nav2_stop_delay_sec",
                     "control_period_sec", "robot_radius_m", "hard_margin_m"):
            value = float(self.get_parameter(name).value)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            setattr(self, name, value)
        self.scan_half_angle_rad = math.radians(self.scan_half_angle_deg)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.last_odom_time = _stamp_to_sec(msg.header.stamp)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_amcl = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_time = _stamp_to_sec(msg.header.stamp)

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self.map_grid = OccupancyGridData(
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            msg.data,
        )
        clearance = compute_clearance_field(self.map_grid)
        self.map_valid = all(
            path_segment_is_safe(
                self.map_grid,
                start,
                end,
                clearance,
                robot_radius_m=self.robot_radius_m,
                hard_margin_m=self.hard_margin_m,
                allow_unknown=False,
                occupied_threshold=50,
            )
            for start, end in zip(self.vertices, self.vertices[1:])
        )
        if self.map_valid:
            self._publish_path()
            self.get_logger().info("Route passed static-map clearance validation")
        else:
            self.get_logger().error("Route failed static-map clearance validation")

    def _on_start(self, request, response):
        del request
        now = self._now()
        reason = self._preflight_failure(now)
        if reason:
            response.success = False
            response.message = reason
            return response

        pose = self._amcl_pose(self.latest_amcl)
        distance = math.hypot(pose[0] - self.vertices[0][0], pose[1] - self.vertices[0][1])
        if distance > self.start_tolerance_m:
            response.success = False
            response.message = (
                f"robot is {distance:.2f}m from route start {self.vertices[0]}; "
                f"must be <= {self.start_tolerance_m:.2f}m"
            )
            return response

        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())
        self._publish_stop()
        self.odom_anchor = pose
        self.odom_start = self._odom_pose(self.latest_odom)
        self.closest_index = 0
        self.target_index = 0
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.blocked_since = None
        self.started_at = now
        self._set_state("STARTING")
        response.success = True
        response.message = (
            f"starting '{self.route_name}' after {self.nav2_stop_delay_sec:.1f}s Nav2 stop delay"
        )
        return response

    def _on_stop(self, request, response):
        del request
        self._publish_stop()
        self._set_state("STOPPED")
        response.success = True
        response.message = "fallback route test stopped"
        return response

    def _preflight_failure(self, now: float) -> str:
        if not self.map_valid:
            return "static map missing or route is unsafe"
        if self.latest_amcl is None:
            return "AMCL pose missing"
        if self.latest_odom is None or is_stale(self.last_odom_time, now, self.odom_timeout_sec):
            return "odom missing or stale"
        if self.latest_scan is None or is_stale(self.last_scan_time, now, self.scan_timeout_sec):
            return "scan missing or stale"
        return ""

    def _on_tick(self) -> None:
        if self.state not in ("STARTING", "ACTIVE", "BLOCKED"):
            return
        now = self._now()
        if self.started_at is not None and now - self.started_at < self.nav2_stop_delay_sec:
            self._publish_stop()
            return

        reason = self._preflight_failure(now)
        if reason:
            self._fail(reason)
            return
        current = integrate_odom_delta(
            self.odom_anchor, self.odom_start, self._odom_pose(self.latest_odom)
        )
        progress = update_path_progress(
            self.path_points,
            current[0],
            current[1],
            self.closest_index,
            self.target_index,
            self.lookahead_m,
            1.0,
            0.3,
            0.5,
        )
        self.closest_index = progress.closest_index
        self.target_index = progress.target_index
        deviation = path_deviation_m(
            self.path_points,
            current[0],
            current[1],
            progress.search_start_index,
            progress.search_end_index,
        )
        if deviation > self.max_path_deviation_m:
            self._fail(f"path deviation {deviation:.2f}m")
            return
        if goal_reached(
            self.path_points, current[0], current[1], self.arrival_tolerance_m
        ):
            self._publish_stop()
            self._set_state("SUCCEEDED")
            self.get_logger().info(f"Route '{self.route_name}' completed")
            return

        target = progress.target_point
        linear, angular = compute_cmd_vel(
            current[0],
            current[1],
            current[2],
            target[0],
            target[1],
            self.max_linear_speed,
            self.max_angular_speed,
        )
        nearest = minimum_range_in_sector(
            self.latest_scan.ranges,
            self.latest_scan.angle_min,
            self.latest_scan.angle_increment,
            self.scan_half_angle_rad,
        )
        if nearest < self.minimum_scan_distance_m:
            if self.blocked_since is None:
                self.blocked_since = now
            self._publish_stop()
            self._set_state("BLOCKED")
            if now - self.blocked_since > self.blocked_timeout_sec:
                self._fail(f"LiDAR safety stop persisted; nearest={nearest:.2f}m")
            return

        self.blocked_since = None
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
        self._set_state("ACTIVE")
        self._publish_target(target)

        if self.last_log_time is None or now - self.last_log_time >= 1.0:
            heading = heading_error_to_target(
                current[0], current[1], current[2], target[0], target[1]
            )
            self.get_logger().info(
                f"route_test state={self.state} pose=({current[0]:.2f},"
                f"{current[1]:.2f},{math.degrees(current[2]):.0f}deg) "
                f"target=({target[0]:.2f},{target[1]:.2f}) "
                f"heading_error={math.degrees(heading):.1f}deg "
                f"cmd=({linear:.3f},{angular:.3f}) scan_min={nearest:.2f}m"
            )
            self.last_log_time = now

    def _fail(self, reason: str) -> None:
        self._publish_stop()
        self._set_state("FAILED")
        self.get_logger().error(f"Fallback route test failed: {reason}")

    def _publish_stop(self) -> None:
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.cmd_pub.publish(Twist())

    def _set_state(self, state: str) -> None:
        if state != self.state:
            self.get_logger().warning(f"fallback_test state: {self.state} -> {state}")
        self.state = state
        self.state_pub.publish(String(data=state))

    def _publish_path(self) -> None:
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path_points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_target(self, target: Tuple[float, float]) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.orientation.w = 1.0
        self.target_pub.publish(message)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _amcl_pose(msg: PoseWithCovarianceStamped) -> Pose2D:
        pose = msg.pose.pose
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)

    @staticmethod
    def _odom_pose(msg: Odometry) -> Pose2D:
        pose = msg.pose.pose
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FallbackRouteTest()
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
