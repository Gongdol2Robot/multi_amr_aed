"""
Run an open-loop 0.5 m cmd_vel test with AMCL/odom error measurement.

This node intentionally contains no path following, A*, fallback state machine,
depth processing, or LiDAR fault logic. It establishes the most basic hardware
baseline: publish one constant forward Twist for a fixed duration, stop, and
compare the measured final pose with the ideal pose projected from the measured
start pose.
"""

import json
import math
import time
from typing import Optional, Tuple

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from sensor_recovery.distance_test_metrics import (
    calculate_distance_test_metrics,
    project_odom_pose_to_map,
)
from sensor_recovery.path_follow_control import normalize_angle


Pose2D = Tuple[float, float, float]
_LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CmdVelDistanceTest(Node):
    """Run one constant-speed movement and report its position error."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_distance_test")
        self._declare_parameters()
        self._read_parameters()

        self.latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self.latest_odom: Optional[Odometry] = None
        self.last_amcl_received: Optional[float] = None
        self.last_odom_received: Optional[float] = None
        self.start_amcl: Optional[Pose2D] = None
        self.start_odom: Optional[Pose2D] = None
        self.move_started_at: Optional[float] = None
        self.move_started_ros: Optional[float] = None
        self.move_stopped_at: Optional[float] = None
        self.start_requested_at: Optional[float] = None
        self.last_log_at: Optional[float] = None
        self.last_wait_log_at: Optional[float] = None
        self.auto_start_attempted = False
        self.state = "WAITING"

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub = self.create_publisher(
            String, "cmd_vel_distance_test/state", _LATCHED_QOS
        )
        self.result_pub = self.create_publisher(
            String, "cmd_vel_distance_test/result", _LATCHED_QOS
        )
        self.expected_pub = self.create_publisher(
            PoseStamped, "cmd_vel_distance_test/expected_pose", _LATCHED_QOS
        )
        self.actual_pub = self.create_publisher(
            PoseStamped, "cmd_vel_distance_test/actual_pose", _LATCHED_QOS
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl, 10
        )
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.cancel_client = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )
        self.create_service(Trigger, "cmd_vel_distance_test/start", self._on_start)
        self.create_service(Trigger, "cmd_vel_distance_test/stop", self._on_stop)
        self.create_timer(self.control_period_sec, self._on_tick)
        self._set_state("WAITING")

        duration = self.commanded_distance_m / self.linear_speed_mps
        self.get_logger().info(
            "Ready for isolated cmd_vel distance test: nominal start="
            f"({self.nominal_start_x:.2f}, {self.nominal_start_y:.2f}, "
            f"{self.nominal_start_yaw_deg:.1f}deg), command={self.linear_speed_mps:.3f}m/s "
            f"for {duration:.2f}s ({self.commanded_distance_m:.3f}m)"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "nominal_start_x": 0.8,
            "nominal_start_y": 0.2,
            "nominal_start_yaw_deg": 90.0,
            "commanded_distance_m": 0.5,
            "linear_speed_mps": 0.05,
            "control_period_sec": 0.05,
            "nav2_stop_delay_sec": 1.0,
            "settling_time_sec": 2.0,
            "start_position_tolerance_m": 0.2,
            "start_yaw_tolerance_deg": 15.0,
            "sample_timeout_sec": 1.0,
            "auto_start": True,
            "exit_after_completion": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        for name in (
            "nominal_start_x",
            "nominal_start_y",
            "nominal_start_yaw_deg",
        ):
            setattr(self, name, float(self.get_parameter(name).value))
        for name in (
            "commanded_distance_m",
            "linear_speed_mps",
            "control_period_sec",
            "nav2_stop_delay_sec",
            "settling_time_sec",
            "start_position_tolerance_m",
            "start_yaw_tolerance_deg",
            "sample_timeout_sec",
        ):
            value = float(self.get_parameter(name).value)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            setattr(self, name, value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.exit_after_completion = bool(
            self.get_parameter("exit_after_completion").value
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_amcl = msg
        self.last_amcl_received = self._now_ros()

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.last_odom_received = self._now_ros()

    def _on_start(self, request, response):
        del request
        if self.state in ("STARTING", "MOVING", "SETTLING"):
            response.success = False
            response.message = f"test already running in state {self.state}"
            return response
        reason = self._start_readiness_failure()
        if reason:
            response.success = False
            response.message = reason
            return response
        reason = self._begin_test()
        response.success = reason is None
        response.message = reason or (
            f"Nav2 stopping; movement begins after {self.nav2_stop_delay_sec:.1f}s"
        )
        return response

    def _begin_test(self) -> Optional[str]:
        measured = self._amcl_pose(self.latest_amcl)
        position_error = math.hypot(
            measured[0] - self.nominal_start_x,
            measured[1] - self.nominal_start_y,
        )
        nominal_yaw = math.radians(self.nominal_start_yaw_deg)
        yaw_error_deg = abs(math.degrees(normalize_angle(measured[2] - nominal_yaw)))
        if position_error > self.start_position_tolerance_m:
            return (
                f"start position error {position_error:.3f}m exceeds "
                f"{self.start_position_tolerance_m:.3f}m"
            )
        if yaw_error_deg > self.start_yaw_tolerance_deg:
            return (
                f"start yaw error {yaw_error_deg:.1f}deg exceeds "
                f"{self.start_yaw_tolerance_deg:.1f}deg"
            )

        self.cancel_client.call_async(CancelGoal.Request())
        self._publish_stop()
        self.start_amcl = None
        self.start_odom = None
        self.move_started_at = None
        self.move_started_ros = None
        self.move_stopped_at = None
        self.start_requested_at = time.monotonic()
        self.last_log_at = None
        self._set_state("STARTING")
        return None

    def _on_stop(self, request, response):
        del request
        self._publish_stop()
        self._set_state("STOPPED")
        response.success = True
        response.message = "cmd_vel distance test stopped"
        return response

    def _on_tick(self) -> None:
        if self.state == "WAITING" and self.auto_start and not self.auto_start_attempted:
            reason = self._start_readiness_failure()
            if reason:
                now = time.monotonic()
                if self.last_wait_log_at is None or now - self.last_wait_log_at >= 2.0:
                    self.get_logger().info(f"Waiting to auto-start: {reason}")
                    self.last_wait_log_at = now
                return
            self.auto_start_attempted = True
            reason = self._begin_test()
            if reason:
                self._fail(reason)
                return

        if self.state == "STARTING":
            self._publish_stop()
            if time.monotonic() - self.start_requested_at < self.nav2_stop_delay_sec:
                return
            reason = self._odom_failure()
            if reason:
                self._fail(reason)
                return
            self.start_amcl = self._amcl_pose(self.latest_amcl)
            self.start_odom = self._odom_pose(self.latest_odom)
            self.move_started_at = time.monotonic()
            self.move_started_ros = self._now_ros()
            self._set_state("MOVING")
            self.get_logger().info(
                f"Measured start AMCL=({self.start_amcl[0]:.3f}, "
                f"{self.start_amcl[1]:.3f}, {math.degrees(self.start_amcl[2]):.1f}deg)"
            )

        if self.state == "MOVING":
            reason = self._odom_failure()
            if reason:
                self._fail(reason)
                return
            elapsed = time.monotonic() - self.move_started_at
            duration = self.commanded_distance_m / self.linear_speed_mps
            if elapsed >= duration:
                self._publish_stop()
                self.move_stopped_at = time.monotonic()
                self._set_state("SETTLING")
                self.get_logger().info(
                    f"Forward command stopped after {elapsed:.3f}s; waiting "
                    f"{self.settling_time_sec:.1f}s for localization to settle"
                )
                return
            command = Twist()
            command.linear.x = self.linear_speed_mps
            self.cmd_pub.publish(command)
            if self.last_log_at is None or elapsed - self.last_log_at >= 1.0:
                self.get_logger().info(
                    f"MOVING elapsed={elapsed:.2f}/{duration:.2f}s "
                    f"cmd_vel=({self.linear_speed_mps:.3f}, 0.000)"
                )
                self.last_log_at = elapsed
            return

        if self.state == "SETTLING":
            self._publish_stop()
            if time.monotonic() - self.move_stopped_at < self.settling_time_sec:
                return
            reason = self._odom_failure()
            if reason:
                self._fail(reason)
                return
            self._finish()

    def _finish(self) -> None:
        actual_odom = self._odom_pose(self.latest_odom)
        odom_projected_pose = project_odom_pose_to_map(
            self.start_amcl, self.start_odom, actual_odom
        )
        amcl_updated = (
            self.last_amcl_received is not None
            and self.move_started_ros is not None
            and self.last_amcl_received > self.move_started_ros
        )
        actual_amcl = self._amcl_pose(self.latest_amcl) if amcl_updated else None
        actual_pose = actual_amcl if actual_amcl is not None else odom_projected_pose
        actual_pose_source = "amcl" if actual_amcl is not None else "odom_projected"
        metrics = calculate_distance_test_metrics(
            self.start_amcl, actual_pose, self.commanded_distance_m
        )
        odom_dx = actual_odom[0] - self.start_odom[0]
        odom_dy = actual_odom[1] - self.start_odom[1]
        odom_distance = math.hypot(odom_dx, odom_dy)
        command_duration = self.move_stopped_at - self.move_started_at
        result = {
            "commanded_distance_m": round(self.commanded_distance_m, 6),
            "linear_speed_mps": round(self.linear_speed_mps, 6),
            "command_duration_sec": round(command_duration, 6),
            "time_integrated_command_distance_m": round(
                self.linear_speed_mps * command_duration, 6
            ),
            "actual_pose_source": actual_pose_source,
            "start_amcl": self._rounded_pose(self.start_amcl),
            "expected_amcl": self._rounded_pose(metrics.expected_pose),
            "actual_pose": self._rounded_pose(metrics.actual_pose),
            "actual_amcl": (
                self._rounded_pose(actual_amcl) if actual_amcl is not None else None
            ),
            "odom_projected_pose": self._rounded_pose(odom_projected_pose),
            "actual_forward_m": round(metrics.actual_forward_m, 6),
            "actual_lateral_m": round(metrics.actual_lateral_m, 6),
            "forward_error_m": round(metrics.forward_error_m, 6),
            "lateral_error_m": round(metrics.lateral_error_m, 6),
            "position_error_m": round(metrics.position_error_m, 6),
            "yaw_error_deg": round(metrics.yaw_error_deg, 3),
            "odom_distance_m": round(odom_distance, 6),
            "odom_distance_error_m": round(odom_distance - self.commanded_distance_m, 6),
        }
        encoded = json.dumps(result, ensure_ascii=False, sort_keys=True)
        self.result_pub.publish(String(data=encoded))
        self._publish_pose(self.expected_pub, metrics.expected_pose)
        self._publish_pose(self.actual_pub, actual_pose)
        self._set_state("COMPLETE")
        self.get_logger().info(f"CMD_VEL_DISTANCE_RESULT {encoded}")

    def _start_readiness_failure(self) -> str:
        if self.latest_amcl is None or self.last_amcl_received is None:
            return "AMCL pose not received yet"
        reason = self._odom_failure()
        if reason:
            return reason
        if not self.cancel_client.service_is_ready():
            return "Nav2 cancel service not ready yet"
        return ""

    def _odom_failure(self) -> str:
        now = self._now_ros()
        if self.latest_odom is None or self.last_odom_received is None:
            return "odom missing"
        if now - self.last_odom_received > self.sample_timeout_sec:
            return "odom stale"
        return ""

    def _fail(self, reason: str) -> None:
        self._publish_stop()
        self._set_state("FAILED")
        self.get_logger().error(f"cmd_vel distance test failed: {reason}")

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _set_state(self, state: str) -> None:
        if state != self.state:
            self.get_logger().warning(f"cmd_vel_distance_test: {self.state} -> {state}")
        self.state = state
        self.state_pub.publish(String(data=state))

    def _publish_pose(self, publisher, pose: Pose2D) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        message.pose.orientation.z = math.sin(pose[2] * 0.5)
        message.pose.orientation.w = math.cos(pose[2] * 0.5)
        publisher.publish(message)

    def _now_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _amcl_pose(msg: PoseWithCovarianceStamped) -> Pose2D:
        pose = msg.pose.pose
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)

    @staticmethod
    def _odom_pose(msg: Odometry) -> Pose2D:
        pose = msg.pose.pose
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)

    @staticmethod
    def _rounded_pose(pose: Pose2D):
        return {
            "x": round(pose[0], 6),
            "y": round(pose[1], 6),
            "yaw_deg": round(math.degrees(pose[2]), 3),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelDistanceTest()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.exit_after_completion and node.state in (
                "COMPLETE",
                "FAILED",
                "STOPPED",
            ):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
