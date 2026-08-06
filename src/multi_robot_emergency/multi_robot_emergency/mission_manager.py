"""Select one of two robots and dispatch only that robot through Nav2."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
import time

from action_msgs.msg import GoalStatus
from aed_interfaces.msg import MissionAssignment, MissionStatus, RobotState
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .assignment import path_length, path_motion_cost


@dataclass
class RobotObservation:
    """Most recently received map pose for one robot."""

    pose: PoseStamped
    received_at: float


class EmergencyMissionManager(Node):
    """Compare both Nav2 plans and dispatch only the shorter candidate."""

    def __init__(self) -> None:
        """Create robot inputs, mission outputs, and Nav2 action clients."""
        super().__init__("emergency_mission_manager")
        self.declare_parameter("robot_ids", ["robot1", "robot2"])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("request_topic", "/emergency/request")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("pose_timeout_sec", 15.0)
        self.declare_parameter("allow_stale_pose", True)
        self.declare_parameter("use_planner_start", True)
        self.declare_parameter("docked_start_offset_m", 0.35)
        self.declare_parameter("planning_timeout_sec", 30.0)
        self.declare_parameter("dispatch_retry_timeout_sec", 15.0)
        self.declare_parameter("assignment_ack_timeout_sec", 3.0)
        self.declare_parameter("nominal_linear_speed_mps", 0.20)
        self.declare_parameter("nominal_angular_speed_radps", 0.70)
        self.declare_parameter("slowdown_turn_threshold_deg", 45.0)
        self.declare_parameter("slowdown_penalty_sec", 4.0)
        self.declare_parameter("path_simplification_tolerance_m", 0.10)
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("dispatch_enabled", False)
        self.declare_parameter("automatic_request", False)
        self.declare_parameter("automatic_request_delay_sec", 3.0)
        self.declare_parameter("initial_target_x", 1.2)
        self.declare_parameter("initial_target_y", 2.4)
        self.declare_parameter("initial_target_yaw", 0.0)

        self.robot_ids = [
            str(value) for value in self.get_parameter("robot_ids").value
        ]
        if len(self.robot_ids) != 2 or len(set(self.robot_ids)) != 2:
            raise ValueError("robot_ids must contain exactly two unique IDs")
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.pose_timeout = float(
            self.get_parameter("pose_timeout_sec").value
        )
        self.allow_stale_pose = bool(
            self.get_parameter("allow_stale_pose").value
        )
        self.use_planner_start = bool(
            self.get_parameter("use_planner_start").value
        )
        self.docked_start_offset = float(
            self.get_parameter("docked_start_offset_m").value
        )
        self.planning_timeout = float(
            self.get_parameter("planning_timeout_sec").value
        )
        self.dispatch_retry_timeout = float(
            self.get_parameter("dispatch_retry_timeout_sec").value
        )
        self.assignment_ack_timeout = float(
            self.get_parameter("assignment_ack_timeout_sec").value
        )
        self.nominal_linear_speed = float(
            self.get_parameter("nominal_linear_speed_mps").value
        )
        self.nominal_angular_speed = float(
            self.get_parameter("nominal_angular_speed_radps").value
        )
        self.slowdown_turn_threshold = math.radians(
            float(self.get_parameter("slowdown_turn_threshold_deg").value)
        )
        self.slowdown_penalty = float(
            self.get_parameter("slowdown_penalty_sec").value
        )
        self.path_simplification_tolerance = float(
            self.get_parameter("path_simplification_tolerance_m").value
        )
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.dispatch_enabled = bool(
            self.get_parameter("dispatch_enabled").value
        )
        if self.pose_timeout <= 0.0:
            raise ValueError("pose_timeout_sec must be positive")
        if self.planning_timeout <= 0.0:
            raise ValueError("planning_timeout_sec must be positive")
        if self.dispatch_retry_timeout <= 0.0:
            raise ValueError("dispatch_retry_timeout_sec must be positive")
        if self.assignment_ack_timeout <= 0.0:
            raise ValueError("assignment_ack_timeout_sec must be positive")
        if self.nominal_linear_speed <= 0.0:
            raise ValueError("nominal_linear_speed_mps must be positive")
        if self.nominal_angular_speed <= 0.0:
            raise ValueError("nominal_angular_speed_radps must be positive")
        if self.slowdown_turn_threshold < 0.0:
            raise ValueError(
                "slowdown_turn_threshold_deg must be non-negative"
            )
        if self.slowdown_penalty < 0.0:
            raise ValueError("slowdown_penalty_sec must be non-negative")
        if (
            not math.isfinite(self.path_simplification_tolerance)
            or self.path_simplification_tolerance < 0.0
        ):
            raise ValueError(
                "path_simplification_tolerance_m must be finite and "
                "non-negative"
            )
        if self.docked_start_offset < 0.0:
            raise ValueError("docked_start_offset_m must be non-negative")

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, "/emergency/status", latched_qos
        )
        self.selected_publisher = self.create_publisher(
            String, "/emergency/selected_robot", latched_qos
        )
        self.target_marker_publisher = self.create_publisher(
            Marker, "/emergency/target_marker", latched_qos
        )
        self.robot_marker_publisher = self.create_publisher(
            MarkerArray, "/emergency/robot_markers", latched_qos
        )
        self.path_publishers = {
            robot_id: self.create_publisher(
                Path,
                f"/emergency/candidate_path/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        self.distance_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/path_distance/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        self.predicted_eta_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/eta/predicted/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        self.actual_eta_publishers = {
            robot_id: self.create_publisher(
                Float32,
                f"/emergency/eta/actual/{robot_id}",
                latched_qos,
            )
            for robot_id in self.robot_ids
        }
        eta_result_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.eta_result_publisher = self.create_publisher(
            String, "/emergency/eta/result", eta_result_qos
        )

        self.observations: dict[str, RobotObservation] = {}
        self.dock_states: dict[str, bool] = {}
        self.assignment_publishers = {
            robot_id: self.create_publisher(
                MissionAssignment,
                f"/{robot_id}/mission_assignment",
                10,
            )
            for robot_id in self.robot_ids
        }
        self.mission_status_subscription = self.create_subscription(
            MissionStatus,
            "/aed/mission_status",
            self._on_mission_status,
            20,
        )
        self.planner_clients = {
            robot_id: ActionClient(
                self,
                ComputePathToPose,
                f"/{robot_id}/compute_path_to_pose",
            )
            for robot_id in self.robot_ids
        }
        self.pose_subscriptions = [
            self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{robot_id}/amcl_pose",
                lambda message, rid=robot_id: self._on_pose(rid, message),
                # AMCL publishes a transient-local pose. Matching that QoS
                # gives a newly started central manager the last known pose
                # even while the robot is stationary.
                latched_qos,
            )
            for robot_id in self.robot_ids
        ]
        self.dock_subscriptions = [
            self.create_subscription(
                DockStatus,
                f"/{robot_id}/dock_status",
                lambda message, rid=robot_id: self._on_dock_status(
                    rid, message
                ),
                latched_qos,
            )
            for robot_id in self.robot_ids
        ]
        self.request_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("request_topic").value),
            self._on_request,
            10,
        )
        global_click_topic = str(
            self.get_parameter("clicked_point_topic").value
        )
        click_topics = [global_click_topic]
        click_topics.extend(
            f"/{robot_id}/clicked_point" for robot_id in self.robot_ids
        )
        self.clicked_point_subscriptions = [
            self.create_subscription(
                PointStamped,
                topic,
                self._on_clicked_point,
                10,
            )
            for topic in dict.fromkeys(click_topics)
        ]
        self.marker_timer = self.create_timer(0.5, self._publish_robot_markers)

        self.request_serial = 0
        self.navigation_active = False
        self.planning_active = False
        self.pending_plans: set[str] = set()
        self.plan_results: dict[
            str, tuple[Path, float, float, float, int]
        ] = {}
        self.plan_failures: dict[str, str] = {}
        self.planning_target: PoseStamped | None = None
        self.planning_timer = None
        self.selected_robot = ""
        self.ranked_candidates: list[str] = []
        self.excluded_robots: set[str] = set()
        self.assignment_version = 0
        self.assignment_ack_timer = None
        self.active_request_id = ""
        self.state = "IDLE"
        self._last_feedback_log = 0.0
        self.navigation_started_at: float | None = None
        self.navigation_predicted_eta: float | None = None
        self._automatic_request_timer = None
        if bool(self.get_parameter("automatic_request").value):
            delay = float(
                self.get_parameter("automatic_request_delay_sec").value
            )
            self._automatic_request_timer = self.create_timer(
                max(0.1, delay), self._publish_automatic_request
            )

        self._publish_status("IDLE", "waiting for an emergency request")
        self.get_logger().info(
            "Emergency request: ros2 topic pub --once /emergency/request "
            "geometry_msgs/msg/PoseStamped "
            "'{header: {frame_id: map}, pose: {position: {x: 1.2, "
            "y: 2.4}, orientation: {w: 1.0}}}'"
        )
        self.get_logger().info(
            f"Dispatch enabled: {self.dispatch_enabled}; "
            f"robots={self.robot_ids}"
        )
        self.get_logger().info(
            "RViz Publish Point topics: " + ", ".join(click_topics)
        )

    def _on_pose(
        self, robot_id: str, message: PoseWithCovarianceStamped
    ) -> None:
        if (
            message.header.frame_id
            and message.header.frame_id != self.map_frame
        ):
            self.get_logger().warning(
                f"Ignoring {robot_id} pose in frame "
                f"{message.header.frame_id!r}; expected {self.map_frame!r}"
            )
            return
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.header.frame_id = self.map_frame
        pose.pose = deepcopy(message.pose.pose)
        self.observations[robot_id] = RobotObservation(
            pose=pose,
            received_at=time.monotonic(),
        )

    def _on_dock_status(self, robot_id: str, message: DockStatus) -> None:
        self.dock_states[robot_id] = bool(message.is_docked)

    def _on_request(self, message: PoseStamped) -> None:
        request = deepcopy(message)
        if not request.header.frame_id:
            request.header.frame_id = self.map_frame
        if request.header.frame_id != self.map_frame:
            self._publish_status(
                "FAILED",
                f"request frame must be {self.map_frame}, got "
                f"{request.header.frame_id}",
            )
            return
        if not self._finite_pose(request):
            self._publish_status(
                "FAILED", "request pose contains non-finite data"
            )
            return
        if self.planning_active or self.navigation_active:
            self.get_logger().warning(
                "Ignoring a new emergency request while a mission is active"
            )
            return

        self.request_serial += 1
        self.assignment_version = 0
        self.ranked_candidates.clear()
        self.excluded_robots.clear()
        request.header.stamp = self.get_clock().now().to_msg()
        self.active_request_id = f"emergency-{self.request_serial:03d}"
        self._publish_status("EMERGENCY_RECEIVED", self.active_request_id)
        self._publish_target_marker(request)
        self._calculate_and_assign(request)

    def _on_clicked_point(self, message: PointStamped) -> None:
        """Convert an RViz Publish Point click into a planning request."""
        request = PoseStamped()
        request.header = deepcopy(message.header)
        request.pose.position = deepcopy(message.point)
        request.pose.orientation.w = 1.0
        self.get_logger().info(
            "RViz point received: "
            f"x={message.point.x:.3f}, y={message.point.y:.3f}"
        )
        self._on_request(request)

    def _calculate_and_assign(self, target: PoseStamped) -> None:
        self._publish_status("CALCULATING", "requesting both Nav2 paths")
        now = time.monotonic()
        self.planning_active = True
        self.planning_target = deepcopy(target)
        self.pending_plans.clear()
        self.plan_results.clear()
        self.plan_failures.clear()
        for robot_id in self.robot_ids:
            self._publish_distance(robot_id, math.nan)
            self._publish_predicted_eta(robot_id, math.nan)
            self._publish_actual_eta(robot_id, math.nan)

        for robot_id in self.robot_ids:
            client = self.planner_clients[robot_id]
            if not client.wait_for_server(timeout_sec=1.0):
                self.plan_failures[robot_id] = (
                    f"/{robot_id}/compute_path_to_pose unavailable"
                )
                continue

            goal = ComputePathToPose.Goal()
            goal.goal = deepcopy(target)
            stamp = self.get_clock().now().to_msg()
            goal.goal.header.stamp = stamp
            goal.planner_id = self.planner_id
            docked = self.dock_states.get(robot_id, False)
            goal.use_start = docked or not self.use_planner_start
            if goal.use_start:
                observation = self.observations.get(robot_id)
                if observation is None:
                    self.plan_failures[robot_id] = "no AMCL pose"
                    continue
                age = now - observation.received_at
                if age > self.pose_timeout:
                    if not self.allow_stale_pose:
                        self.plan_failures[robot_id] = (
                            f"stale pose {age:.1f}s"
                        )
                        continue
                    self.get_logger().warning(
                        f"{robot_id}: using last AMCL pose "
                        f"({age:.1f}s old)"
                    )
                goal.start = deepcopy(observation.pose)
                if docked:
                    goal.start = self._project_undocked_start(goal.start)
                    self.get_logger().info(
                        f"{robot_id}: docked; plan from predicted "
                        f"undocked pose ({goal.start.pose.position.x:.2f}, "
                        f"{goal.start.pose.position.y:.2f})"
                    )
                goal.start.header.stamp = stamp
            self.pending_plans.add(robot_id)
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda response, rid=robot_id, serial=self.request_serial:
                self._on_plan_response(rid, serial, response)
            )

        for robot_id in self.plan_failures:
            self.path_publishers[robot_id].publish(Path())

        if not self.pending_plans:
            self._finish_planning(self.request_serial)
            return
        self.planning_timer = self.create_timer(
            self.planning_timeout,
            lambda serial=self.request_serial:
            self._on_planning_timeout(serial),
        )

    def _on_plan_response(self, robot_id: str, serial: int, future) -> None:
        if serial != self.request_serial or robot_id not in self.pending_plans:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._record_plan_failure(
                robot_id, serial, f"request error: {error}"
            )
            return
        if not handle.accepted:
            self._record_plan_failure(
                robot_id, serial, "planner rejected goal"
            )
            return
        handle.get_result_async().add_done_callback(
            lambda result, rid=robot_id: self._on_plan_result(
                rid, serial, result
            )
        )

    def _on_plan_result(self, robot_id: str, serial: int, future) -> None:
        if serial != self.request_serial or robot_id not in self.pending_plans:
            return
        try:
            wrapped_result = future.result()
            path = wrapped_result.result.path
            status = int(wrapped_result.status)
        except Exception as error:
            self._record_plan_failure(
                robot_id, serial, f"result error: {error}"
            )
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._record_plan_failure(
                robot_id, serial, f"planner result status={status}"
            )
            return
        if not path.poses:
            self._record_plan_failure(
                robot_id, serial, "planner returned empty path"
            )
            return

        try:
            points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in path.poses
            ]
            distance = path_length(points)
            observation = self.observations.get(robot_id)
            initial_yaw = (
                self._quaternion_yaw(observation.pose)
                if observation is not None
                else None
            )
            final_yaw = (
                self._quaternion_yaw(self.planning_target)
                if self.planning_target is not None
                else None
            )
            eta, turn_angle, slowdown_count = path_motion_cost(
                points,
                linear_speed=self.nominal_linear_speed,
                angular_speed=self.nominal_angular_speed,
                slowdown_turn_threshold=self.slowdown_turn_threshold,
                slowdown_penalty=self.slowdown_penalty,
                simplification_tolerance=(
                    self.path_simplification_tolerance
                ),
                initial_yaw=initial_yaw,
                final_yaw=final_yaw,
            )
        except ValueError as error:
            self._record_plan_failure(robot_id, serial, str(error))
            return

        self.plan_results[robot_id] = (
            path,
            distance,
            eta,
            turn_angle,
            slowdown_count,
        )
        self.path_publishers[robot_id].publish(path)
        self._publish_distance(robot_id, distance)
        self._publish_predicted_eta(robot_id, eta)
        self.pending_plans.discard(robot_id)
        self.get_logger().info(
            f"Candidate {robot_id}: distance={distance:.2f}m, "
            f"turn={math.degrees(turn_angle):.1f}deg, "
            f"slowdowns={slowdown_count}, eta={eta:.2f}s"
        )
        if not self.pending_plans:
            self._finish_planning(serial)

    def _record_plan_failure(
        self, robot_id: str, serial: int, reason: str
    ) -> None:
        if serial != self.request_serial or robot_id not in self.pending_plans:
            return
        self.pending_plans.discard(robot_id)
        self.plan_failures[robot_id] = reason
        self.path_publishers[robot_id].publish(Path())
        self._publish_distance(robot_id, math.nan)
        self._publish_predicted_eta(robot_id, math.nan)
        self.get_logger().warning(f"Candidate {robot_id} excluded: {reason}")
        if not self.pending_plans:
            self._finish_planning(serial)

    def _on_planning_timeout(self, serial: int) -> None:
        if serial != self.request_serial or not self.planning_active:
            return
        for robot_id in tuple(self.pending_plans):
            self.plan_failures[robot_id] = (
                f"planning timeout after {self.planning_timeout:.1f}s"
            )
            self.path_publishers[robot_id].publish(Path())
            self._publish_distance(robot_id, math.nan)
            self._publish_predicted_eta(robot_id, math.nan)
        self.pending_plans.clear()
        self._finish_planning(serial)

    def _finish_planning(self, serial: int) -> None:
        if serial != self.request_serial or not self.planning_active:
            return
        self.planning_active = False
        if self.planning_timer is not None:
            timer = self.planning_timer
            self.planning_timer = None
            timer.cancel()
            self.destroy_timer(timer)

        if not self.plan_results:
            self.selected_robot = ""
            self._publish_selected_robot("")
            detail = "; ".join(
                f"{robot_id}: {reason}"
                for robot_id, reason in self.plan_failures.items()
            )
            self._publish_status("FAILED", detail or "no valid Nav2 path")
            return

        ranked = sorted(
            (
                (robot_id, result[2])
                for robot_id, result in self.plan_results.items()
            ),
            key=lambda item: (item[1], item[0]),
        )
        self.ranked_candidates = [robot_id for robot_id, _ in ranked]
        self.selected_robot = self.ranked_candidates[0]
        self._publish_selected_robot(self.selected_robot)
        detail = ", ".join(
            (
                f"{robot_id}={score:.2f}s"
                f"({self.plan_results[robot_id][1]:.2f}m, "
                f"{math.degrees(self.plan_results[robot_id][3]):.1f}deg, "
                f"slow={self.plan_results[robot_id][4]})"
            )
            for robot_id, score in ranked
        )
        if self.plan_failures:
            detail += "; excluded: " + ", ".join(
                f"{robot_id}({reason})"
                for robot_id, reason in self.plan_failures.items()
            )
        self._publish_status(
            "ASSIGNED", f"selected={self.selected_robot}; {detail}"
        )

        if not self.dispatch_enabled:
            self.get_logger().warning(
                "Dispatch is disabled. Set dispatch_enabled:=true to move "
                "the selected robot."
            )
            return
        self._publish_assignment(self.selected_robot)

    def _publish_assignment(self, robot_id: str) -> None:
        if self.planning_target is None:
            self.navigation_active = False
            self._publish_status("FAILED", "planning target was lost")
            return
        self.assignment_version += 1
        self.selected_robot = robot_id
        self.navigation_active = True
        self.navigation_started_at = None
        self.navigation_predicted_eta = self.plan_results[robot_id][2]
        self._publish_selected_robot(robot_id)
        assignment = MissionAssignment()
        assignment.mission_id = f"{self.active_request_id}-aed"
        assignment.event_id = self.active_request_id
        assignment.robot_id = robot_id
        assignment.role = RobotState.ROLE_AED_DELIVERY
        assignment.target = deepcopy(self.planning_target)
        assignment.assigned_at = self.get_clock().now().to_msg()
        assignment.assignment_version = self.assignment_version
        assignment.cancel_previous = True
        self.assignment_publishers[robot_id].publish(assignment)
        self._start_assignment_ack_timer(
            self.active_request_id,
            robot_id,
            self.assignment_version,
        )
        self._publish_status(
            "DISPATCHING",
            f"assignment v{self.assignment_version} published to {robot_id}",
        )

    def _on_mission_status(self, status: MissionStatus) -> None:
        if status.event_id != self.active_request_id:
            return
        if status.robot_id != self.selected_robot:
            return
        if status.assignment_version != self.assignment_version:
            return

        self._cancel_assignment_ack_timer()

        if status.status == MissionStatus.DISPATCHING:
            self._publish_status(
                "DISPATCHING", status.reason or status.robot_id
            )
            return
        if status.status == MissionStatus.EN_ROUTE:
            if self.navigation_started_at is None:
                self.navigation_started_at = time.monotonic()
            self._publish_status("NAVIGATING", f"{status.robot_id} is moving")
            return
        if status.status in (MissionStatus.ARRIVED, MissionStatus.COMPLETED):
            self.navigation_active = False
            if self.navigation_started_at is not None:
                actual = time.monotonic() - self.navigation_started_at
                predicted = self.navigation_predicted_eta
                if predicted is not None:
                    self._publish_actual_eta(status.robot_id, actual)
                    self._publish_eta_result(
                        status.robot_id, predicted, actual
                    )
                    self.get_logger().info(
                        f"ETA measurement {status.robot_id}: "
                        f"predicted={predicted:.2f}s, actual={actual:.2f}s, "
                        f"error={actual - predicted:+.2f}s"
                    )
            self.navigation_started_at = None
            self.navigation_predicted_eta = None
            self._publish_status(
                "ARRIVED", f"{status.robot_id} reached the emergency"
            )
            self._publish_status("COMPLETED", self.active_request_id)
            return
        if status.status not in {
            MissionStatus.CANCELED,
            MissionStatus.BLOCKED,
            MissionStatus.NETWORK_LOST,
            MissionStatus.NAVIGATION_ERROR,
        }:
            return

        self._reassign_after_failure(status.robot_id, status.reason)

    def _start_assignment_ack_timer(
        self, event_id: str, robot_id: str, assignment_version: int
    ) -> None:
        self._cancel_assignment_ack_timer()
        self.assignment_ack_timer = self.create_timer(
            self.assignment_ack_timeout,
            lambda: self._on_assignment_ack_timeout(
                event_id, robot_id, assignment_version
            ),
        )

    def _cancel_assignment_ack_timer(self) -> None:
        if self.assignment_ack_timer is None:
            return
        timer = self.assignment_ack_timer
        self.assignment_ack_timer = None
        timer.cancel()
        self.destroy_timer(timer)

    def _on_assignment_ack_timeout(
        self, event_id: str, robot_id: str, assignment_version: int
    ) -> None:
        self._cancel_assignment_ack_timer()
        if (
            event_id != self.active_request_id
            or robot_id != self.selected_robot
            or assignment_version != self.assignment_version
            or not self.navigation_active
        ):
            return
        self._reassign_after_failure(
            robot_id,
            f"mission executor did not acknowledge within "
            f"{self.assignment_ack_timeout:.1f}s",
        )

    def _reassign_after_failure(self, failed_robot: str, reason: str) -> None:
        if self.navigation_started_at is not None:
            elapsed = time.monotonic() - self.navigation_started_at
            self.get_logger().warning(
                f"ETA measurement {failed_robot}: aborted after "
                f"{elapsed:.2f}s ({reason or 'unspecified failure'})"
            )
        self.navigation_started_at = None
        self.navigation_predicted_eta = None
        self.excluded_robots.add(failed_robot)
        next_robot = next(
            (
                robot_id
                for robot_id in self.ranked_candidates
                if robot_id not in self.excluded_robots
            ),
            None,
        )
        if next_robot is None:
            self.navigation_active = False
            self._publish_status(
                "FAILED",
                f"all candidates failed; last={failed_robot}: {reason}",
            )
            return
        self._publish_status(
            "REASSIGNING",
            f"exclude {failed_robot}; assigning {next_robot}",
        )
        self._publish_assignment(next_robot)

    def _publish_automatic_request(self) -> None:
        if self._automatic_request_timer is None:
            return
        self._automatic_request_timer.cancel()
        self._automatic_request_timer = None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = float(
            self.get_parameter("initial_target_x").value
        )
        pose.pose.position.y = float(
            self.get_parameter("initial_target_y").value
        )
        yaw = float(self.get_parameter("initial_target_yaw").value)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        self._on_request(pose)

    def _publish_target_marker(self, target: PoseStamped) -> None:
        marker = Marker()
        marker.header = deepcopy(target.header)
        marker.ns = "emergency_target"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = deepcopy(target.pose)
        marker.pose.position.z = 0.15
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.30
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 0.95
        self.target_marker_publisher.publish(marker)

    def _publish_robot_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        colors = ((0.1, 0.45, 1.0), (0.1, 0.9, 0.25))
        for index, robot_id in enumerate(self.robot_ids):
            observation = self.observations.get(robot_id)
            if observation is None:
                continue
            position = observation.pose.pose.position
            selected = robot_id == self.selected_robot
            body = Marker()
            body.header.frame_id = self.map_frame
            body.header.stamp = stamp
            body.ns = "robot_positions"
            body.id = index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose = deepcopy(observation.pose.pose)
            body.pose.position.z = 0.12
            body.scale.x = 0.42 if selected else 0.32
            body.scale.y = 0.42 if selected else 0.32
            body.scale.z = 0.24
            if selected:
                body.color.r, body.color.g, body.color.b = (1.0, 0.8, 0.0)
            else:
                body.color.r, body.color.g, body.color.b = colors[index]
            body.color.a = 0.9
            markers.markers.append(body)

            label = Marker()
            label.header = deepcopy(body.header)
            label.ns = "robot_labels"
            label.id = 100 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = position.x
            label.pose.position.y = position.y
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            suffix = " [SELECTED]" if selected else ""
            label.text = f"{robot_id}{suffix}"
            markers.markers.append(label)
        self.robot_marker_publisher.publish(markers)

    def _publish_selected_robot(self, robot_id: str) -> None:
        message = String()
        message.data = robot_id
        self.selected_publisher.publish(message)

    def _publish_distance(self, robot_id: str, distance: float) -> None:
        message = Float32()
        message.data = float(distance)
        self.distance_publishers[robot_id].publish(message)

    def _publish_predicted_eta(self, robot_id: str, eta: float) -> None:
        message = Float32()
        message.data = float(eta)
        self.predicted_eta_publishers[robot_id].publish(message)

    def _publish_actual_eta(self, robot_id: str, elapsed: float) -> None:
        message = Float32()
        message.data = float(elapsed)
        self.actual_eta_publishers[robot_id].publish(message)

    def _publish_eta_result(
        self, robot_id: str, predicted: float, actual: float
    ) -> None:
        stamp = self.get_clock().now()
        message = String()
        message.data = json.dumps(
            {
                "request_id": self.active_request_id,
                "robot_id": robot_id,
                "predicted_eta_sec": round(predicted, 3),
                "actual_arrival_sec": round(actual, 3),
                "error_sec": round(actual - predicted, 3),
                "status": "ARRIVED",
                "stamp_sec": round(stamp.nanoseconds / 1e9, 9),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.eta_result_publisher.publish(message)

    def _publish_status(self, state: str, detail: str) -> None:
        self.state = state
        message = String()
        message.data = (
            f"{state}|request={self.active_request_id}|"
            f"selected={self.selected_robot}|{detail}"
        )
        self.status_publisher.publish(message)
        self.get_logger().info(message.data)

    @staticmethod
    def _finite_pose(pose: PoseStamped) -> bool:
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _quaternion_yaw(pose: PoseStamped) -> float:
        orientation = pose.pose.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _project_undocked_start(self, pose: PoseStamped) -> PoseStamped:
        """Move a docked pose backward to the expected post-undock pose."""
        result = deepcopy(pose)
        yaw = self._quaternion_yaw(result)
        result.pose.position.x -= self.docked_start_offset * math.cos(yaw)
        result.pose.position.y -= self.docked_start_offset * math.sin(yaw)
        return result


def main(args=None) -> None:
    """Run the emergency mission manager until ROS shuts down."""
    rclpy.init(args=args)
    node = EmergencyMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
