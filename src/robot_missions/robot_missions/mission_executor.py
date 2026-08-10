"""Execute assigned Multi-AMR missions through Nav2."""

from copy import deepcopy
import math
import time

from action_msgs.msg import GoalStatus
from aed_interfaces.msg import MissionAssignment, MissionStatus, RobotState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


_FALLBACK_TAKEOVER_STATES = frozenset({
    "STARTING",
    "ACTIVE",
    "BLOCKED",
    "RECOVERING",
})


def fallback_takes_ownership(state: str) -> bool:
    """Return whether sensor recovery currently owns robot motion."""
    return state.strip().upper() in _FALLBACK_TAKEOVER_STATES


def lidar_blocks_nav2(state: str, recovery_ready: bool) -> bool:
    """Return whether a new Nav2 goal must wait for sensor recovery."""
    state_blocks = state.strip().upper() in {"FAULT", "RECOVERING"}
    return state_blocks or not recovery_ready


def angle_distance(first: float, second: float) -> float:
    """Return the shortest absolute angular separation in radians."""
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


def pose_has_progress(
    previous: tuple[float, float, float] | None,
    current: tuple[float, float, float],
    *,
    translation_epsilon: float,
    rotation_epsilon: float,
) -> bool:
    """Treat either translation or in-place rotation as Nav2 progress."""
    if previous is None:
        return True
    translation = math.hypot(
        current[0] - previous[0], current[1] - previous[1]
    )
    rotation = angle_distance(previous[2], current[2])
    return translation >= translation_epsilon or rotation >= rotation_epsilon


def position_within_radius(
    current: tuple[float, float],
    target: tuple[float, float],
    radius: float,
) -> bool:
    """Return whether the current position is inside a planar goal radius."""
    return math.hypot(current[0] - target[0], current[1] - target[1]) <= radius


class MissionExecutor(Node):
    """Translate mission assignments into cancelable Nav2 goals."""

    def __init__(self) -> None:
        super().__init__("mission_executor")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("assignment_topic", "mission_assignment")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("dispatch_retry_timeout_sec", 15.0)
        self.declare_parameter("dispatch_retry_interval_sec", 0.5)
        self.declare_parameter("cancel_settle_sec", 0.5)
        self.declare_parameter("blocked_timeout_sec", 8.0)
        self.declare_parameter("progress_epsilon_m", 0.05)
        self.declare_parameter("progress_translation_epsilon_m", 0.03)
        self.declare_parameter("progress_rotation_epsilon_deg", 8.0)
        self.declare_parameter("return_arrival_tolerance_m", 0.15)

        self.robot_id = str(self.get_parameter("robot_id").value)
        if not self.robot_id:
            raise ValueError("robot_id parameter is required")
        self.dispatch_retry_timeout = float(
            self.get_parameter("dispatch_retry_timeout_sec").value
        )
        self.dispatch_retry_interval = float(
            self.get_parameter("dispatch_retry_interval_sec").value
        )
        self.cancel_settle = float(
            self.get_parameter("cancel_settle_sec").value
        )
        self.blocked_timeout = float(
            self.get_parameter("blocked_timeout_sec").value
        )
        self.progress_epsilon = float(
            self.get_parameter("progress_epsilon_m").value
        )
        self.progress_translation_epsilon = float(
            self.get_parameter("progress_translation_epsilon_m").value
        )
        self.progress_rotation_epsilon = math.radians(
            float(self.get_parameter("progress_rotation_epsilon_deg").value)
        )
        self.return_arrival_tolerance = float(
            self.get_parameter("return_arrival_tolerance_m").value
        )
        if self.dispatch_retry_timeout <= 0.0:
            raise ValueError("dispatch_retry_timeout_sec must be positive")
        if self.dispatch_retry_interval <= 0.0:
            raise ValueError("dispatch_retry_interval_sec must be positive")
        if self.cancel_settle < 0.0:
            raise ValueError("cancel_settle_sec must be non-negative")
        if self.blocked_timeout <= 0.0:
            raise ValueError("blocked_timeout_sec must be positive")
        if self.progress_epsilon < 0.0:
            raise ValueError("progress_epsilon_m must be non-negative")
        if self.progress_translation_epsilon < 0.0:
            raise ValueError(
                "progress_translation_epsilon_m must be non-negative"
            )
        if self.progress_rotation_epsilon < 0.0:
            raise ValueError(
                "progress_rotation_epsilon_deg must be non-negative"
            )
        if self.return_arrival_tolerance <= 0.0:
            raise ValueError("return_arrival_tolerance_m must be positive")

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
        )
        self.status_publisher = self.create_publisher(
            MissionStatus, "/aed/mission_status", 10
        )
        recovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MissionAssignment,
            str(self.get_parameter("assignment_topic").value),
            self._on_assignment,
            10,
        )
        self.create_subscription(
            String,
            f"/{self.robot_id}/lidar_state",
            self._on_lidar_state,
            recovery_qos,
        )
        self.create_subscription(
            String,
            f"/{self.robot_id}/fallback_state",
            self._on_fallback_state,
            recovery_qos,
        )
        self.create_subscription(
            Bool,
            f"/{self.robot_id}/recovery_ready",
            self._on_recovery_ready,
            recovery_qos,
        )

        self.assignment = None
        self.goal_handle = None
        self.goal_serial = 0
        self.pending_pose = None
        self.retry_deadline = 0.0
        self.retry_timer = None
        self.last_distance = None
        self.last_feedback_pose: tuple[float, float, float] | None = None
        self.last_progress_at = 0.0
        self.blocked_reported = False
        self.return_arrival_reported = False
        self.lidar_state = "UNKNOWN"
        self.fallback_state = "UNKNOWN"
        self.fallback_takeover_active = False
        self.fallback_resume_requested = False
        self.fallback_terminal_reported = False
        self.recovery_ready = True
        self.watchdog_timer = self.create_timer(1.0, self._check_progress)

    def _on_assignment(self, assignment: MissionAssignment) -> None:
        if assignment.robot_id != self.robot_id:
            return
        if not assignment.mission_id:
            self.get_logger().warning("Ignoring assignment without mission_id")
            return
        if (
            self.assignment is not None
            and assignment.event_id == self.assignment.event_id
            and assignment.assignment_version
            <= self.assignment.assignment_version
            and (
                assignment.assigned_at.sec,
                assignment.assigned_at.nanosec,
            )
            <= (
                self.assignment.assigned_at.sec,
                self.assignment.assigned_at.nanosec,
            )
        ):
            self.get_logger().warning(
                "Ignoring duplicate or stale assignment "
                f"v{assignment.assignment_version}"
            )
            return

        self.goal_serial += 1
        self.assignment = assignment
        self._cancel_retry_timer()
        self.pending_pose = deepcopy(assignment.target)
        self.retry_deadline = time.monotonic() + self.dispatch_retry_timeout
        self.last_distance = None
        self.last_feedback_pose = None
        self.last_progress_at = time.monotonic()
        self.blocked_reported = False
        self.return_arrival_reported = False
        self.fallback_resume_requested = False
        self.fallback_terminal_reported = False
        self.fallback_takeover_active = fallback_takes_ownership(
            self.fallback_state
        )
        self._publish_status(
            MissionStatus.DISPATCHING, "assignment received"
        )
        if assignment.cancel_previous and self.goal_handle is not None:
            old_goal_handle = self.goal_handle
            self.goal_handle = None
            cancel_future = old_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future, serial=self.goal_serial:
                self._on_cancel_complete(future, serial)
            )
            return
        if self._goal_blocked_for_recovery():
            self._publish_recovery_wait("assignment held until LiDAR is safe")
            return
        self._send_goal(self.pending_pose, self.goal_serial)

    def _on_cancel_complete(self, future, serial: int) -> None:
        """Wait briefly after Nav2 confirms cancellation before replacement."""
        if serial != self.goal_serial or self.pending_pose is None:
            return
        try:
            future.result()
        except Exception as error:
            self.get_logger().warning(
                f"Previous goal cancel response failed: {error}"
            )
        self.get_logger().info(
            f"Previous goal canceled; waiting {self.cancel_settle:.1f}s "
            "before replacement goal"
        )
        self._cancel_retry_timer()
        if self._goal_blocked_for_recovery():
            self._publish_recovery_wait(
                "replacement goal held until LiDAR is safe"
            )
            return
        if self.cancel_settle == 0.0:
            self._send_goal(self.pending_pose, serial)
            return
        self.retry_timer = self.create_timer(
            self.cancel_settle,
            lambda: self._retry_pending_goal(serial),
        )

    def _send_goal(self, pose, serial: int) -> None:
        if not pose.header.frame_id:
            self._publish_status(
                MissionStatus.NAVIGATION_ERROR, "target frame_id is empty"
            )
            return
        if not self.action_client.wait_for_server(timeout_sec=0.2):
            self._retry_or_fail(serial, "Nav2 action unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_feedback(
                feedback, serial
            ),
        )
        future.add_done_callback(
            lambda response: self._goal_response(response, serial)
        )

    def _goal_response(self, future, serial: int) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._retry_or_fail(serial, f"goal send error: {error}")
            return
        if serial != self.goal_serial:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._retry_or_fail(
                serial, "goal rejected while Nav2 is activating"
            )
            return

        self.pending_pose = None
        self._cancel_retry_timer()
        self.goal_handle = handle
        self.last_feedback_pose = None
        self.last_progress_at = time.monotonic()
        self._publish_status(MissionStatus.EN_ROUTE)
        handle.get_result_async().add_done_callback(
            lambda result: self._navigation_done(result, serial)
        )

    def _navigation_done(self, future, serial: int) -> None:
        if serial != self.goal_serial:
            return
        self.goal_handle = None
        if self.return_arrival_reported:
            return
        try:
            status = future.result().status
        except Exception as error:
            self._publish_status(MissionStatus.NAVIGATION_ERROR, str(error))
            return

        if status == GoalStatus.STATUS_CANCELED and (
            self.fallback_takeover_active
            or self.fallback_terminal_reported
        ):
            self.get_logger().info(
                "Nav2 cancellation belongs to LiDAR fallback; "
                "suppressing ordinary CANCELED status"
            )
            return
        if (
            status != GoalStatus.STATUS_SUCCEEDED
            and self.assignment is not None
            and self.assignment.role == RobotState.ROLE_RETURN
        ):
            self.pending_pose = deepcopy(self.assignment.target)
            self._retry_or_fail(
                serial, f"return Nav2 status={status}"
            )
            return
        if status == GoalStatus.STATUS_CANCELED:
            self._publish_status(MissionStatus.CANCELED)
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(
                MissionStatus.NAVIGATION_ERROR, f"Nav2 status={status}"
            )
            return
        self._publish_status(MissionStatus.ARRIVED)

    def _retry_or_fail(self, serial: int, reason: str) -> None:
        if serial != self.goal_serial:
            return
        if time.monotonic() >= self.retry_deadline:
            self.pending_pose = None
            self._cancel_retry_timer()
            self._publish_status(MissionStatus.NAVIGATION_ERROR, reason)
            return
        self.get_logger().warning(
            f"{reason}; retrying in {self.dispatch_retry_interval:.1f}s"
        )
        self._cancel_retry_timer()
        self.retry_timer = self.create_timer(
            self.dispatch_retry_interval,
            lambda: self._retry_pending_goal(serial),
        )

    def _retry_pending_goal(self, serial: int) -> None:
        self._cancel_retry_timer()
        if serial != self.goal_serial or self.pending_pose is None:
            return
        if self._goal_blocked_for_recovery():
            self._publish_recovery_wait("Nav2 retry held for sensor recovery")
            return
        self._send_goal(self.pending_pose, serial)

    def _on_lidar_state(self, message: String) -> None:
        self.lidar_state = message.data.strip().upper()
        if self.lidar_state in {"FAULT", "RECOVERING"}:
            self.recovery_ready = False
        elif self.lidar_state == "ALIVE" and self.fallback_state in {
            "UNKNOWN",
            "IDLE",
        }:
            # A fault while Nav2 was idle creates no fallback session. The
            # watchdog's stable ALIVE transition is sufficient in that case.
            self.recovery_ready = True
        self._maybe_resume_after_recovery()

    def _on_recovery_ready(self, message: Bool) -> None:
        self.recovery_ready = bool(message.data)
        self._maybe_resume_after_recovery()

    def _on_fallback_state(self, message: String) -> None:
        state = message.data.strip().upper()
        if not state or state == self.fallback_state:
            return
        self.fallback_state = state
        if fallback_takes_ownership(state):
            self.fallback_takeover_active = True
            self._cancel_retry_timer()
            if self.assignment is not None:
                self._publish_recovery_wait(f"LiDAR fallback {state.lower()}")
            return
        if state == "RESUMED":
            self.fallback_takeover_active = False
            self.fallback_resume_requested = True
            self._maybe_resume_after_recovery()
            return
        if state == "SUCCEEDED":
            if self.assignment is None or not self.fallback_takeover_active:
                return
            self.fallback_takeover_active = False
            self.fallback_resume_requested = False
            self.fallback_terminal_reported = True
            self.pending_pose = None
            self._cancel_retry_timer()
            self._publish_status(
                MissionStatus.ARRIVED,
                "destination reached by LiDAR fallback",
            )
            return
        if state == "FAILED":
            if self.assignment is None or not self.fallback_takeover_active:
                return
            self.fallback_takeover_active = False
            self.fallback_resume_requested = False
            self.fallback_terminal_reported = True
            self.pending_pose = None
            self._cancel_retry_timer()
            self._publish_status(
                MissionStatus.NAVIGATION_ERROR,
                "LiDAR fallback failed; replacement required",
            )

    def _goal_blocked_for_recovery(self) -> bool:
        return self.fallback_takeover_active or lidar_blocks_nav2(
            self.lidar_state, self.recovery_ready
        )

    def _publish_recovery_wait(self, reason: str) -> None:
        self._publish_status(MissionStatus.RECOVERY_WAIT, reason)

    def _maybe_resume_after_recovery(self) -> None:
        if self.assignment is None or self.fallback_terminal_reported:
            return
        if self._goal_blocked_for_recovery():
            return
        if self.goal_handle is not None:
            return
        if self.fallback_resume_requested and self.pending_pose is None:
            self.pending_pose = deepcopy(self.assignment.target)
        if self.pending_pose is None:
            return
        self.fallback_resume_requested = False
        self.fallback_takeover_active = False
        # Invalidate the result callback belonging to the canceled Nav2 goal.
        self.goal_serial += 1
        serial = self.goal_serial
        self.retry_deadline = time.monotonic() + self.dispatch_retry_timeout
        self.last_distance = None
        self.last_feedback_pose = None
        self.last_progress_at = time.monotonic()
        self.blocked_reported = False
        self._publish_status(
            MissionStatus.RECOVERY_RESUMED,
            "LiDAR recovery confirmed; resuming Nav2",
        )
        self._send_goal(self.pending_pose, serial)

    def _cancel_retry_timer(self) -> None:
        if self.retry_timer is None:
            return
        timer = self.retry_timer
        self.retry_timer = None
        timer.cancel()
        self.destroy_timer(timer)

    def _on_feedback(self, feedback, serial: int) -> None:
        if serial != self.goal_serial or self.blocked_reported:
            return
        distance = float(feedback.feedback.distance_remaining)
        now = time.monotonic()
        pose = feedback.feedback.current_pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        current_pose = (pose.position.x, pose.position.y, yaw)
        if (
            self.assignment is not None
            and self.assignment.role == RobotState.ROLE_RETURN
            and position_within_radius(
                (current_pose[0], current_pose[1]),
                (
                    self.assignment.target.pose.position.x,
                    self.assignment.target.pose.position.y,
                ),
                self.return_arrival_tolerance,
            )
        ):
            self.return_arrival_reported = True
            self.blocked_reported = True
            goal_handle = self.goal_handle
            self.goal_handle = None
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            self._publish_status(
                MissionStatus.ARRIVED,
                "return target reached within "
                f"{self.return_arrival_tolerance:.2f}m",
            )
            return
        if pose_has_progress(
            self.last_feedback_pose,
            current_pose,
            translation_epsilon=self.progress_translation_epsilon,
            rotation_epsilon=self.progress_rotation_epsilon,
        ):
            self.last_feedback_pose = current_pose
            self.last_progress_at = now
        if self.last_distance is None:
            self.last_distance = distance
            self.last_progress_at = now
            return
        # Nav2 can briefly report 0.0 before its first real path distance.
        if self.last_distance <= self.progress_epsilon < distance:
            self.last_distance = distance
            self.last_progress_at = now
            return
        if distance < self.last_distance - self.progress_epsilon:
            self.last_distance = distance
            self.last_progress_at = now

    def _check_progress(self) -> None:
        if self.goal_handle is None or self.blocked_reported:
            return
        if time.monotonic() - self.last_progress_at < self.blocked_timeout:
            return
        self.blocked_reported = True
        self.goal_handle.cancel_goal_async()
        self._publish_status(
            MissionStatus.BLOCKED,
            f"no translation, rotation, or path progress for "
            f"{self.blocked_timeout:.1f}s",
        )

    def _publish_status(self, state: int, reason: str = "") -> None:
        message = MissionStatus()
        if self.assignment is not None:
            message.mission_id = self.assignment.mission_id
            message.event_id = self.assignment.event_id
            message.assignment_version = self.assignment.assignment_version
        message.robot_id = self.robot_id
        message.status = state
        message.stamp = self.get_clock().now().to_msg()
        message.reason = reason
        self.status_publisher.publish(message)
        self.get_logger().info(
            f"mission={message.mission_id} status={state} reason={reason}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutor()
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
