"""Publish robot health and event-specific Nav2 path costs."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import time

from action_msgs.msg import GoalStatus
from aed_interfaces.msg import EmergencyEvent, RobotState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


@dataclass
class RobotRuntime:
    """Latest inputs and path-evaluation state for one robot."""

    pose: PoseStamped | None = None
    pose_received_at: float = 0.0
    battery_percentage: float = -1.0
    path_valid: bool = False
    path_cost: float = -1.0
    path_event_id: str = ""
    planning: bool = False
    last_plan_attempt: float = 0.0


def path_length(path: Path) -> float:
    """Return the accumulated planar length of a Nav2 path."""
    points = [
        (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
    ]
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("path contains non-finite coordinates")
    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )


class RobotStateMonitor(Node):
    """Evaluate configured Nav2 paths and publish event-tagged RobotState values."""

    def __init__(self) -> None:
        super().__init__("robot_state_monitor")
        self.declare_parameter("robot_ids", ["robot1", "robot2"])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("pose_timeout_sec", 3.0)
        self.declare_parameter("plan_retry_sec", 3.0)
        self.declare_parameter("state_publish_period_sec", 0.5)
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("event_topic", "/aed/emergency_event")
        self.declare_parameter("state_topic", "/aed/robot_state")

        self.robot_ids = [str(item) for item in self.get_parameter("robot_ids").value]
        if not self.robot_ids or len(set(self.robot_ids)) != len(self.robot_ids):
            raise ValueError("robot_ids must contain one or more unique robots")
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self.plan_retry = float(self.get_parameter("plan_retry_sec").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        period = float(self.get_parameter("state_publish_period_sec").value)
        if self.pose_timeout <= 0.0 or self.plan_retry <= 0.0 or period <= 0.0:
            raise ValueError("timeout, retry, and publish period must be positive")

        self.runtime = {robot_id: RobotRuntime() for robot_id in self.robot_ids}
        self.current_event_id = ""
        self.current_target: PoseStamped | None = None
        self.event_serial = 0

        self.state_publisher = self.create_publisher(
            RobotState, str(self.get_parameter("state_topic").value), 20
        )
        self.path_publishers = {
            robot_id: self.create_publisher(
                Path, f"/aed/candidate_path/{robot_id}", 10
            )
            for robot_id in self.robot_ids
        }
        self.planner_clients = {
            robot_id: ActionClient(
                self,
                ComputePathToPose,
                f"/{robot_id}/compute_path_to_pose",
            )
            for robot_id in self.robot_ids
        }
        self.pose_subscriptions = []
        self.battery_subscriptions = []
        for robot_id in self.robot_ids:
            self.pose_subscriptions.append(
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    f"/{robot_id}/amcl_pose",
                    lambda message, rid=robot_id: self._on_pose(rid, message),
                    qos_profile_sensor_data,
                )
            )
            self.battery_subscriptions.append(
                self.create_subscription(
                    BatteryState,
                    f"/{robot_id}/battery_state",
                    lambda message, rid=robot_id: self._on_battery(rid, message),
                    qos_profile_sensor_data,
                )
            )
        self.event_subscription = self.create_subscription(
            EmergencyEvent,
            str(self.get_parameter("event_topic").value),
            self._on_event,
            10,
        )
        self.timer = self.create_timer(period, self._on_timer)

    def _on_pose(
        self, robot_id: str, message: PoseWithCovarianceStamped
    ) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f"Ignore {robot_id} pose frame={message.header.frame_id}; "
                f"expected {self.map_frame}"
            )
            return
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.header.frame_id = self.map_frame
        pose.pose = deepcopy(message.pose.pose)
        runtime = self.runtime[robot_id]
        runtime.pose = pose
        runtime.pose_received_at = time.monotonic()

    def _on_battery(self, robot_id: str, message: BatteryState) -> None:
        percentage = float(message.percentage)
        if math.isfinite(percentage):
            self.runtime[robot_id].battery_percentage = percentage

    def _on_event(self, event: EmergencyEvent) -> None:
        if event.status != EmergencyEvent.CONFIRMED:
            if event.event_id == self.current_event_id:
                self.current_event_id = ""
                self.current_target = None
            return
        if not event.event_id or not event.location.header.frame_id:
            self.get_logger().error("Confirmed event requires ID and location frame")
            return
        if event.event_id == self.current_event_id:
            return

        target = PoseStamped()
        target.header = deepcopy(event.location.header)
        target.pose.position = deepcopy(event.location.point)
        target.pose.orientation.w = 1.0
        self.current_event_id = event.event_id
        self.current_target = target
        self.event_serial += 1
        for runtime in self.runtime.values():
            runtime.path_valid = False
            runtime.path_cost = -1.0
            runtime.path_event_id = ""
            runtime.planning = False
            runtime.last_plan_attempt = 0.0
        self.get_logger().info(f"Evaluate Nav2 paths for event {event.event_id}")
        self._request_missing_plans()

    def _on_timer(self) -> None:
        self._request_missing_plans()
        for robot_id in self.robot_ids:
            self._publish_state(robot_id)

    def _request_missing_plans(self) -> None:
        if not self.current_event_id or self.current_target is None:
            return
        now = time.monotonic()
        for robot_id, runtime in self.runtime.items():
            if runtime.planning or runtime.path_valid:
                continue
            if now - runtime.last_plan_attempt < self.plan_retry:
                continue
            runtime.last_plan_attempt = now
            self._request_plan(robot_id, self.event_serial)

    def _request_plan(self, robot_id: str, serial: int) -> None:
        runtime = self.runtime[robot_id]
        if runtime.pose is None or self._pose_age(runtime) > self.pose_timeout:
            runtime.path_event_id = self.current_event_id
            runtime.path_valid = False
            runtime.path_cost = -1.0
            return
        client = self.planner_clients[robot_id]
        if not client.server_is_ready():
            runtime.path_event_id = self.current_event_id
            runtime.path_valid = False
            runtime.path_cost = -1.0
            return

        goal = ComputePathToPose.Goal()
        goal.goal = deepcopy(self.current_target)
        goal.start = deepcopy(runtime.pose)
        stamp = self.get_clock().now().to_msg()
        goal.goal.header.stamp = stamp
        goal.start.header.stamp = stamp
        goal.planner_id = self.planner_id
        goal.use_start = True
        runtime.planning = True
        client.send_goal_async(goal).add_done_callback(
            lambda future, rid=robot_id, event_id=self.current_event_id:
            self._on_plan_response(rid, serial, event_id, future)
        )

    def _on_plan_response(
        self, robot_id: str, serial: int, event_id: str, future
    ) -> None:
        if serial != self.event_serial or event_id != self.current_event_id:
            return
        runtime = self.runtime[robot_id]
        try:
            handle = future.result()
        except Exception as error:
            self._plan_failed(robot_id, event_id, f"request error: {error}")
            return
        if not handle.accepted:
            self._plan_failed(robot_id, event_id, "planner rejected request")
            return
        handle.get_result_async().add_done_callback(
            lambda result, rid=robot_id: self._on_plan_result(
                rid, serial, event_id, result
            )
        )
        runtime.planning = True

    def _on_plan_result(
        self, robot_id: str, serial: int, event_id: str, future
    ) -> None:
        if serial != self.event_serial or event_id != self.current_event_id:
            return
        try:
            wrapped = future.result()
            path = wrapped.result.path
            if int(wrapped.status) != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"planner status={wrapped.status}")
            if not path.poses:
                raise RuntimeError("planner returned empty path")
            cost = path_length(path)
        except Exception as error:
            self._plan_failed(robot_id, event_id, str(error))
            return

        runtime = self.runtime[robot_id]
        runtime.planning = False
        runtime.path_event_id = event_id
        runtime.path_valid = True
        runtime.path_cost = cost
        self.path_publishers[robot_id].publish(path)
        self.get_logger().info(
            f"{event_id} {robot_id}: Nav2 path cost={cost:.2f}m"
        )
        self._publish_state(robot_id)

    def _plan_failed(self, robot_id: str, event_id: str, reason: str) -> None:
        runtime = self.runtime[robot_id]
        runtime.planning = False
        runtime.path_event_id = event_id
        runtime.path_valid = False
        runtime.path_cost = -1.0
        self.path_publishers[robot_id].publish(Path())
        self.get_logger().warning(f"{event_id} {robot_id}: {reason}")
        self._publish_state(robot_id)

    def _publish_state(self, robot_id: str) -> None:
        runtime = self.runtime[robot_id]
        pose_fresh = runtime.pose is not None and self._pose_age(runtime) <= self.pose_timeout
        nav2_ok = self.planner_clients[robot_id].server_is_ready()
        state = RobotState()
        state.robot_id = robot_id
        state.stamp = self.get_clock().now().to_msg()
        if runtime.pose is not None:
            state.pose = deepcopy(runtime.pose)
        state.battery_percentage = runtime.battery_percentage
        state.role = RobotState.ROLE_NONE
        state.network_ok = pose_fresh
        state.localization_ok = pose_fresh
        state.nav2_ok = nav2_ok
        state.emergency_stop = False
        state.path_valid = runtime.path_valid
        state.estimated_path_cost = runtime.path_cost
        state.path_event_id = runtime.path_event_id
        state.last_heartbeat = state.stamp
        if not pose_fresh:
            state.availability = RobotState.LOCALIZATION_ERROR
            state.detail = "AMCL pose missing or stale"
        elif not nav2_ok:
            state.availability = RobotState.NAVIGATION_ERROR
            state.detail = "ComputePathToPose server unavailable"
        else:
            state.availability = RobotState.AVAILABLE
            state.detail = "ready"
        self.state_publisher.publish(state)

    @staticmethod
    def _pose_age(runtime: RobotRuntime) -> float:
        return time.monotonic() - runtime.pose_received_at


def main(args=None) -> None:
    """Run the robot state and path-cost monitor."""
    rclpy.init(args=args)
    node = RobotStateMonitor()
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
