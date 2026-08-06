"""Select one of two robots and dispatch only that robot through Nav2."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
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

from .assignment import path_length


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
        self.declare_parameter("planning_timeout_sec", 30.0)
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
        self.planning_timeout = float(
            self.get_parameter("planning_timeout_sec").value
        )
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.dispatch_enabled = bool(
            self.get_parameter("dispatch_enabled").value
        )
        if self.pose_timeout <= 0.0:
            raise ValueError("pose_timeout_sec must be positive")
        if self.planning_timeout <= 0.0:
            raise ValueError("planning_timeout_sec must be positive")

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

        self.observations: dict[str, RobotObservation] = {}
        self.navigation_clients = {
            robot_id: ActionClient(
                self,
                NavigateToPose,
                f"/{robot_id}/navigate_to_pose",
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
        self.goal_serial = 0
        self.goal_handle = None
        self.navigation_active = False
        self.planning_active = False
        self.pending_plans: set[str] = set()
        self.plan_results: dict[str, tuple[Path, float]] = {}
        self.plan_failures: dict[str, str] = {}
        self.planning_target: PoseStamped | None = None
        self.planning_timer = None
        self.selected_robot = ""
        self.active_request_id = ""
        self.state = "IDLE"
        self._last_feedback_log = 0.0
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
            f"Dispatch enabled: {self.dispatch_enabled}; robots={self.robot_ids}"
        )
        self.get_logger().info(
            "RViz Publish Point topics: " + ", ".join(click_topics)
        )

    def _on_pose(
        self, robot_id: str, message: PoseWithCovarianceStamped
    ) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
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
            self._publish_status("FAILED", "request pose contains non-finite data")
            return
        if self.planning_active or self.navigation_active:
            self.get_logger().warning(
                "Ignoring a new emergency request while a mission is active"
            )
            return

        self.request_serial += 1
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
            goal.use_start = not self.use_planner_start
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
            lambda serial=self.request_serial: self._on_planning_timeout(serial),
        )

    def _on_plan_response(self, robot_id: str, serial: int, future) -> None:
        if serial != self.request_serial or robot_id not in self.pending_plans:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._record_plan_failure(robot_id, serial, f"request error: {error}")
            return
        if not handle.accepted:
            self._record_plan_failure(robot_id, serial, "planner rejected goal")
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
            self._record_plan_failure(robot_id, serial, f"result error: {error}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._record_plan_failure(
                robot_id, serial, f"planner result status={status}"
            )
            return
        if not path.poses:
            self._record_plan_failure(robot_id, serial, "planner returned empty path")
            return

        try:
            distance = path_length(
                (pose.pose.position.x, pose.pose.position.y)
                for pose in path.poses
            )
        except ValueError as error:
            self._record_plan_failure(robot_id, serial, str(error))
            return

        self.plan_results[robot_id] = (path, distance)
        self.path_publishers[robot_id].publish(path)
        self._publish_distance(robot_id, distance)
        self.pending_plans.discard(robot_id)
        self.get_logger().info(
            f"Candidate {robot_id}: Nav2 path distance={distance:.2f}m"
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
                (robot_id, result[1])
                for robot_id, result in self.plan_results.items()
            ),
            key=lambda item: (item[1], item[0]),
        )
        self.selected_robot = ranked[0][0]
        self._publish_selected_robot(self.selected_robot)
        detail = ", ".join(
            f"{robot_id}={distance:.2f}m" for robot_id, distance in ranked
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
        if self.planning_target is None:
            self._publish_status("FAILED", "planning target was lost")
            return
        self._dispatch_goal(self.selected_robot, self.planning_target)

    def _dispatch_goal(self, robot_id: str, target: PoseStamped) -> None:
        client = self.navigation_clients[robot_id]
        if not client.wait_for_server(timeout_sec=2.0):
            self._publish_status(
                "FAILED", f"/{robot_id}/navigate_to_pose is unavailable"
            )
            return
        self.navigation_active = True
        self.goal_serial += 1
        serial = self.goal_serial
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(target)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        future = client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_feedback(
                robot_id, serial, feedback
            ),
        )
        future.add_done_callback(
            lambda response: self._on_goal_response(robot_id, serial, response)
        )
        self._publish_status(
            "ASSIGNED", f"goal sent only to {robot_id}"
        )

    def _on_goal_response(self, robot_id: str, serial: int, future) -> None:
        if serial != self.goal_serial:
            return
        try:
            handle = future.result()
        except Exception as error:
            self.navigation_active = False
            self._publish_status("FAILED", f"goal send error: {error}")
            return
        if not handle.accepted:
            self.navigation_active = False
            self._publish_status("FAILED", f"{robot_id} rejected the goal")
            return
        self.goal_handle = handle
        self._publish_status("NAVIGATING", f"{robot_id} is moving")
        handle.get_result_async().add_done_callback(
            lambda result: self._on_navigation_result(robot_id, serial, result)
        )

    def _on_feedback(self, robot_id: str, serial: int, feedback) -> None:
        if serial != self.goal_serial:
            return
        now = time.monotonic()
        if now - self._last_feedback_log < 2.0:
            return
        self._last_feedback_log = now
        remaining = float(feedback.feedback.distance_remaining)
        self.get_logger().info(
            f"{robot_id} navigating: remaining={remaining:.2f}m"
        )

    def _on_navigation_result(self, robot_id: str, serial: int, future) -> None:
        if serial != self.goal_serial:
            return
        self.goal_handle = None
        self.navigation_active = False
        try:
            status = int(future.result().status)
        except Exception as error:
            self._publish_status("FAILED", f"result error: {error}")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status("ARRIVED", f"{robot_id} reached the emergency")
            self._publish_status("COMPLETED", self.active_request_id)
            return
        if status == GoalStatus.STATUS_CANCELED:
            self._publish_status("FAILED", f"{robot_id} goal was canceled")
            return
        self._publish_status(
            "FAILED", f"{robot_id} Nav2 result status={status}"
        )

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
