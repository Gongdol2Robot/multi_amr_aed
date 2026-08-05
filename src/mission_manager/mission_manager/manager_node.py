"""Assign one AED robot and automatically fail over on mission failure."""

from aed_interfaces.msg import (
    EmergencyEvent,
    MissionAssignment,
    MissionStatus,
    RobotState,
)
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

from mission_manager.role_assignment import rank_candidates


FAILURE_STATES = {
    MissionStatus.BLOCKED,
    MissionStatus.NETWORK_LOST,
    MissionStatus.NAVIGATION_ERROR,
}


class MissionManager(Node):
    """Maintain exactly one active AED delivery robot per emergency event."""

    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("robot_ids", ["robot1", "robot2"])
        self.declare_parameter("event_topic", "/aed/emergency_event")
        self.declare_parameter("robot_state_topic", "/aed/robot_state")
        self.declare_parameter("mission_status_topic", "/aed/mission_status")

        self.robot_ids = list(self.get_parameter("robot_ids").value)
        if len(self.robot_ids) != 2:
            raise ValueError("robot_ids must contain exactly two robots")

        self.robot_states = {}
        self.events = {}
        self.assignment_publishers = {
            robot_id: self.create_publisher(
                MissionAssignment, f"/{robot_id}/mission_assignment", 10
            )
            for robot_id in self.robot_ids
        }
        self.status_publisher = self.create_publisher(
            MissionStatus, "/aed/mission_status", 20
        )
        self.create_subscription(
            RobotState,
            str(self.get_parameter("robot_state_topic").value),
            self._on_robot_state,
            20,
        )
        self.create_subscription(
            EmergencyEvent,
            str(self.get_parameter("event_topic").value),
            self._on_emergency,
            10,
        )
        self.create_subscription(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            self._on_mission_status,
            20,
        )

    def _on_robot_state(self, state: RobotState) -> None:
        if state.robot_id in self.assignment_publishers:
            self.robot_states[state.robot_id] = state

    def _on_emergency(self, event: EmergencyEvent) -> None:
        if event.status != EmergencyEvent.CONFIRMED:
            return
        if not event.event_id or event.event_id in self.events:
            return
        if not event.location.header.frame_id:
            self.get_logger().error("Emergency location frame_id is empty")
            return

        self.events[event.event_id] = {
            "event": event,
            "active_robot": None,
            "excluded": set(),
            "version": 0,
            "terminal": False,
        }
        self._assign_next(event.event_id)

    def _on_mission_status(self, status: MissionStatus) -> None:
        context = self.events.get(status.event_id)
        if context is None or context["terminal"]:
            return
        if status.robot_id != context["active_robot"]:
            return
        if status.assignment_version != context["version"]:
            return

        if status.status in (MissionStatus.ARRIVED, MissionStatus.COMPLETED):
            context["terminal"] = True
            self.get_logger().info(
                f"Event {status.event_id}: AED arrived by {status.robot_id}"
            )
            return
        if status.status in FAILURE_STATES:
            failed_robot = status.robot_id
            context["excluded"].add(failed_robot)
            context["active_robot"] = None
            self.get_logger().warning(
                f"Event {status.event_id}: exclude {failed_robot}: {status.reason}"
            )
            self._assign_next(status.event_id)

    def _assign_next(self, event_id: str) -> None:
        context = self.events[event_id]
        event = context["event"]
        candidates = {}
        for robot_id, state in self.robot_states.items():
            if robot_id in context["excluded"]:
                continue
            if not self._is_available(state):
                continue
            candidates[robot_id] = {
                "position": (
                    state.pose.pose.position.x,
                    state.pose.pose.position.y,
                ),
                "path_valid": state.path_valid,
                "path_cost": float(state.estimated_path_cost),
            }

        ranked = rank_candidates(
            candidates, (event.location.point.x, event.location.point.y)
        )
        if not ranked:
            context["terminal"] = True
            self._publish_final_failure(event_id, "no available robot")
            return

        robot_id = ranked[0]
        context["version"] += 1
        context["active_robot"] = robot_id
        assignment = MissionAssignment()
        assignment.mission_id = f"{event_id}-aed"
        assignment.event_id = event_id
        assignment.robot_id = robot_id
        assignment.role = RobotState.ROLE_AED_DELIVERY
        assignment.target = self._event_pose(event)
        assignment.assigned_at = self.get_clock().now().to_msg()
        assignment.assignment_version = context["version"]
        assignment.cancel_previous = True
        self.assignment_publishers[robot_id].publish(assignment)
        self.get_logger().info(
            f"Event {event_id}: assign v{context['version']} to {robot_id}"
        )

    @staticmethod
    def _is_available(state: RobotState) -> bool:
        return (
            state.availability == RobotState.AVAILABLE
            and state.network_ok
            and state.localization_ok
            and state.nav2_ok
            and not state.emergency_stop
            and bool(state.pose.header.frame_id)
        )

    def _publish_final_failure(self, event_id: str, reason: str) -> None:
        status = MissionStatus()
        status.mission_id = f"{event_id}-aed"
        status.event_id = event_id
        status.status = MissionStatus.MISSION_FAILED
        status.stamp = self.get_clock().now().to_msg()
        status.reason = reason
        self.status_publisher.publish(status)
        self.get_logger().error(f"Event {event_id}: MISSION_FAILED: {reason}")

    @staticmethod
    def _event_pose(event: EmergencyEvent) -> PoseStamped:
        pose = PoseStamped()
        pose.header = event.location.header
        pose.pose.position.x = event.location.point.x
        pose.pose.position.y = event.location.point.y
        pose.pose.position.z = event.location.point.z
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
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
