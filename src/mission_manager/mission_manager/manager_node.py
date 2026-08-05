"""Assign AED delivery and guidance roles to available robots."""

from copy import deepcopy

from aed_interfaces.msg import EmergencyEvent, MissionAssignment, RobotState
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

from mission_manager.role_assignment import select_roles


class MissionManager(Node):
    """Create two parallel emergency missions from one confirmed event."""

    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("robot_ids", ["robot1", "robot2"])
        self.declare_parameter("event_topic", "/aed/emergency_event")
        self.declare_parameter("robot_state_topic", "/aed/robot_state")
        self.declare_parameter("guide_wait_x", 0.0)
        self.declare_parameter("guide_wait_y", 0.0)
        self.declare_parameter("guide_wait_yaw_w", 1.0)

        self.robot_ids = list(self.get_parameter("robot_ids").value)
        if len(self.robot_ids) < 2:
            raise ValueError("robot_ids must contain at least two robots")

        self.robot_states = {}
        self.handled_events = set()
        self.assignment_publishers = {
            robot_id: self.create_publisher(
                MissionAssignment,
                f"/{robot_id}/mission_assignment",
                10,
            )
            for robot_id in self.robot_ids
        }
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

    def _on_robot_state(self, state: RobotState) -> None:
        if state.robot_id not in self.assignment_publishers:
            return
        self.robot_states[state.robot_id] = state

    def _on_emergency(self, event: EmergencyEvent) -> None:
        if event.status != EmergencyEvent.CONFIRMED:
            return
        if not event.event_id or event.event_id in self.handled_events:
            return
        if not event.location.header.frame_id:
            self.get_logger().error("Emergency location frame_id is empty")
            return

        candidates = {
            robot_id: (state.pose.pose.position.x, state.pose.pose.position.y)
            for robot_id, state in self.robot_states.items()
            if state.availability == RobotState.AVAILABLE
            and bool(state.pose.header.frame_id)
        }
        if len(candidates) < 2:
            self.get_logger().error(
                f"Event {event.event_id}: fewer than two available robots"
            )
            return

        aed_robot, guide_robot = select_roles(
            candidates,
            (event.location.point.x, event.location.point.y),
        )
        emergency_pose = self._event_pose(event)
        now = self.get_clock().now().to_msg()

        aed_assignment = MissionAssignment()
        aed_assignment.mission_id = f"{event.event_id}-aed"
        aed_assignment.event_id = event.event_id
        aed_assignment.robot_id = aed_robot
        aed_assignment.role = RobotState.ROLE_AED_DELIVERY
        aed_assignment.target = emergency_pose
        aed_assignment.assigned_at = now
        aed_assignment.resume_previous_mission = True

        guide_assignment = MissionAssignment()
        guide_assignment.mission_id = f"{event.event_id}-guide"
        guide_assignment.event_id = event.event_id
        guide_assignment.robot_id = guide_robot
        guide_assignment.role = RobotState.ROLE_GUIDE
        guide_assignment.target = self._guide_wait_pose(
            event.location.header.frame_id
        )
        guide_assignment.secondary_target = deepcopy(emergency_pose)
        guide_assignment.assigned_at = now
        guide_assignment.resume_previous_mission = True

        self.assignment_publishers[aed_robot].publish(aed_assignment)
        self.assignment_publishers[guide_robot].publish(guide_assignment)
        self.handled_events.add(event.event_id)
        self.get_logger().info(
            f"Event {event.event_id}: AED={aed_robot}, GUIDE={guide_robot}"
        )

    @staticmethod
    def _event_pose(event: EmergencyEvent) -> PoseStamped:
        pose = PoseStamped()
        pose.header = event.location.header
        pose.pose.position.x = event.location.point.x
        pose.pose.position.y = event.location.point.y
        pose.pose.position.z = event.location.point.z
        pose.pose.orientation.w = 1.0
        return pose

    def _guide_wait_pose(self, frame_id: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.get_parameter("guide_wait_x").value)
        pose.pose.position.y = float(self.get_parameter("guide_wait_y").value)
        pose.pose.orientation.w = float(
            self.get_parameter("guide_wait_yaw_w").value
        )
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

