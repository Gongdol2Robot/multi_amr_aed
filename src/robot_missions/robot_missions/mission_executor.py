"""Execute assigned Multi-AMR missions through Nav2."""

from action_msgs.msg import GoalStatus
from aed_interfaces.msg import MissionAssignment, MissionStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MissionExecutor(Node):
    """Translate mission assignments into cancelable Nav2 goals."""

    def __init__(self) -> None:
        super().__init__("mission_executor")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("assignment_topic", "mission_assignment")
        self.declare_parameter("navigate_action", "navigate_to_pose")

        self.robot_id = str(self.get_parameter("robot_id").value)
        if not self.robot_id:
            raise ValueError("robot_id parameter is required")

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
        )
        self.status_publisher = self.create_publisher(
            MissionStatus, "/aed/mission_status", 10
        )
        self.create_subscription(
            MissionAssignment,
            str(self.get_parameter("assignment_topic").value),
            self._on_assignment,
            10,
        )

        self.assignment = None
        self.goal_handle = None
        self.goal_serial = 0

    def _on_assignment(self, assignment: MissionAssignment) -> None:
        if assignment.robot_id != self.robot_id:
            return
        if not assignment.mission_id:
            self.get_logger().warning("Ignoring assignment without mission_id")
            return

        self.goal_serial += 1
        self.assignment = assignment
        if assignment.cancel_previous and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self._send_goal(assignment.target, self.goal_serial)

    def _send_goal(self, pose, serial: int) -> None:
        if not pose.header.frame_id:
            self._publish_status(
                MissionStatus.NAVIGATION_ERROR, "target frame_id is empty"
            )
            return
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self._publish_status(
                MissionStatus.NAVIGATION_ERROR, "Nav2 action unavailable"
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self._publish_status(MissionStatus.DISPATCHING)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response: self._goal_response(response, serial)
        )

    def _goal_response(self, future, serial: int) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._publish_status(MissionStatus.NAVIGATION_ERROR, str(error))
            return
        if serial != self.goal_serial:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._publish_status(
                MissionStatus.NAVIGATION_ERROR, "goal rejected"
            )
            return

        self.goal_handle = handle
        self._publish_status(MissionStatus.EN_ROUTE)
        handle.get_result_async().add_done_callback(
            lambda result: self._navigation_done(result, serial)
        )

    def _navigation_done(self, future, serial: int) -> None:
        if serial != self.goal_serial:
            return
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as error:
            self._publish_status(MissionStatus.NAVIGATION_ERROR, str(error))
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
