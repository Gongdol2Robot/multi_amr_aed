"""Alternative stop-and-reassign policy for a LiDAR fault.

[CODE REVIEW]
이 모듈은 LiDAR 장애 로봇이 직접 fallback 주행하는 방식과 비교하기 위해 만든
대안이다. 현재 운영 경로는 ``lidar_watchdog``과
``lidar_fallback_controller``(``fallback_path_follower.py``)를 사용하므로 이
노드는 실행하지 않는다. 정책 비교나 단독 시험을 위해 보존하며, 같은 로봇에서
fallback controller와 동시에 실행하면 안 된다.

On LiDAR FAULT: stop this robot's Nav2 goal and publish a replacement
request (with the pending destination) for a human or another system to
act on. This node does not drive the robot itself.

On LiDAR ALIVE: clear the replacement request and wait for Mission Manager
(or an operator) to decide what this robot does next. By default this node
does NOT resend the old goal itself — Mission Manager may have already
reassigned it to another robot while this one was down, and blindly
resuming would risk two robots converging on the same target. Set
`auto_resume_on_recovery:=true` to get the old standalone behavior back
(useful for testing without a Mission Manager in the loop).

"""

from typing import Optional

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from sensor_recovery.lidar_state_machine import LidarState

# Latched: a late subscriber (dashboard, future Mission Manager) must see
# the current replacement request immediately, not wait for the next edge.
_STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ReplacementRequestNode(Node):
    """Stop-and-signal LiDAR-fault response: no driving, just ask for help."""

    # [CODE REVIEW] 현재 운영에서는 생성되지 않는다. "현재 로봇이 계속 간다"가
    # 아니라 "즉시 멈추고 다른 로봇이 이어받는다"는 대안 정책 시험용이다.

    def __init__(self) -> None:
        super().__init__("replacement_request")

        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("auto_resume_on_recovery", False)
        navigate_action = str(self.get_parameter("navigate_action").value)

        self._lidar_state = LidarState.STARTING
        self._latest_path: Optional[Path] = None
        self._pending_goal: Optional[PoseStamped] = None

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.replacement_needed_pub = self.create_publisher(
            Bool, "replacement_needed", _STATUS_QOS
        )
        self.pending_goal_pub = self.create_publisher(
            PoseStamped, "pending_goal", _STATUS_QOS
        )

        self.create_subscription(String, "lidar_state", self._on_lidar_state, 10)
        self.create_subscription(Path, "plan", self._on_plan, 10)

        self._nav_client = ActionClient(self, NavigateToPose, navigate_action)
        self._cancel_client = self.create_client(
            CancelGoal, f"{navigate_action}/_action/cancel_goal"
        )

        self.replacement_needed_pub.publish(Bool(data=False))

    def _on_plan(self, msg: Path) -> None:
        self._latest_path = msg

    def _on_lidar_state(self, msg: String) -> None:
        try:
            new_state = LidarState(msg.data)
        except ValueError:
            return
        previous = self._lidar_state
        self._lidar_state = new_state
        if new_state == LidarState.FAULT and previous != LidarState.FAULT:
            self._stop_and_request_replacement()
        elif new_state == LidarState.ALIVE and previous in (
            LidarState.FAULT,
            LidarState.RECOVERING,
        ):
            self._resume_and_clear_replacement()

    def _stop_and_request_replacement(self) -> None:
        # [CODE REVIEW] 먼저 0속도와 Nav2 cancel을 요청한 뒤,
        # 마지막 /plan endpoint를 대체 로봇이 이어갈 pending goal로 보존한다.
        self.cmd_vel_pub.publish(Twist())
        self._cancel_nav2_goal()

        if self._latest_path is not None and self._latest_path.poses:
            self._pending_goal = self._latest_path.poses[-1]
        else:
            self._pending_goal = None
            self.get_logger().warning(
                "LiDAR FAULT but no /plan available; replacement request has no destination"
            )

        self.replacement_needed_pub.publish(Bool(data=True))
        if self._pending_goal is not None:
            self.pending_goal_pub.publish(self._pending_goal)
        self.get_logger().warning(
            "LiDAR FAULT: stopped and requested replacement"
            + (" (destination published)" if self._pending_goal is not None else "")
        )

    def _resume_and_clear_replacement(self) -> None:
        # [CODE REVIEW] 기본값에서는 LiDAR가 살아나도 기존 goal을 자동 재전송하지 않는다.
        # 중앙이 이미 다른 로봇을 보냈을 수 있어 이중 출동을 막기 위한 선택이다.
        self.replacement_needed_pub.publish(Bool(data=False))
        self.get_logger().info("LiDAR ALIVE: replacement request cleared")

        auto_resume = bool(self.get_parameter("auto_resume_on_recovery").value)
        if not auto_resume:
            self.get_logger().info(
                "auto_resume_on_recovery is false; waiting for Mission Manager "
                "or an operator to command this robot"
            )
            self._pending_goal = None
            return

        if self._pending_goal is not None:
            self._send_resume_goal(self._pending_goal)
        self._pending_goal = None

    def _cancel_nav2_goal(self) -> None:
        if not self._cancel_client.service_is_ready():
            self.get_logger().warning(
                "navigate_to_pose cancel service unavailable; Nav2 may still be driving"
            )
            return
        # Zero goal_id + zero stamp cancels every active goal, regardless of
        # who sent it (same convention used in fallback_path_follower.py).
        self._cancel_client.call_async(CancelGoal.Request())

    def _send_resume_goal(self, pose: PoseStamped) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_to_pose action server unavailable; cannot resume")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Resuming Nav2 toward the pre-fault goal")
        self._nav_client.send_goal_async(goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReplacementRequestNode()
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
