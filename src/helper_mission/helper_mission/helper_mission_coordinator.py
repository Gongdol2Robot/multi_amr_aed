"""AED 도착 로봇에 현장 구조 인력 탐색 임무를 연결한다."""

from aed_interfaces.action import GuideHelper
from aed_interfaces.msg import EmergencyEvent, MissionStatus
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from helper_mission.mission_logic import (
    arrival_dispatch_allowed,
    dispatch_response_is_current,
)


class HelperMissionCoordinator(Node):
    """AED가 도착하면 바로 그 로봇에 회전 탐색 임무를 요청한다."""

    def __init__(self) -> None:
        """이벤트·도착 상태 구독과 로봇별 GuideHelper client를 초기화한다."""
        super().__init__("helper_mission_coordinator")
        self.declare_parameter("robot_ids", ["robot1", "robot2"])
        self.declare_parameter("event_topic", "/aed/emergency_event")
        self.declare_parameter(
            "additional_event_topics",
            [
                "/camera_open/vision/emergency_event",
                "/camera_alley/vision/emergency_event",
            ],
        )
        self.declare_parameter("mission_status_topic", "/aed/mission_status")
        self.declare_parameter("guide_action_suffix", "aed/guide_helper")
        self.declare_parameter("action_server_timeout", 2.0)

        self.robot_ids = list(self.get_parameter("robot_ids").value)
        if (
            not self.robot_ids
            or len(set(self.robot_ids)) != len(self.robot_ids)
        ):
            raise ValueError("robot_ids must be a non-empty unique list")
        self.action_server_timeout = float(
            self.get_parameter("action_server_timeout").value
        )
        if self.action_server_timeout <= 0.0:
            raise ValueError("action_server_timeout must be positive")

        suffix = str(
            self.get_parameter("guide_action_suffix").value
        ).strip("/")
        self.action_clients = {
            robot_id: ActionClient(self, GuideHelper, f"/{robot_id}/{suffix}")
            for robot_id in self.robot_ids
        }
        self.status_publisher = self.create_publisher(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            20,
        )
        primary_event_topic = str(
            self.get_parameter("event_topic").value
        ).strip()
        additional_event_topics = [
            str(topic).strip()
            for topic in self.get_parameter(
                "additional_event_topics"
            ).value
        ]
        event_topics = list(
            dict.fromkeys(
                topic
                for topic in [primary_event_topic, *additional_event_topics]
                if topic
            )
        )
        if not event_topics:
            raise ValueError("at least one emergency event topic is required")
        self.event_subscriptions = [
            self.create_subscription(
                EmergencyEvent,
                topic,
                self._on_event,
                20,
            )
            for topic in event_topics
        ]
        self.create_subscription(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            self._on_status,
            20,
        )

        self.events = {}
        self.pending = {}
        self.active_goals = {}
        self.canceled_events = set()
        self.handled_arrivals = set()
        self.dispatch_serial = 0
        self.get_logger().info(
            "ready: arrived AED robot will rotate and call for a helper; "
            f"event_topics={event_topics}"
        )

    def _on_event(self, event: EmergencyEvent) -> None:
        """확정 이벤트를 도착 뒤 현장 탐색까지 보존한다."""
        if not event.event_id:
            return
        if event.status == EmergencyEvent.CONFIRMED:
            if not event.location_valid:
                self.get_logger().warning(
                    f"Ignoring event {event.event_id} with unsafe location "
                    f"source={event.location_source}"
                )
                return
            if not event.location.header.frame_id:
                self.get_logger().warning(
                    f"Ignoring event {event.event_id} without location frame"
                )
                return
            self.canceled_events.discard(event.event_id)
            self.events[event.event_id] = event
            self._try_dispatch(event.event_id)
            return
        # Vision의 CANCELED는 카메라가 쓰러진 사람을 잠깐 놓쳤다는 뜻이다.
        # 중앙 mission_manager도 CONFIRMED 이후 이 신호로 출동을 취소하지
        # 않으므로, helper 쪽만 이벤트를 지우면 도착 후 호출·복귀가 영원히
        # 시작되지 않는다. 확정 당시 좌표는 helper 결과가 끝날 때까지 보존한다.

    def _on_status(self, status: MissionStatus) -> None:
        """AED 정상 도착을 기록하고 도착한 동일 로봇에 탐색 임무를 준비한다."""
        if status.status != MissionStatus.ARRIVED or not status.event_id:
            return
        if status.robot_id not in self.action_clients:
            self.get_logger().warning(
                f"Unknown arrived robot ignored: {status.robot_id}"
            )
            return
        context = self.pending.get(status.event_id)
        if context is not None:
            # 전송 실패 뒤 같은 ARRIVED가 오면 기존 문맥으로만 재시도한다.
            if not context.get("dispatching", False):
                self._try_dispatch(status.event_id)
            return
        if not arrival_dispatch_allowed(
            status.event_id,
            canceled_events=self.canceled_events,
            handled_events=self.handled_arrivals,
            pending_events=self.pending,
            active_events=self.active_goals,
        ):
            # 같은 ARRIVED가 재전송되어도 진행 중인 Goal 문맥을 덮어쓰지 않는다.
            return
        self.handled_arrivals.add(status.event_id)
        self.pending[status.event_id] = {
            "robot_id": status.robot_id,
            "version": max(1, int(status.assignment_version) + 1),
            "dispatching": False,
        }
        self._try_dispatch(status.event_id)

    def _try_dispatch(self, event_id: str) -> None:
        """이벤트 좌표와 도착 로봇이 모두 확인되면 회전 탐색 Goal을 보낸다."""
        event = self.events.get(event_id)
        context = self.pending.get(event_id)
        if event is None or context is None or context["dispatching"]:
            return
        robot_id = context["robot_id"]
        client = self.action_clients[robot_id]
        if not client.wait_for_server(timeout_sec=self.action_server_timeout):
            self._publish_wait(
                event_id,
                robot_id,
                f"GuideHelper action unavailable on {robot_id}",
            )
            return

        patient_pose = self._patient_pose(event)
        goal = GuideHelper.Goal()
        goal.mission_id = f"{event_id}-helper-scan"
        goal.event_id = event_id
        goal.robot_id = robot_id
        goal.mission_version = context["version"]
        # 기존 Action 형식과 호환하기 위해 두 pose에 같은 현장 좌표를 넣는다.
        goal.helper_search_pose = patient_pose
        goal.patient_pose = patient_pose
        self.dispatch_serial += 1
        serial = self.dispatch_serial
        context["serial"] = serial
        context["dispatching"] = True
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda response, eid=event_id, rid=robot_id, seq=serial: (
                self._goal_response(response, eid, rid, seq)
            )
        )
        self.get_logger().info(
            f"Event {event_id}: requesting on-site helper scan from {robot_id}"
        )

    def _goal_response(
        self, future, event_id: str, robot_id: str, serial: int
    ) -> None:
        """탐색 Goal 수락 여부를 처리하고 완료 결과 callback을 연결한다."""
        try:
            handle = future.result()
        except Exception as error:
            self._dispatch_failed(event_id, robot_id, serial, str(error))
            return
        if handle is None or not handle.accepted:
            self._dispatch_failed(
                event_id, robot_id, serial, "goal rejected"
            )
            return
        context = self.pending.get(event_id)
        is_current = dispatch_response_is_current(
            event_exists=event_id in self.events,
            canceled=event_id in self.canceled_events,
            context_serial=(
                None if context is None else context.get("serial")
            ),
            response_serial=serial,
            dispatching=(
                False if context is None else context.get("dispatching", False)
            ),
        )
        if not is_current:
            # 취소와 Goal 수락이 겹쳤다면 수락 직후 즉시 취소한다.
            handle.cancel_goal_async()
            self.get_logger().warning(
                f"Event {event_id}: canceled stale accepted helper goal"
            )
            return
        self.active_goals[event_id] = {
            "handle": handle,
            "serial": serial,
        }
        handle.get_result_async().add_done_callback(
            lambda result, eid=event_id, rid=robot_id, seq=serial: (
                self._goal_result(result, eid, rid, seq)
            )
        )

    def _goal_result(
        self, future, event_id: str, robot_id: str, serial: int
    ) -> None:
        """구조 인력 탐색 결과를 기록하고 이벤트별 실행 상태를 정리한다."""
        active = self.active_goals.get(event_id)
        if active is None or active.get("serial") != serial:
            return
        try:
            wrapped = future.result()
            code = wrapped.result.code
            reason = wrapped.result.reason
        except Exception as error:
            code = GuideHelper.Result.NAVIGATION_FAILED
            reason = str(error)
        self.active_goals.pop(event_id, None)
        self.pending.pop(event_id, None)
        self.events.pop(event_id, None)
        if code == GuideHelper.Result.SUCCEEDED:
            self.get_logger().info(
                f"Event {event_id}: helper detected by {robot_id}; "
                "guidance played"
            )
        elif code != GuideHelper.Result.CANCELED:
            self.get_logger().error(
                f"Event {event_id}: helper scan failed on {robot_id}: {reason}"
            )

    def _dispatch_failed(
        self, event_id: str, robot_id: str, serial: int, reason: str
    ) -> None:
        """Action 전송 실패를 대기 상태로 바꿔 이후 도착 상태 재수신을 허용한다."""
        context = self.pending.get(event_id)
        if (
            context is None
            or context.get("serial") != serial
            or event_id in self.canceled_events
        ):
            return
        context["dispatching"] = False
        self._publish_wait(event_id, robot_id, reason)

    @staticmethod
    def _patient_pose(event: EmergencyEvent) -> PoseStamped:
        """EmergencyEvent의 사고 위치를 현장 탐색 Goal용 PoseStamped로 바꾼다."""
        pose = PoseStamped()
        pose.header = event.location.header
        pose.pose.position.x = event.location.point.x
        pose.pose.position.y = event.location.point.y
        pose.pose.position.z = event.location.point.z
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_wait(
        self, event_id: str, robot_id: str, reason: str
    ) -> None:
        """탐색 Action을 시작할 수 없는 이유를 공통 상태 토픽에 발행한다."""
        message = MissionStatus()
        message.mission_id = f"{event_id}-helper-scan"
        message.event_id = event_id
        message.robot_id = robot_id
        message.status = MissionStatus.RECOVERY_WAIT
        message.stamp = self.get_clock().now().to_msg()
        message.reason = reason
        self.status_publisher.publish(message)
        self.get_logger().warning(f"Event {event_id}: {reason}")

    def destroy_node(self) -> None:
        """노드 종료 전에 수락된 모든 구조 인력 탐색 Goal을 취소한다."""
        for active in self.active_goals.values():
            active["handle"].cancel_goal_async()
        self.active_goals.clear()
        super().destroy_node()


def main(args=None) -> None:
    """중앙 coordinator 노드를 실행하고 종료 시 ROS 자원을 정리한다."""
    rclpy.init(args=args)
    node = HelperMissionCoordinator()
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
