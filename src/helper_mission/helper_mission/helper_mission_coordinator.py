"""Coordinate an automatic helper mission after AED arrival."""

from math import cos, sin

from aed_interfaces.action import GuideHelper
from aed_interfaces.msg import (
    EmergencyEvent,
    HelperPresence,
    MissionStatus,
    RobotState,
)
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from helper_mission.mission_logic import Candidate, select_helper_robot


class HelperMissionCoordinator(Node):
    """Dispatch a reserve AMR when no helper is near the delivered AED."""

    def __init__(self) -> None:
        """배정 파라미터와 ROS 통신 객체 및 이벤트별 임무 문맥을 초기화한다."""
        super().__init__("helper_mission_coordinator")
        self.declare_parameter("robot_ids", ["robot1", "robot2"])
        self.declare_parameter("event_topic", "/aed/emergency_event")
        self.declare_parameter("robot_state_topic", "/aed/robot_state")
        self.declare_parameter("mission_status_topic", "/aed/mission_status")
        self.declare_parameter("presence_topic", "/aed/helper_presence")
        self.declare_parameter("minimum_absence_evidence", 3)
        self.declare_parameter("helper_station_frame", "")
        self.declare_parameter("helper_station_x", 0.0)
        self.declare_parameter("helper_station_y", 0.0)
        self.declare_parameter("helper_station_yaw", 0.0)
        self.declare_parameter("guide_action_suffix", "aed/guide_helper")
        self.declare_parameter("action_server_timeout", 2.0)

        self.robot_ids = list(self.get_parameter("robot_ids").value)
        if (
            not self.robot_ids
            or len(set(self.robot_ids)) != len(self.robot_ids)
        ):
            raise ValueError("robot_ids must be a non-empty unique list")
        self.minimum_absence_evidence = int(
            self.get_parameter("minimum_absence_evidence").value
        )
        if self.minimum_absence_evidence < 1:
            raise ValueError("minimum_absence_evidence must be at least one")
        self.action_server_timeout = float(
            self.get_parameter("action_server_timeout").value
        )
        if self.action_server_timeout <= 0.0:
            raise ValueError("action_server_timeout must be positive")

        suffix = str(
            self.get_parameter("guide_action_suffix").value
        ).strip("/")
        self.action_clients = {
            robot_id: ActionClient(
                self, GuideHelper, f"/{robot_id}/{suffix}"
            )
            for robot_id in self.robot_ids
        }
        self.status_publisher = self.create_publisher(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            20,
        )
        self.create_subscription(
            EmergencyEvent,
            str(self.get_parameter("event_topic").value),
            self._on_event,
            20,
        )
        self.create_subscription(
            RobotState,
            str(self.get_parameter("robot_state_topic").value),
            self._on_robot_state,
            20,
        )
        self.create_subscription(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            self._on_status,
            20,
        )
        self.create_subscription(
            HelperPresence,
            str(self.get_parameter("presence_topic").value),
            self._on_presence,
            20,
        )

        self.events = {}
        self.robot_states = {}
        self.pending = {}
        self.active_goals = {}
        self.get_logger().info(
            "ready: waiting for AED arrival and event-scoped helper presence"
        )

    def _on_event(self, event: EmergencyEvent) -> None:
        """확정된 응급 이벤트의 환자 위치를 저장하고 보류 중인 배정을 재검토한다."""
        if event.status != EmergencyEvent.CONFIRMED or not event.event_id:
            return
        if not event.location.header.frame_id:
            self.get_logger().warning(
                f"Ignoring event {event.event_id} without location frame"
            )
            return
        self.events[event.event_id] = event
        self._try_dispatch(event.event_id)

    def _on_robot_state(self, state: RobotState) -> None:
        """대상 로봇의 최신 가용성과 건강 상태를 저장하고 배정을 다시 시도한다."""
        if state.robot_id not in self.action_clients:
            return
        self.robot_states[state.robot_id] = state
        for event_id in tuple(self.pending):
            self._try_dispatch(event_id)

    def _on_status(self, status: MissionStatus) -> None:
        """AED 도착 상태를 받으면 같은 이벤트의 조력자 부재 판정을 기다린다."""
        if status.status != MissionStatus.ARRIVED or not status.event_id:
            return
        if status.event_id in self.active_goals:
            return
        context = self.pending.setdefault(
            status.event_id,
            {
                "aed_robot": status.robot_id,
                "helper_version": status.assignment_version,
                "absence_confirmed": False,
                "dispatching": False,
                "waiting_reported": False,
            },
        )
        context["aed_robot"] = status.robot_id
        context["helper_version"] = max(
            context["helper_version"], status.assignment_version
        )
        self.get_logger().info(
            f"Event {status.event_id}: AED arrived; waiting for helper check"
        )

    def _on_presence(self, presence: HelperPresence) -> None:
        """Vision 근거로 현장 조력자 유무를 판정해 불필요한 출동을 막는다."""
        context = self.pending.get(presence.event_id)
        if context is None or context["dispatching"]:
            return
        if presence.evidence_count < self.minimum_absence_evidence:
            return
        if presence.helper_count > 0:
            self.get_logger().info(
                f"Event {presence.event_id}: helper already present; "
                "no guide mission needed"
            )
            self.pending.pop(presence.event_id, None)
            return
        context["absence_confirmed"] = True
        self._try_dispatch(presence.event_id)

    def _try_dispatch(self, event_id: str) -> None:
        """좌표·부재·후보 조건이 갖춰지면 선택 로봇에 새 A03 Goal을 전송한다."""
        context = self.pending.get(event_id)
        event = self.events.get(event_id)
        if (
            context is None
            or event is None
            or context["dispatching"]
            or not context["absence_confirmed"]
        ):
            return
        station_pose = self._helper_station_pose()
        if station_pose is None:
            if not context["waiting_reported"]:
                self._publish_wait(
                    event_id,
                    "helper_station_frame is not configured; dispatch blocked",
                )
                context["waiting_reported"] = True
            return

        candidates = [
            self._candidate(state) for state in self.robot_states.values()
        ]
        robot_id = select_helper_robot(candidates, context["aed_robot"])
        if robot_id is None:
            if not context["waiting_reported"]:
                self._publish_wait(
                    event_id, "no healthy reserve robot available"
                )
                context["waiting_reported"] = True
            return

        client = self.action_clients[robot_id]
        if not client.wait_for_server(timeout_sec=self.action_server_timeout):
            self._publish_wait(
                event_id, f"GuideHelper action unavailable on {robot_id}"
            )
            context["waiting_reported"] = True
            return

        context["dispatching"] = True
        context["waiting_reported"] = False
        goal = GuideHelper.Goal()
        goal.mission_id = f"{event_id}-helper"
        goal.event_id = event_id
        goal.robot_id = robot_id
        context["helper_version"] += 1
        goal.mission_version = max(1, int(context["helper_version"]))
        goal.helper_search_pose = station_pose
        goal.patient_pose = self._patient_pose(event)
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda response, eid=event_id, rid=robot_id: self._goal_response(
                response, eid, rid
            )
        )
        self.get_logger().info(
            f"Event {event_id}: requesting helper mission from {robot_id}"
        )

    def _goal_response(self, future, event_id: str, robot_id: str) -> None:
        """로봇의 A03 Goal 수락 결과를 처리하고 최종 Result callback을 연결한다."""
        context = self.pending.get(event_id)
        try:
            handle = future.result()
        except Exception as error:
            self._dispatch_failed(event_id, f"goal request failed: {error}")
            return
        if handle is None or not handle.accepted:
            self._dispatch_failed(event_id, f"goal rejected by {robot_id}")
            return
        self.active_goals[event_id] = handle
        handle.get_result_async().add_done_callback(
            lambda result, eid=event_id, rid=robot_id: self._goal_result(
                result, eid, rid
            )
        )
        if context is not None:
            context["dispatching"] = True

    def _goal_result(self, future, event_id: str, robot_id: str) -> None:
        """완료된 안내 임무의 성공·실패를 기록하고 이벤트별 실행 상태를 정리한다."""
        try:
            wrapped = future.result()
            code = wrapped.result.code
            reason = wrapped.result.reason
        except Exception as error:
            code = GuideHelper.Result.NAVIGATION_FAILED
            reason = str(error)
        self.active_goals.pop(event_id, None)
        self.pending.pop(event_id, None)
        if code == GuideHelper.Result.SUCCEEDED:
            self.get_logger().info(
                f"Event {event_id}: helper mission completed by {robot_id}"
            )
        else:
            self.get_logger().error(
                f"Event {event_id}: helper mission failed on {robot_id}: "
                f"{reason}"
            )

    def _dispatch_failed(self, event_id: str, reason: str) -> None:
        """Goal 전송 실패를 대기 상태로 전환하여 이후 상태 갱신 때 재시도한다."""
        context = self.pending.get(event_id)
        if context is not None:
            context["dispatching"] = False
        self._publish_wait(event_id, reason)

    def _helper_station_pose(self):
        """설정된 구조 인력 대기 좌표를 PoseStamped로 만들고 미설정 시 None을 준다."""
        frame = str(self.get_parameter("helper_station_frame").value)
        if not frame:
            return None
        yaw = float(self.get_parameter("helper_station_yaw").value)
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(
            self.get_parameter("helper_station_x").value
        )
        pose.pose.position.y = float(
            self.get_parameter("helper_station_y").value
        )
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        return pose

    def _patient_pose(self, event: EmergencyEvent) -> PoseStamped:
        """EmergencyEvent의 PointStamped 위치를 Nav2용 PoseStamped로 변환한다."""
        pose = PoseStamped()
        pose.header = event.location.header
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = event.location.point.x
        pose.pose.position.y = event.location.point.y
        pose.pose.position.z = event.location.point.z
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _candidate(state: RobotState) -> Candidate:
        """ROS RobotState에서 순수 후보 선정 로직에 필요한 필드만 추출한다."""
        return Candidate(
            robot_id=state.robot_id,
            available=state.availability == RobotState.AVAILABLE,
            network_ok=state.network_ok,
            localization_ok=state.localization_ok,
            nav2_ok=state.nav2_ok,
            emergency_stop=state.emergency_stop,
            path_valid=state.path_valid,
            battery_percentage=float(state.battery_percentage),
            path_cost=float(state.estimated_path_cost),
        )

    def _publish_wait(self, event_id: str, reason: str) -> None:
        """안전하게 출동할 수 없는 이유를 RECOVERY_WAIT 상태로 발행한다."""
        message = MissionStatus()
        message.mission_id = f"{event_id}-helper"
        message.event_id = event_id
        message.status = MissionStatus.RECOVERY_WAIT
        message.stamp = self.get_clock().now().to_msg()
        message.reason = reason
        self.status_publisher.publish(message)
        self.get_logger().warning(f"Event {event_id}: {reason}")


def main(args=None) -> None:
    """중앙 HelperMissionCoordinator 노드를 초기화하고 종료될 때까지 실행한다."""
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
