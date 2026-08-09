"""AED 배정을 받아 Undock, 경보 재생, Nav2 이동을 순서대로 수행한다."""

from math import isfinite

from action_msgs.msg import GoalStatus
from aed_interfaces.msg import MissionAssignment, MissionStatus, RobotState
from irobot_create_msgs.action import Undock
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from emergency_alert.alert_logic import TonePattern
from emergency_alert.audio_output import AudioOutput


class AlertMissionExecutor(Node):
    """
    TurtleBot4의 AED 출동과 주행 경보를 함께 관리하는 ROS 2 노드.

    로봇 전용 ``mission_assignment``를 구독하고, 자신에게 할당된 AED 임무만
    처리한다. Undock이 성공한 뒤 경보음을 시작하고 Nav2 Goal을 전송하며,
    도착·취소·실패 중 하나로 주행이 끝나면 반드시 경보음을 중지한다.

    실패 상태는 공통 ``/aed/mission_status``에 발행한다. Mission Manager는
    이 상태를 근거로 현재 로봇을 제외하고 다른 로봇에 임무를 재할당한다.
    """

    def __init__(self) -> None:
        """
        파라미터와 ROS 통신 객체, 임무 추적 상태를 초기화한다.

        상대 이름인 ``undock``, ``navigate_to_pose``, ``cmd_audio``는 노드를
        각 로봇 namespace에서 실행하면 해당 로봇의 토픽과 Action으로 자동
        해석된다. 따라서 로봇 수가 늘어나도 같은 노드를 ID만 바꿔 재사용한다.
        """
        super().__init__("alert_mission_executor")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("assignment_topic", "mission_assignment")
        self.declare_parameter("mission_status_topic", "/aed/mission_status")
        self.declare_parameter("undock_action", "undock")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("audio_topic", "cmd_audio")
        self.declare_parameter("audio_backend", "system")
        self.declare_parameter("audio_player", "auto")
        self.declare_parameter("audio_device", "")
        self.declare_parameter("action_server_timeout", 5.0)
        self.declare_parameter("alarm_period", 0.8)
        self.declare_parameter("note_duration", 0.25)
        self.declare_parameter("high_frequency", 1000)
        self.declare_parameter("low_frequency", 440)
        self.declare_parameter("terminal_note_duration", 0.2)
        self.declare_parameter(
            "arrival_frequencies", [523, 659, 784, 1047]
        )
        self.declare_parameter(
            "interrupted_frequencies", [880, 660, 440, 220]
        )

        self.robot_id = str(self.get_parameter("robot_id").value)
        if not self.robot_id:
            raise ValueError("robot_id parameter is required")

        self.server_timeout = float(
            self.get_parameter("action_server_timeout").value
        )
        self.alarm_period = float(self.get_parameter("alarm_period").value)
        self.note_duration = float(self.get_parameter("note_duration").value)
        travel_frequencies = (
            self.get_parameter("high_frequency").value,
            self.get_parameter("low_frequency").value,
        )
        self.terminal_note_duration = float(
            self.get_parameter("terminal_note_duration").value
        )
        self.travel_pattern = TonePattern.from_values(
            travel_frequencies, self.note_duration
        )
        self.arrival_pattern = TonePattern.from_values(
            self.get_parameter("arrival_frequencies").value,
            self.terminal_note_duration,
        )
        self.interrupted_pattern = TonePattern.from_values(
            self.get_parameter("interrupted_frequencies").value,
            self.terminal_note_duration,
        )
        if self.alarm_period < self.travel_pattern.total_duration:
            raise ValueError(
                "alarm_period must be at least the travel pattern duration"
            )
        if not isfinite(self.server_timeout) or self.server_timeout <= 0.0:
            raise ValueError("action_server_timeout must be positive")
        if not isfinite(self.alarm_period) or self.alarm_period <= 0.0:
            raise ValueError("alarm_period must be positive")
        if not isfinite(self.note_duration) or self.note_duration <= 0.0:
            raise ValueError("note_duration must be positive")
        if (
            not isfinite(self.terminal_note_duration)
            or self.terminal_note_duration <= 0.0
        ):
            raise ValueError("terminal_note_duration must be positive")
        self.undock_client = ActionClient(
            self,
            Undock,
            str(self.get_parameter("undock_action").value),
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
        )
        self.audio = AudioOutput(
            self,
            str(self.get_parameter("audio_topic").value),
            str(self.get_parameter("audio_backend").value),
            str(self.get_parameter("audio_player").value),
            str(self.get_parameter("audio_device").value),
        )
        self.status_publisher = self.create_publisher(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            20,
        )
        self.create_subscription(
            MissionAssignment,
            str(self.get_parameter("assignment_topic").value),
            self._on_assignment,
            10,
        )
        self.alarm_timer = self.create_timer(
            self.alarm_period, self._alarm_tick
        )

        self.assignment = None
        self.undock_goal_handle = None
        self.navigation_goal_handle = None
        self.mission_serial = 0
        self.alarm_active = False
        self.latest_versions = {}

        self.get_logger().info(
            f"ready: robot={self.robot_id}, "
            "mission flow=Undock -> alarm + NavigateToPose"
        )

    def _on_assignment(self, assignment: MissionAssignment) -> None:
        """
        새 AED 배정을 검증하고 Undock 단계부터 임무를 시작한다.

        다른 로봇이나 다른 역할의 배정은 무시한다. 같은 event의 과거 또는
        중복 assignment version도 무시하여 장애 복구 뒤 예전 Goal이 다시
        실행되는 것을 방지한다. 유효한 새 배정이면 기존 Goal과 경보를 먼저
        정리한 후 새로운 임무 일련번호로 비동기 흐름을 시작한다.
        """
        if assignment.robot_id != self.robot_id:
            return
        if assignment.role != RobotState.ROLE_AED_DELIVERY:
            return
        if not assignment.mission_id or not assignment.event_id:
            self.get_logger().warning(
                "Ignoring assignment without mission_id or event_id"
            )
            return
        if not assignment.target.header.frame_id:
            self.get_logger().warning(
                "Ignoring assignment without target frame"
            )
            return

        latest = self.latest_versions.get(assignment.event_id, -1)
        if assignment.assignment_version <= latest:
            self.get_logger().warning(
                f"Ignoring stale assignment: event={assignment.event_id}, "
                f"version={assignment.assignment_version}, latest={latest}"
            )
            return
        self.latest_versions[assignment.event_id] = (
            assignment.assignment_version
        )

        self.mission_serial += 1
        serial = self.mission_serial
        self._cancel_active_goals()
        self._stop_alarm()
        self.assignment = assignment
        self._publish_status(MissionStatus.ASSIGNED)
        self._start_undock(serial)

    def _start_undock(self, serial: int) -> None:
        """
        Undock Action 서버를 확인하고 비동기 Goal을 전송한다.

        서버를 찾지 못하면 출동할 수 없으므로 ``NAVIGATION_ERROR``를 보고한다.
        이 단계에서는 아직 로봇이 주행하지 않으므로 경보음도 시작하지 않는다.
        """
        if not self.undock_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):
            self._mission_error("Undock action unavailable", serial)
            return

        self._publish_status(MissionStatus.DISPATCHING, "undocking")
        future = self.undock_client.send_goal_async(
            Undock.Goal()
        )
        future.add_done_callback(
            lambda response: self._undock_response(response, serial)
        )

    def _undock_response(self, future, serial: int) -> None:
        """
        Undock Goal의 접수 결과를 처리하고 완료 결과를 기다린다.

        ``serial``이 현재 임무와 다르면 새 임무가 이미 시작된 것이므로, 늦게
        접수된 이전 Goal을 즉시 취소한다.
        """
        try:
            handle = future.result()
        except Exception as error:
            self._mission_error(f"Undock request failed: {error}", serial)
            return
        if serial != self.mission_serial:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._mission_error("Undock goal rejected", serial)
            return

        self.undock_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result: self._undock_done(result, serial)
        )

    def _undock_done(self, future, serial: int) -> None:
        """Undock 성공 후에만 경보 재생과 Nav2 이동을 시작한다."""
        if serial != self.mission_serial:
            return
        self.undock_goal_handle = None
        try:
            status = future.result().status
        except Exception as error:
            self._mission_error(f"Undock result failed: {error}", serial)
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._mission_error(f"Undock failed: status={status}", serial)
            return

        self.get_logger().info("Undock complete; starting travel alarm")
        self._start_alarm()
        self._start_navigation(serial)

    def _start_navigation(self, serial: int) -> None:
        """
        현재 배정의 목표 위치로 NavigateToPose Goal을 전송한다.

        Mission Manager가 경로 유효성을 확인한 뒤 배정했다는 전제에서 실행되며,
        Action 서버가 없으면 경보를 끄고 실패 상태를 보고한다.
        """
        if not self.navigate_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):
            self._mission_error("Nav2 action unavailable", serial)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.assignment.target
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response: self._navigation_response(response, serial)
        )

    def _navigation_response(self, future, serial: int) -> None:
        """Nav2 Goal 접수 여부를 확인하고 주행 완료 callback을 연결한다."""
        try:
            handle = future.result()
        except Exception as error:
            self._mission_error(f"Nav2 request failed: {error}", serial)
            return
        if serial != self.mission_serial:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._mission_error("Nav2 goal rejected", serial)
            return

        self.navigation_goal_handle = handle
        self._publish_status(MissionStatus.EN_ROUTE)
        handle.get_result_async().add_done_callback(
            lambda result: self._navigation_done(result, serial)
        )

    def _navigation_done(self, future, serial: int) -> None:
        """
        Nav2 최종 결과에 맞춰 경보를 끄고 임무 상태를 보고한다.

        성공은 ``ARRIVED``, 취소는 ``CANCELED``로 보고한다. Nav2가 장애물
        회피와 recovery를 수행한 뒤에도 도달하지 못해 ABORTED 등의 상태를
        반환하면 ``NAVIGATION_ERROR``로 보고하여 대체 로봇 재할당을 유도한다.
        """
        if serial != self.mission_serial:
            return
        self.navigation_goal_handle = None
        self._stop_alarm()
        try:
            status = future.result().status
        except Exception as error:
            self._mission_error(f"Nav2 result failed: {error}", serial)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._play_arrival_alert()
            self._publish_status(MissionStatus.ARRIVED)
            self.get_logger().info("AED arrived; travel alarm stopped")
        elif status == GoalStatus.STATUS_CANCELED:
            self._play_interrupted_alert()
            self._publish_status(MissionStatus.CANCELED, "Nav2 goal canceled")
        else:
            self._mission_error(f"Nav2 failed: status={status}", serial)

    def _mission_error(self, reason: str, serial: int) -> None:
        """현재 임무의 경보를 중지하고 재할당 가능한 실패를 보고한다."""
        if serial != self.mission_serial:
            return
        self._stop_alarm()
        self._play_interrupted_alert()
        self._publish_status(MissionStatus.NAVIGATION_ERROR, reason)
        self.get_logger().error(reason)

    def _start_alarm(self) -> None:
        """주행 경보 상태를 활성화하고 첫 음계를 즉시 발행한다."""
        self.alarm_active = True
        self._publish_alarm_sequence()

    def _alarm_tick(self) -> None:
        """타이머 주기마다 활성 상태인 경보 시퀀스를 다시 발행한다."""
        if self.alarm_active:
            self._publish_alarm_sequence()

    def _publish_alarm_sequence(self) -> None:
        """
        높은 음과 낮은 음으로 구성된 한 번의 경보 패턴을 발행한다.

        ``append=False``를 사용해 Create3의 기존 음계 큐를 교체한다. 따라서
        오래된 음계가 계속 누적되지 않고 최신 경보 패턴만 재생된다.
        """
        self.audio.play(self.travel_pattern)

    def _play_arrival_alert(self) -> None:
        """낮은 음에서 높은 음으로 올라가는 도착 완료음을 한 번 재생한다."""
        self.audio.play(self.arrival_pattern)
        self.get_logger().info("Arrival alert published")

    def _play_interrupted_alert(self) -> None:
        """높은 음에서 낮은 음으로 내려가는 출동 중단음을 한 번 재생한다."""
        self.audio.play(self.interrupted_pattern)
        self.get_logger().warning("Mission interrupted alert published")

    def _stop_alarm(self) -> None:
        """
        공통 오디오 출력에 정지 명령을 보내 현재 경보 큐를 지운다.

        도착뿐 아니라 Nav2 오류, Goal 취소, 새 임무 수신, 노드 종료에서도
        호출되어 로봇에 경보음이 남는 것을 방지한다.
        """
        was_active = self.alarm_active
        self.alarm_active = False
        self.audio.stop()
        if was_active:
            self.get_logger().info("Travel alarm stopped")

    def _cancel_active_goals(self) -> None:
        """진행 중인 Undock과 Nav2 Goal이 있으면 비동기로 취소한다."""
        if self.undock_goal_handle is not None:
            self.undock_goal_handle.cancel_goal_async()
            self.undock_goal_handle = None
        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
            self.navigation_goal_handle = None

    def _publish_status(self, state: int, reason: str = "") -> None:
        """현재 배정 식별자와 version을 포함한 MissionStatus를 발행한다."""
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

    def destroy_node(self) -> None:
        """노드 종료 전에 진행 중인 Action을 취소하고 경보를 끈다."""
        self.mission_serial += 1
        self._cancel_active_goals()
        self._stop_alarm()
        super().destroy_node()


def main(args=None) -> None:
    """ROS를 초기화하고 출동 경보 노드를 종료될 때까지 실행한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = AlertMissionExecutor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
