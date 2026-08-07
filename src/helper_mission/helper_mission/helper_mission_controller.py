"""Robot-side GuideHelper action server."""

import time

from action_msgs.msg import GoalStatus
from aed_interfaces.action import GuideHelper
from aed_interfaces.msg import HelperPresence, MissionStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String

from helper_mission.mission_logic import PresenceGate

try:
    from irobot_create_msgs.msg import AudioNote, AudioNoteVector
except ImportError:  # Development PCs may not have the TurtleBot4 messages.
    AudioNote = None
    AudioNoteVector = None


class HelperMissionController(Node):
    """Call a nearby helper and guide that person to the patient."""

    def __init__(self) -> None:
        """ROS 파라미터와 Action·토픽 통신 객체 및 임무 상태를 초기화한다."""
        super().__init__("helper_mission_controller")
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("robot_id", "")
        self.declare_parameter("guide_action", "aed/guide_helper")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("presence_topic", "/aed/helper_presence")
        self.declare_parameter("mission_status_topic", "/aed/mission_status")
        self.declare_parameter("audio_topic", "cmd_audio")
        self.declare_parameter("nav_server_timeout", 5.0)
        self.declare_parameter("helper_call_timeout", 30.0)
        self.declare_parameter("arrival_confirmation_timeout", 10.0)
        self.declare_parameter("minimum_evidence", 3)
        self.declare_parameter("call_distance_m", 3.0)
        self.declare_parameter("arrival_distance_m", 1.0)
        self.declare_parameter("arrival_hold_seconds", 2.0)
        self.declare_parameter("presence_stale_seconds", 1.0)
        self.declare_parameter("buzzer_period", 1.0)
        self.declare_parameter("buzzer_note_duration", 0.18)
        self.declare_parameter("buzzer_frequencies", [880, 660])

        self.robot_id = str(self.get_parameter("robot_id").value)
        if not self.robot_id:
            raise ValueError("robot_id parameter is required")
        self.nav_server_timeout = self._positive("nav_server_timeout")
        self.call_timeout = self._positive("helper_call_timeout")
        self.confirmation_timeout = self._positive(
            "arrival_confirmation_timeout"
        )
        self.minimum_evidence = int(
            self.get_parameter("minimum_evidence").value
        )
        if self.minimum_evidence < 1:
            raise ValueError("minimum_evidence must be at least one")
        self.call_distance = self._positive("call_distance_m")
        self.arrival_distance = self._positive("arrival_distance_m")
        self.arrival_hold = self._positive("arrival_hold_seconds")
        self.presence_stale = self._positive("presence_stale_seconds")
        self.buzzer_period = self._positive("buzzer_period")
        self.buzzer_duration = self._positive("buzzer_note_duration")
        self.buzzer_frequencies = tuple(
            int(value)
            for value in self.get_parameter("buzzer_frequencies").value
        )
        if not self.buzzer_frequencies or any(
            value <= 0 for value in self.buzzer_frequencies
        ):
            raise ValueError("buzzer_frequencies must be positive")

        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
            callback_group=self.callback_group,
        )
        audio_topic = str(self.get_parameter("audio_topic").value)
        self.uses_create_audio = AudioNoteVector is not None
        if self.uses_create_audio:
            self.audio_publisher = self.create_publisher(
                AudioNoteVector, audio_topic, 10
            )
        else:
            self.audio_publisher = self.create_publisher(
                String, f"{audio_topic}_fallback", 10
            )
            self.get_logger().warning(
                "irobot_create_msgs unavailable; publishing BEEP/STOP strings "
                f"on {audio_topic}_fallback"
            )
        self.status_publisher = self.create_publisher(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            20,
        )
        self.create_subscription(
            HelperPresence,
            str(self.get_parameter("presence_topic").value),
            self._on_presence,
            20,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            GuideHelper,
            str(self.get_parameter("guide_action").value),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self.callback_group,
        )

        self.busy = False
        self.current_request = None
        self.presence_gate = None
        self.latest_helper_pose = None
        self.navigation_goal_handle = None
        self.latest_versions = {}
        self.get_logger().info(
            f"ready: robot={self.robot_id}, action=GuideHelper, "
            "notifier=temporary buzzer"
        )

    def _positive(self, name: str) -> float:
        """양수여야 하는 실수형 파라미터를 읽고 잘못된 설정을 거부한다."""
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _on_goal(self, request) -> GoalResponse:
        """A03 Goal의 로봇·식별자·좌표·version을 검증해 수락 여부를 정한다."""
        if self.busy:
            self.get_logger().warning("Rejecting GuideHelper goal while busy")
            return GoalResponse.REJECT
        if request.robot_id != self.robot_id:
            return GoalResponse.REJECT
        if not request.mission_id or not request.event_id:
            return GoalResponse.REJECT
        if (
            not request.helper_search_pose.header.frame_id
            or not request.patient_pose.header.frame_id
        ):
            return GoalResponse.REJECT
        latest = self.latest_versions.get(request.event_id, -1)
        if request.mission_version <= latest:
            self.get_logger().warning(
                f"Rejecting stale helper goal: event={request.event_id}, "
                f"version={request.mission_version}, latest={latest}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _on_cancel(_goal_handle) -> CancelResponse:
        """운영자나 coordinator가 요청한 A03 취소를 항상 수락한다."""
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        """구조 인력 탐색점 이동부터 환자 현장 안내까지 A03 단계를 수행한다."""
        request = goal_handle.request
        self.busy = True
        self.current_request = request
        self.latest_versions[request.event_id] = request.mission_version
        self.latest_helper_pose = None
        result = GuideHelper.Result()
        try:
            self._publish_status(MissionStatus.HELPER_REQUESTED)
            nav_code = await self._navigate(
                goal_handle,
                request.helper_search_pose,
                GuideHelper.Goal.PHASE_MOVING_TO_HELPER,
                "moving to helper search point",
            )
            if nav_code is not None:
                return self._terminate(goal_handle, result, nav_code)

            self.presence_gate = PresenceGate(
                self.minimum_evidence,
                self.call_distance,
                0.0,
                self.presence_stale,
            )
            found = await self._call_helper(goal_handle)
            if goal_handle.is_cancel_requested:
                return self._terminate(
                    goal_handle, result, GuideHelper.Result.CANCELED
                )
            if not found:
                return self._terminate(
                    goal_handle, result, GuideHelper.Result.HELPER_NOT_FOUND
                )

            self._stop_buzzer()
            self.presence_gate = PresenceGate(
                self.minimum_evidence,
                self.arrival_distance,
                self.arrival_hold,
                self.presence_stale,
            )
            self._publish_status(
                MissionStatus.HELPER_EN_ROUTE,
                "helper confirmed; guiding to patient",
            )
            nav_code = await self._navigate(
                goal_handle,
                request.patient_pose,
                GuideHelper.Goal.PHASE_GUIDING_TO_PATIENT,
                "guiding helper to patient",
            )
            if nav_code is not None:
                return self._terminate(goal_handle, result, nav_code)

            arrived = await self._confirm_arrival(goal_handle)
            if goal_handle.is_cancel_requested:
                return self._terminate(
                    goal_handle, result, GuideHelper.Result.CANCELED
                )
            if not arrived:
                return self._terminate(
                    goal_handle, result, GuideHelper.Result.HELPER_LOST
                )

            goal_handle.succeed()
            result.code = GuideHelper.Result.SUCCEEDED
            result.reason = "helper arrived within configured distance"
            if self.latest_helper_pose is not None:
                result.helper_pose = self.latest_helper_pose
            result.finished_at = self.get_clock().now().to_msg()
            self._publish_status(MissionStatus.HELPER_ARRIVED, result.reason)
            self._publish_status(MissionStatus.COMPLETED, result.reason)
            return result
        except Exception as error:
            # Keep unexpected action failures visible to the coordinator.
            self.get_logger().error(
                f"GuideHelper execution failed: {error}"
            )
            return self._terminate(
                goal_handle,
                result,
                GuideHelper.Result.NAVIGATION_FAILED,
                str(error),
            )
        finally:
            self._stop_buzzer()
            self.presence_gate = None
            self.current_request = None
            self.navigation_goal_handle = None
            self.busy = False

    async def _navigate(self, goal_handle, pose, phase: int, detail: str):
        """Nav2 Goal을 보내고 A03 취소를 감시하면서 최종 주행 상태를 반환한다."""
        if not self.navigation_client.wait_for_server(
            timeout_sec=self.nav_server_timeout
        ):
            return GuideHelper.Result.NAVIGATION_FAILED

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        response_future = self.navigation_client.send_goal_async(goal)
        await response_future
        navigation_handle = response_future.result()
        if navigation_handle is None or not navigation_handle.accepted:
            return GuideHelper.Result.NAVIGATION_FAILED
        self.navigation_goal_handle = navigation_handle
        result_future = navigation_handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                navigation_handle.cancel_goal_async()
                return GuideHelper.Result.CANCELED
            self._publish_feedback(goal_handle, phase, detail)
            await self._delay(0.2)

        self.navigation_goal_handle = None
        wrapped_result = result_future.result()
        if wrapped_result.status == GoalStatus.STATUS_CANCELED:
            return GuideHelper.Result.CANCELED
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            return GuideHelper.Result.NAVIGATION_FAILED
        return None

    async def _call_helper(self, goal_handle) -> bool:
        """대기 장소에서 부저를 반복하며 제한 시간 동안 구조 인력을 확인한다."""
        started_at = time.monotonic()
        next_buzzer_at = 0.0
        while time.monotonic() - started_at < self.call_timeout:
            if goal_handle.is_cancel_requested:
                return False
            now = time.monotonic()
            if now >= next_buzzer_at:
                self._publish_buzzer()
                next_buzzer_at = now + self.buzzer_period
            self._publish_feedback(
                goal_handle,
                GuideHelper.Goal.PHASE_CALLING_HELPER,
                "calling helper with temporary buzzer",
            )
            if self.presence_gate.confirmed(now):
                return True
            await self._delay(0.1)
        return False

    async def _confirm_arrival(self, goal_handle) -> bool:
        """환자 위치에서 구조 인력이 기준 거리 안에 연속 유지되는지 확인한다."""
        started_at = time.monotonic()
        while time.monotonic() - started_at < self.confirmation_timeout:
            if goal_handle.is_cancel_requested:
                return False
            now = time.monotonic()
            self._publish_feedback(
                goal_handle,
                GuideHelper.Goal.PHASE_CONFIRMING_ARRIVAL,
                "confirming helper within patient distance",
            )
            if self.presence_gate.confirmed(now):
                return True
            await self._delay(0.1)
        return False

    def _on_presence(self, message: HelperPresence) -> None:
        """현재 이벤트와 로봇에 해당하는 Vision 검출만 판정 게이트에 반영한다."""
        if self.current_request is None or self.presence_gate is None:
            return
        if message.event_id != self.current_request.event_id:
            return
        if message.robot_id and message.robot_id != self.robot_id:
            return
        self.latest_helper_pose = message.helper_pose
        self.presence_gate.observe(
            helper_count=message.helper_count,
            evidence_count=message.evidence_count,
            distance=float(message.distance_m),
            observed_at=time.monotonic(),
        )

    def _publish_feedback(self, goal_handle, phase: int, detail: str) -> None:
        """현재 A03 단계와 최근 구조 인력 거리를 Action Feedback으로 보낸다."""
        feedback = GuideHelper.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        if self.presence_gate is not None:
            feedback.helper_distance_m = self.presence_gate.distance
        goal_handle.publish_feedback(feedback)

    def _publish_buzzer(self) -> None:
        """TurtleBot4 2음 부저 또는 개발 환경용 대체 문자열을 한 번 발행한다."""
        if not self.uses_create_audio:
            message = String()
            message.data = "BEEP " + ",".join(
                str(value) for value in self.buzzer_frequencies
            )
            self.audio_publisher.publish(message)
            return
        message = AudioNoteVector()
        message.append = False
        seconds = int(self.buzzer_duration)
        nanoseconds = int((self.buzzer_duration - seconds) * 1_000_000_000)
        for frequency in self.buzzer_frequencies:
            note = AudioNote()
            note.frequency = frequency
            note.max_runtime.sec = seconds
            note.max_runtime.nanosec = nanoseconds
            message.notes.append(note)
        self.audio_publisher.publish(message)

    def _stop_buzzer(self) -> None:
        """오디오 큐를 비우거나 STOP 문자열을 보내 남은 호출음을 정지한다."""
        if not self.uses_create_audio:
            message = String()
            message.data = "STOP"
            self.audio_publisher.publish(message)
            return
        message = AudioNoteVector()
        message.append = False
        self.audio_publisher.publish(message)

    def _terminate(self, goal_handle, result, code: int, reason: str = ""):
        """실패 코드를 A03 Result와 MissionStatus에 기록하고 Goal을 종료한다."""
        reasons = {
            GuideHelper.Result.CANCELED: "helper mission canceled",
            GuideHelper.Result.NAVIGATION_FAILED: "navigation failed",
            GuideHelper.Result.HELPER_NOT_FOUND: "helper was not confirmed",
            GuideHelper.Result.HELPER_LOST: "helper arrival was not confirmed",
        }
        result.code = code
        result.reason = reason or reasons.get(code, "helper mission failed")
        result.finished_at = self.get_clock().now().to_msg()
        if self.latest_helper_pose is not None:
            result.helper_pose = self.latest_helper_pose
        if code == GuideHelper.Result.CANCELED:
            goal_handle.canceled()
            self._publish_status(MissionStatus.CANCELED, result.reason)
        else:
            goal_handle.abort()
            self._publish_status(MissionStatus.NAVIGATION_ERROR, result.reason)
        return result

    def _publish_status(self, state: int, reason: str = "") -> None:
        """현재 임무 식별자와 version을 포함한 공통 MissionStatus를 발행한다."""
        message = MissionStatus()
        if self.current_request is not None:
            message.mission_id = self.current_request.mission_id
            message.event_id = self.current_request.event_id
            message.assignment_version = self.current_request.mission_version
        message.robot_id = self.robot_id
        message.status = state
        message.stamp = self.get_clock().now().to_msg()
        message.reason = reason
        self.status_publisher.publish(message)

    async def _delay(self, seconds: float) -> None:
        """executor를 막지 않는 ROS 타이머 기반 비동기 지연을 수행한다."""
        future = Future()
        timer_holder = {}

        def wake() -> None:
            """타이머를 제거하고 대기 중인 ROS Future를 완료한다."""
            timer = timer_holder.get("timer")
            if timer is not None:
                self.destroy_timer(timer)
            if not future.done():
                future.set_result(True)

        timer_holder["timer"] = self.create_timer(
            seconds, wake, callback_group=self.callback_group
        )
        await future

    def destroy_node(self) -> None:
        """노드 종료 전에 진행 중인 Nav2 Goal과 부저 출력을 안전하게 정리한다."""
        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
        self._stop_buzzer()
        self.action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    """다중 스레드 executor로 로봇 측 GuideHelper Action 서버를 실행한다."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = HelperMissionController()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
