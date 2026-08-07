"""AED 도착 로봇에서 회전 탐색·호출음·안내음을 실행한다."""

import time

from aed_interfaces.action import GuideHelper
from aed_interfaces.msg import MissionStatus
from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool, String

from helper_mission.mission_logic import helper_confirmation_is_fresh

try:
    from irobot_create_msgs.msg import AudioNote, AudioNoteVector
except ImportError:  # 개발 PC에는 TurtleBot4 메시지가 없을 수 있다.
    AudioNote = None
    AudioNoteVector = None


class HelperMissionController(Node):
    """현장에서 회전하며 구조 인력을 호출하고 Vision 감지 시 안내음을 낸다."""

    def __init__(self) -> None:
        """로봇별 Action·Vision·속도·오디오 통신과 탐색 파라미터를 구성한다."""
        super().__init__("helper_mission_controller")
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("robot_id", "")
        self.declare_parameter("guide_action", "aed/guide_helper")
        self.declare_parameter(
            "helper_confirmed_topic", "vision/helper_confirmed"
        )
        self.declare_parameter("mission_status_topic", "/aed/mission_status")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("audio_topic", "cmd_audio")
        self.declare_parameter("rotation_speed_rps", 0.35)
        self.declare_parameter("control_period", 0.1)
        self.declare_parameter("vision_stale_seconds", 1.0)
        # 0은 구조 인력이 올 때까지 시간 제한 없이 계속 탐색한다.
        self.declare_parameter("helper_wait_timeout", 0.0)
        self.declare_parameter("buzzer_period", 1.0)
        self.declare_parameter("buzzer_note_duration", 0.18)
        self.declare_parameter("buzzer_frequencies", [880, 660])
        self.declare_parameter("guide_note_duration", 0.3)
        self.declare_parameter("guide_frequencies", [523, 659, 784])

        self.robot_id = str(self.get_parameter("robot_id").value)
        if not self.robot_id:
            raise ValueError("robot_id parameter is required")
        self.rotation_speed = self._positive("rotation_speed_rps")
        self.control_period = self._positive("control_period")
        self.vision_stale = self._positive("vision_stale_seconds")
        self.wait_timeout = float(
            self.get_parameter("helper_wait_timeout").value
        )
        if self.wait_timeout < 0.0:
            raise ValueError("helper_wait_timeout must be zero or positive")
        self.buzzer_period = self._positive("buzzer_period")
        self.buzzer_duration = self._positive("buzzer_note_duration")
        self.guide_duration = self._positive("guide_note_duration")
        self.buzzer_frequencies = self._frequencies("buzzer_frequencies")
        self.guide_frequencies = self._frequencies("guide_frequencies")

        self.cmd_vel_publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
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
                "irobot_create_msgs unavailable; publishing temporary audio "
                f"commands on {audio_topic}_fallback"
            )
        self.status_publisher = self.create_publisher(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("helper_confirmed_topic").value),
            self._on_helper_confirmed,
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
        self.latest_versions = {}
        self.helper_confirmed = False
        self.helper_observed_at = None
        self.get_logger().info(
            f"ready: robot={self.robot_id}, behavior=rotate+beep until vision"
        )

    def _positive(self, name: str) -> float:
        """양수형 파라미터를 읽고 잘못된 값을 즉시 거부한다."""
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _frequencies(self, name: str) -> tuple[int, ...]:
        """음계 파라미터를 양의 주파수 튜플로 검증해 반환한다."""
        values = tuple(int(value) for value in self.get_parameter(name).value)
        if not values or any(value <= 0 for value in values):
            raise ValueError(f"{name} must contain positive frequencies")
        return values

    def _on_goal(self, request) -> GoalResponse:
        """로봇 ID·이벤트·현장 좌표·version을 검증해 탐색 Goal을 수락한다."""
        if self.busy or request.robot_id != self.robot_id:
            return GoalResponse.REJECT
        if not request.mission_id or not request.event_id:
            return GoalResponse.REJECT
        if not request.patient_pose.header.frame_id:
            return GoalResponse.REJECT
        latest = self.latest_versions.get(request.event_id, -1)
        if request.mission_version <= latest:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _on_cancel(_goal_handle) -> CancelResponse:
        """운영자 또는 coordinator의 현장 탐색 취소 요청을 수락한다."""
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        """구조 인력이 감지될 때까지 회전·호출하고 감지 즉시 안내음을 낸다."""
        request = goal_handle.request
        self.busy = True
        self.current_request = request
        self.latest_versions[request.event_id] = request.mission_version
        # 이전 임무나 노드 시작 전의 true 신호가 새 탐색을 끝내지 못하게 초기화한다.
        self.helper_confirmed = False
        self.helper_observed_at = None
        result = GuideHelper.Result()
        started_at = time.monotonic()
        next_buzzer_at = 0.0
        try:
            self._publish_status(
                MissionStatus.HELPER_REQUESTED,
                "rotating in place and calling for a helper",
            )
            while self.wait_timeout == 0.0 or (
                time.monotonic() - started_at < self.wait_timeout
            ):
                if goal_handle.is_cancel_requested:
                    return self._terminate(
                        goal_handle, result, GuideHelper.Result.CANCELED
                    )
                now = time.monotonic()
                if self._vision_confirmed(now):
                    self._stop_rotation()
                    self._stop_audio()
                    self._publish_guide_tone()
                    goal_handle.succeed()
                    result.code = GuideHelper.Result.SUCCEEDED
                    result.reason = (
                        "helper detected; search stopped and guidance tone "
                        "played"
                    )
                    result.finished_at = self.get_clock().now().to_msg()
                    self._publish_status(
                        MissionStatus.HELPER_ARRIVED, result.reason
                    )
                    self._publish_status(
                        MissionStatus.COMPLETED, result.reason
                    )
                    return result

                self._publish_rotation()
                if now >= next_buzzer_at:
                    self._publish_call_tone()
                    next_buzzer_at = now + self.buzzer_period
                self._publish_feedback(
                    goal_handle,
                    "rotating and waiting for aed_vision helper confirmation",
                )
                await self._delay(self.control_period)

            return self._terminate(
                goal_handle,
                result,
                GuideHelper.Result.HELPER_NOT_FOUND,
                "helper wait timeout expired",
            )
        except Exception as error:
            self.get_logger().error(f"On-site helper scan failed: {error}")
            return self._terminate(
                goal_handle,
                result,
                GuideHelper.Result.NAVIGATION_FAILED,
                str(error),
            )
        finally:
            self._stop_rotation()
            if result.code != GuideHelper.Result.SUCCEEDED:
                self._stop_audio()
            self.current_request = None
            self.busy = False

    def _on_helper_confirmed(self, message: Bool) -> None:
        """aed_vision의 최신 구조 인력 확정 여부와 수신 시각을 저장한다."""
        self.helper_confirmed = bool(message.data)
        self.helper_observed_at = time.monotonic()

    def _vision_confirmed(self, now: float) -> bool:
        """Vision의 true 신호가 최신 데이터일 때만 구조 인력 감지로 인정한다."""
        return helper_confirmation_is_fresh(
            confirmed=self.helper_confirmed,
            observed_at=self.helper_observed_at,
            now=now,
            stale_seconds=self.vision_stale,
        )

    def _publish_rotation(self) -> None:
        """선속도 없이 양의 각속도만 발행해 로봇을 제자리 회전시킨다."""
        command = Twist()
        command.angular.z = self.rotation_speed
        self.cmd_vel_publisher.publish(command)

    def _stop_rotation(self) -> None:
        """감지·취소·오류·종료 모든 경로에서 0 속도를 발행한다."""
        self.cmd_vel_publisher.publish(Twist())

    def _publish_call_tone(self) -> None:
        """구조 인력을 부르는 반복 2음 임시 신호를 발행한다."""
        self._publish_audio(
            "CALL", self.buzzer_frequencies, self.buzzer_duration
        )

    def _publish_guide_tone(self) -> None:
        """추후 TTS로 교체할 상승 3음 안내 신호를 한 번 발행한다."""
        self._publish_audio(
            "GUIDE", self.guide_frequencies, self.guide_duration
        )

    def _publish_audio(
        self, label: str, frequencies: tuple[int, ...], duration: float
    ) -> None:
        """TurtleBot4 AudioNoteVector 또는 개발용 문자열로 임시 음을 출력한다."""
        if not self.uses_create_audio:
            self.audio_publisher.publish(
                String(data=f"{label} " + ",".join(map(str, frequencies)))
            )
            return
        message = AudioNoteVector()
        message.append = False
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1_000_000_000)
        for frequency in frequencies:
            note = AudioNote()
            note.frequency = frequency
            note.max_runtime.sec = seconds
            note.max_runtime.nanosec = nanoseconds
            message.notes.append(note)
        self.audio_publisher.publish(message)

    def _stop_audio(self) -> None:
        """호출음 큐를 비워 구조 인력 감지 즉시 반복음을 중지한다."""
        if not self.uses_create_audio:
            self.audio_publisher.publish(String(data="STOP"))
            return
        message = AudioNoteVector()
        message.append = False
        self.audio_publisher.publish(message)

    def _publish_feedback(self, goal_handle, detail: str) -> None:
        """회전 탐색 단계를 coordinator에 Action feedback으로 전달한다."""
        feedback = GuideHelper.Feedback()
        feedback.phase = GuideHelper.Goal.PHASE_CALLING_HELPER
        feedback.helper_distance_m = -1.0
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _terminate(self, goal_handle, result, code: int, reason: str = ""):
        """탐색 실패 또는 취소 결과와 공통 MissionStatus를 안전하게 발행한다."""
        reasons = {
            GuideHelper.Result.CANCELED: "helper scan canceled",
            GuideHelper.Result.NAVIGATION_FAILED: "helper scan failed",
            GuideHelper.Result.HELPER_NOT_FOUND: "helper was not detected",
        }
        result.code = code
        result.reason = reason or reasons.get(code, "helper scan failed")
        result.finished_at = self.get_clock().now().to_msg()
        if code == GuideHelper.Result.CANCELED:
            goal_handle.canceled()
            self._publish_status(MissionStatus.CANCELED, result.reason)
        else:
            goal_handle.abort()
            self._publish_status(MissionStatus.NAVIGATION_ERROR, result.reason)
        return result

    def _publish_status(self, state: int, reason: str = "") -> None:
        """현재 탐색 임무 식별자와 상태를 공통 MissionStatus 토픽에 발행한다."""
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
        """ROS executor를 막지 않는 타이머 Future로 제어 주기를 기다린다."""
        future = Future()
        timer_holder = {}

        def wake() -> None:
            """일회성 타이머를 제거하고 대기 중인 Future를 완료한다."""
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
        """노드 종료 전 반드시 회전과 호출음을 정지하고 Action 서버를 닫는다."""
        self._stop_rotation()
        self._stop_audio()
        self.action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    """로봇 측 탐색 controller를 다중 스레드 executor로 실행한다."""
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
