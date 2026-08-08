"""MissionStatus만 구독해 Nav2와 독립적으로 출동 경보를 제어한다."""

import time
from math import isfinite

from aed_interfaces.msg import MissionStatus
import rclpy
from rclpy.node import Node

from emergency_alert.alert_logic import (
    AlertCommand,
    MissionAlertPolicy,
    MissionPhase,
    TonePattern,
)
from emergency_alert.audio_output import AudioOutput


class MissionStatusAlert(Node):
    """로봇의 AED 임무 상태를 출동·도착·중단 경보음으로 변환한다."""

    def __init__(self) -> None:
        """경보 패턴과 상태 구독 및 주기 재생 타이머를 초기화한다."""
        super().__init__("mission_status_alert")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("mission_status_topic", "/aed/mission_status")
        self.declare_parameter("audio_topic", "cmd_audio")
        self.declare_parameter("audio_backend", "system")
        self.declare_parameter("audio_player", "auto")
        self.declare_parameter("audio_device", "")
        self.declare_parameter("mission_id_suffix", "-aed")
        self.declare_parameter("alarm_period", 0.8)
        self.declare_parameter("maximum_alarm_duration", 600.0)
        self.declare_parameter("travel_note_duration", 0.25)
        self.declare_parameter("travel_frequencies", [1000, 440])
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
        self.mission_id_suffix = str(
            self.get_parameter("mission_id_suffix").value
        )
        self.alarm_period = self._positive("alarm_period")
        self.maximum_alarm_duration = self._positive(
            "maximum_alarm_duration"
        )
        travel_duration = self._positive("travel_note_duration")
        terminal_duration = self._positive("terminal_note_duration")
        self.travel_pattern = TonePattern.from_values(
            self.get_parameter("travel_frequencies").value,
            travel_duration,
        )
        self.arrival_pattern = TonePattern.from_values(
            self.get_parameter("arrival_frequencies").value,
            terminal_duration,
        )
        self.interrupted_pattern = TonePattern.from_values(
            self.get_parameter("interrupted_frequencies").value,
            terminal_duration,
        )
        if self.alarm_period < self.travel_pattern.total_duration:
            raise ValueError(
                "alarm_period must be at least the travel pattern duration"
            )

        self.policy = MissionAlertPolicy(self.robot_id)
        self.audio = AudioOutput(
            self,
            str(self.get_parameter("audio_topic").value),
            str(self.get_parameter("audio_backend").value),
            str(self.get_parameter("audio_player").value),
            str(self.get_parameter("audio_device").value),
        )
        self.alarm_active = False
        self.alarm_started_at = None
        self.create_subscription(
            MissionStatus,
            str(self.get_parameter("mission_status_topic").value),
            self._on_status,
            20,
        )
        self.alarm_timer = self.create_timer(
            self.alarm_period, self._alarm_tick
        )
        self.audio.stop()
        self.get_logger().info(
            f"ready: robot={self.robot_id}, mode=status-only, "
            f"audio={self.audio.topic}"
        )

    def _positive(self, name: str) -> float:
        """양수여야 하는 실수형 파라미터를 검증하여 반환한다."""
        value = float(self.get_parameter(name).value)
        if not isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _on_status(self, message: MissionStatus) -> None:
        """MissionStatus를 중복 제거 정책에 통과시킨 뒤 오디오 명령을 실행한다."""
        if (
            self.mission_id_suffix
            and not message.mission_id.endswith(self.mission_id_suffix)
        ):
            return
        command = self.policy.handle(
            robot_id=message.robot_id,
            event_id=message.event_id,
            assignment_version=message.assignment_version,
            phase=self._phase(message.status),
        )
        self._apply(command, message)

    @staticmethod
    def _phase(status: int) -> MissionPhase:
        """공통 MissionStatus 상수를 경보 정책에서 사용하는 단계로 변환한다."""
        if status == MissionStatus.ASSIGNED:
            return MissionPhase.ASSIGNED
        if status == MissionStatus.DISPATCHING:
            return MissionPhase.DISPATCHING
        if status == MissionStatus.EN_ROUTE:
            return MissionPhase.EN_ROUTE
        if status == MissionStatus.ARRIVED:
            return MissionPhase.ARRIVED
        if status == MissionStatus.COMPLETED:
            return MissionPhase.COMPLETED
        if status in (
            MissionStatus.CANCELED,
            MissionStatus.BLOCKED,
            MissionStatus.NETWORK_LOST,
            MissionStatus.NAVIGATION_ERROR,
        ):
            return MissionPhase.INTERRUPTED
        return MissionPhase.UNRELATED

    def _apply(self, command: AlertCommand, message: MissionStatus) -> None:
        """정책이 반환한 명령 하나를 실제 오디오 상태와 로그에 반영한다."""
        if command is AlertCommand.IGNORE:
            return
        if command is AlertCommand.STOP:
            self._stop_alarm()
            return
        if command is AlertCommand.START_TRAVEL:
            self.alarm_active = True
            self.alarm_started_at = time.monotonic()
            self.audio.play(self.travel_pattern)
            self.get_logger().info(
                f"travel alert started: event={message.event_id}, "
                f"version={message.assignment_version}"
            )
            return

        self.alarm_active = False
        self.alarm_started_at = None
        if command is AlertCommand.PLAY_ARRIVAL:
            self.audio.play(self.arrival_pattern)
            self.get_logger().info(
                f"arrival alert: event={message.event_id}"
            )
        elif command is AlertCommand.PLAY_INTERRUPTED:
            self.audio.play(self.interrupted_pattern)
            self.get_logger().warning(
                f"interrupted alert: event={message.event_id}, "
                f"reason={message.reason}"
            )

    def _alarm_tick(self) -> None:
        """이동 상태인 동안 설정 주기마다 출동 경보 패턴을 반복 발행한다."""
        if self.alarm_active:
            elapsed = time.monotonic() - self.alarm_started_at
            if elapsed >= self.maximum_alarm_duration:
                self.get_logger().error(
                    "maximum alarm duration exceeded; stopping stale alert"
                )
                self._stop_alarm()
                return
            self.audio.play(self.travel_pattern)

    def _stop_alarm(self) -> None:
        """반복 상태를 해제하고 Create3에 남은 오디오 큐를 비운다."""
        was_active = self.alarm_active
        self.alarm_active = False
        self.alarm_started_at = None
        self.policy.mark_output_stopped()
        self.audio.stop()
        if was_active:
            self.get_logger().info("travel alert stopped")

    def destroy_node(self) -> None:
        """노드 종료 전에 반복 재생과 Create3 오디오 큐를 안전하게 정리한다."""
        self._stop_alarm()
        super().destroy_node()


def main(args=None) -> None:
    """상태 구독형 경보 노드를 초기화하고 종료될 때까지 실행한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = MissionStatusAlert()
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
