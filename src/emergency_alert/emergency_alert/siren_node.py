"""TurtleBot4에 설정 가능한 단발 긴급 경보음을 발행한다."""

from math import isfinite

import rclpy
from rclpy.node import Node

from emergency_alert.alert_logic import TonePattern
from emergency_alert.audio_output import AudioOutput


class SirenNode(Node):
    """
    높은 음과 낮은 음의 단발 경보 시퀀스를 재생하는 ROS 2 노드.

    출동 내내 반복되는 경보는 ``AlertMissionExecutor``가 담당한다. 이 노드는
    스피커 연결 확인이나 독립적인 경보 시험에 사용하고, 한 번 발행한 뒤
    자동으로 종료한다.
    """

    def __init__(self) -> None:
        """음 길이와 반복 횟수를 읽고 cmd_audio publisher를 준비한다."""
        super().__init__("emergency_siren")
        self.declare_parameter("audio_topic", "cmd_audio")
        self.declare_parameter("repeat", 2)
        self.declare_parameter("note_duration", 0.3)

        topic = str(self.get_parameter("audio_topic").value)
        self.repeat = max(1, int(self.get_parameter("repeat").value))
        self.duration = float(self.get_parameter("note_duration").value)
        if not isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError("note_duration must be positive")

        self.audio = AudioOutput(self, topic)
        self.pattern = TonePattern.from_values(
            (1000, 440) * self.repeat, self.duration
        )
        self.timer = self.create_timer(0.5, self._play_once)
        self.finish_timer = None

    def _play_once(self) -> None:
        """설정된 횟수만큼 1000 Hz와 440 Hz 음계를 묶어 한 번 발행한다."""
        self.audio.play(self.pattern)
        self.get_logger().info("Emergency siren published")
        self.timer.cancel()
        self.finish_timer = self.create_timer(
            self.pattern.total_duration + 0.2, self._finish
        )

    def _finish(self) -> None:
        """음계의 최대 재생시간이 지난 뒤 단발 노드를 정상 종료한다."""
        if self.finish_timer is not None:
            self.finish_timer.cancel()
        rclpy.shutdown()


def main(args=None) -> None:
    """ROS를 초기화하고 단발 경보 발행이 끝날 때까지 노드를 실행한다."""
    rclpy.init(args=args)
    node = SirenNode()
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
