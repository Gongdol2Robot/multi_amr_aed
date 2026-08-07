"""TurtleBot4 Create3 오디오와 개발 환경 대체 출력을 캡슐화한다."""

from std_msgs.msg import String

from emergency_alert.alert_logic import TonePattern

try:
    from irobot_create_msgs.msg import AudioNote, AudioNoteVector
except ImportError:  # ROS만 설치된 개발 PC에서도 상태 로직을 실행할 수 있다.
    AudioNote = None
    AudioNoteVector = None


class AudioOutput:
    """음 패턴 재생과 정지를 하나의 교체 가능한 출력 경계로 제공한다."""

    def __init__(self, node, topic: str) -> None:
        """Create3 메시지가 있으면 실제 오디오, 없으면 String publisher를 만든다."""
        if not topic:
            raise ValueError("audio topic must not be empty")
        self._node = node
        self._topic = topic
        self.uses_create3_audio = AudioNoteVector is not None
        if self.uses_create3_audio:
            self._publisher = node.create_publisher(AudioNoteVector, topic, 10)
        else:
            fallback_topic = f"{topic}_fallback"
            self._publisher = node.create_publisher(String, fallback_topic, 10)
            node.get_logger().warning(
                "irobot_create_msgs unavailable; BEEP/STOP strings are sent "
                f"to {fallback_topic} and no physical sound is produced"
            )

    @property
    def topic(self) -> str:
        """설정된 실제 Create3 오디오 토픽명을 반환한다."""
        return self._topic

    def play(self, pattern: TonePattern) -> None:
        """기존 큐를 교체하는 방식으로 검증된 음 패턴을 한 번 발행한다."""
        if not self.uses_create3_audio:
            message = String()
            message.data = "BEEP " + ",".join(
                str(value) for value in pattern.frequencies
            )
            self._publisher.publish(message)
            return

        message = AudioNoteVector()
        message.append = False
        seconds = int(pattern.note_duration)
        nanoseconds = int(
            (pattern.note_duration - seconds) * 1_000_000_000
        )
        for frequency in pattern.frequencies:
            note = AudioNote()
            note.frequency = frequency
            note.max_runtime.sec = seconds
            note.max_runtime.nanosec = nanoseconds
            message.notes.append(note)
        self._publisher.publish(message)

    def stop(self) -> None:
        """Create3 큐를 비우거나 대체 출력에 STOP 명령을 보내 경보를 정지한다."""
        if not self.uses_create3_audio:
            message = String()
            message.data = "STOP"
            self._publisher.publish(message)
            return
        message = AudioNoteVector()
        message.append = False
        self._publisher.publish(message)
