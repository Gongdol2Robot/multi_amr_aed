"""Publish a configurable emergency siren on a TurtleBot4 audio topic."""

import rclpy
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from rclpy.node import Node


class SirenNode(Node):
    """Play one emergency siren sequence and exit."""

    def __init__(self) -> None:
        super().__init__("emergency_siren")
        self.declare_parameter("audio_topic", "cmd_audio")
        self.declare_parameter("repeat", 2)
        self.declare_parameter("note_duration", 0.3)

        topic = str(self.get_parameter("audio_topic").value)
        self.repeat = max(1, int(self.get_parameter("repeat").value))
        self.duration = float(self.get_parameter("note_duration").value)
        if self.duration <= 0.0:
            raise ValueError("note_duration must be positive")

        self.publisher = self.create_publisher(AudioNoteVector, topic, 10)
        self.timer = self.create_timer(0.5, self._play_once)

    def _play_once(self) -> None:
        message = AudioNoteVector()
        message.append = False
        seconds = int(self.duration)
        nanoseconds = int((self.duration - seconds) * 1_000_000_000)

        for _ in range(self.repeat):
            for frequency in (1000, 440):
                note = AudioNote()
                note.frequency = frequency
                note.max_runtime.sec = seconds
                note.max_runtime.nanosec = nanoseconds
                message.notes.append(note)

        self.publisher.publish(message)
        self.get_logger().info("Emergency siren published")
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None) -> None:
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

