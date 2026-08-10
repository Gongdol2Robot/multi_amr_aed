"""Bluetooth/system speaker and optional Create 3 audio output."""

from io import BytesIO
import math
import os
import shutil
import struct
import subprocess
import threading
import wave

from std_msgs.msg import String

from emergency_alert.alert_logic import TonePattern

try:
    from irobot_create_msgs.msg import AudioNote, AudioNoteVector
except ImportError:  # ROS만 설치된 개발 PC에서도 상태 로직을 실행할 수 있다.
    AudioNote = None
    AudioNoteVector = None


class SystemSpeakerOutput:
    """Play generated tones through the PC default PipeWire/PulseAudio sink."""

    _SAMPLE_RATE = 44100

    def __init__(self, node, player: str = "auto", device: str = "") -> None:
        self._node = node
        self._device = device.strip()
        self._player = self._resolve_player(player.strip())
        self._lock = threading.Lock()
        self._process = None
        self._generation = 0
        if self._player is None:
            node.get_logger().error(
                "No system audio player found. Install paplay or pw-play; "
                "Bluetooth call sounds are disabled."
            )
        else:
            target = self._device or "OS default output"
            node.get_logger().info(
                f"system speaker ready: player={self._player}, target={target}"
            )

    @staticmethod
    def _resolve_player(requested: str):
        """Return an available supported player, preferring PulseAudio."""
        if requested and requested != "auto":
            if requested not in ("paplay", "pw-play", "aplay"):
                raise ValueError(
                    "audio_player must be auto, paplay, pw-play, or aplay"
                )
            return requested if shutil.which(requested) else None
        for candidate in ("paplay", "pw-play", "aplay"):
            if shutil.which(candidate):
                return candidate
        return None

    def play(self, pattern: TonePattern) -> None:
        """Replace current playback with one synthesized tone pattern."""
        if self._player is None:
            return
        payload = self._wav_bytes(pattern)
        self.stop()
        with self._lock:
            self._generation += 1
            generation = self._generation
        threading.Thread(
            target=self._play_worker,
            args=(generation, payload, ""),
            daemon=True,
            name="aed-system-speaker",
        ).start()

    def play_file(self, path: str, loop: bool = False) -> None:
        """Replace current playback with an audio file, optionally looping."""
        if self._player is None:
            return
        if not os.path.isfile(path):
            raise FileNotFoundError(f"audio file does not exist: {path}")
        self.stop()
        with self._lock:
            self._generation += 1
            generation = self._generation
        threading.Thread(
            target=self._play_worker,
            args=(generation, None, path, loop),
            daemon=True,
            name="aed-system-speaker",
        ).start()

    def stop(self) -> None:
        """Immediately stop the process playing the previous pattern."""
        with self._lock:
            self._generation += 1
            process = self._process
            self._process = None
        if process is not None and process.poll() is None:
            process.terminate()

    def _command(self, source: str) -> list[str]:
        command = [self._player]
        if self._device:
            if self._player == "paplay":
                command.append(f"--device={self._device}")
            elif self._player == "pw-play":
                command.append(f"--target={self._device}")
            else:
                command.extend(["-D", self._device])
        if source:
            command.append(source)
        elif self._player != "paplay":
            # aplay와 pw-play는 "-"를 stdin으로 읽는다. paplay는 그렇지 않고
            # 이름이 "-"인 파일을 열려다 실패하므로 인자를 아예 붙이지 않는다
            # (paplay는 파일 인자가 없으면 stdin에서 읽는다). 이 한 글자 때문에
            # 합성 톤인 출동·도착·중단 경보가 소리 없이 전부 실패하고 있었다.
            command.append("-")
        return command

    def _play_worker(
        self,
        generation: int,
        payload: bytes | None,
        source: str,
        loop: bool = False,
    ) -> None:
        try:
            while True:
                with self._lock:
                    if generation != self._generation:
                        return
                process = subprocess.Popen(
                    self._command(source),
                    stdin=subprocess.PIPE if payload is not None else None,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                with self._lock:
                    if generation != self._generation:
                        process.terminate()
                        return
                    self._process = process
                process.communicate(payload)
                with self._lock:
                    if self._process is process:
                        self._process = None
                    stopped = generation != self._generation
                if not loop or stopped:
                    return
        except (OSError, subprocess.SubprocessError) as error:
            self._node.get_logger().error(
                f"system speaker playback failed: {error}"
            )
        finally:
            with self._lock:
                if self._process is locals().get("process"):
                    self._process = None

    @classmethod
    def _wav_bytes(cls, pattern: TonePattern) -> bytes:
        """Synthesize a mono PCM WAV with a short click-prevention fade."""
        output = BytesIO()
        with wave.open(output, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(cls._SAMPLE_RATE)
            frames = bytearray()
            sample_count = max(
                1, int(pattern.note_duration * cls._SAMPLE_RATE)
            )
            fade_count = min(sample_count // 2, int(0.01 * cls._SAMPLE_RATE))
            for frequency in pattern.frequencies:
                for index in range(sample_count):
                    gain = 0.35
                    if fade_count:
                        gain *= min(
                            1.0,
                            index / fade_count,
                            (sample_count - 1 - index) / fade_count,
                        )
                    value = int(
                        32767
                        * gain
                        * math.sin(
                            2.0
                            * math.pi
                            * frequency
                            * index
                            / cls._SAMPLE_RATE
                        )
                    )
                    frames.extend(struct.pack("<h", value))
            wav_file.writeframes(frames)
        return output.getvalue()


class AudioOutput:
    """Provide replaceable system-speaker or legacy Create 3 tone output."""

    def __init__(
        self,
        node,
        topic: str = "cmd_audio",
        backend: str = "system",
        player: str = "auto",
        device: str = "",
    ) -> None:
        if backend not in ("system", "create3"):
            raise ValueError("audio_backend must be system or create3")
        if not topic:
            raise ValueError("audio topic must not be empty")
        self._node = node
        self._topic = topic
        self.backend = backend
        self.uses_create3_audio = (
            backend == "create3" and AudioNoteVector is not None
        )
        self._system = None
        if backend == "system":
            self._publisher = None
            self._system = SystemSpeakerOutput(node, player, device)
        elif self.uses_create3_audio:
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
        """Return system or ROS output description for logs."""
        return "system-default" if self.backend == "system" else self._topic

    def play(self, pattern: TonePattern) -> None:
        """Replace the current queue/playback with one validated pattern."""
        if self._system is not None:
            self._system.play(pattern)
            return
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

    def play_file(
        self, path: str, fallback: TonePattern, loop: bool = False
    ) -> None:
        """Play an asset on the system backend, or tones on Create 3."""
        if self._system is not None:
            self._system.play_file(path, loop=loop)
        else:
            self.play(fallback)

    def stop(self) -> None:
        """Stop system playback or clear the Create 3 audio queue."""
        if self._system is not None:
            self._system.stop()
            return
        if not self.uses_create3_audio:
            self._publisher.publish(String(data="STOP"))
            return
        message = AudioNoteVector()
        message.append = False
        self._publisher.publish(message)
