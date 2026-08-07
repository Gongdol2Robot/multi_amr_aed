"""프로세스 하나가 들고 있는 것들을 한 곳에 모은다.

라우터마다 전역 변수를 두면 시험할 때 갈아끼울 수 없다. 여기 담아 두고
app.state 에 붙여 두면 시험에서는 임시 저장소를 넣은 Context 를 만들면 된다.
"""

import logging
from dataclasses import dataclass
from typing import Optional

from .domain.models import Point2D
from .ros.topics import DEFAULT_STREAMS
from .store.repository import Repository
from .stream.frames import FrameRegistry
from .stream.hub import Hub
from .stream.live_state import LiveState

LOGGER = logging.getLogger(__name__)


@dataclass
class Settings:
    database_path: str = "var/aed_hmi.sqlite3"
    # 로봇 상태를 몇 초마다 저장할지. 전량 저장하면 DB 가 금방 커진다.
    robot_sample_interval_s: float = 1.0
    host: str = "0.0.0.0"
    port: int = 8000
    cors_origins: tuple[str, ...] = (
        "http://localhost:5173", "http://127.0.0.1:5173",
    )


class Context:
    """살아 있는 상태와 저장소를 묶는다."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.repository = Repository(settings.database_path)
        self.live = LiveState()
        self.frames = FrameRegistry(DEFAULT_STREAMS)
        self.hub = Hub(self.build_snapshot)
        self.bridge = None
        self._last_sample_at: dict[str, float] = {}

    # ------------------------------------------------------------------
    # ROS bridge가 부르는 입구
    # ------------------------------------------------------------------

    def on_robot(self, robot) -> None:
        self.live.put_robot(robot)
        self._maybe_sample(robot)

    def on_event(self, event) -> None:
        self.live.put_event(event)
        try:
            self.repository.upsert_event(event)
        except Exception:
            LOGGER.exception("응급 이벤트 저장 실패: %s", event.event_id)

    def on_mission(self, mission) -> None:
        self.live.put_mission(mission)
        try:
            self.repository.insert_mission_event(mission)
        except Exception:
            LOGGER.exception("임무 이벤트 저장 실패: %s", mission.mission_id)

    def on_frame(self, stream_id: str, jpeg: bytes) -> None:
        self.frames.put(stream_id, jpeg)

    def on_person_count(self, stream_id: str, count: int) -> None:
        """vision_detector 가 매 프레임 내는 사람 수. 검출 표시의 근거다."""
        self.live.put_person_count(stream_id, count)

    def on_eta_record(self, record) -> None:
        """도착 예상·실제 한 쌍. 통계로만 쓰므로 화면 상태에는 안 넣는다."""
        try:
            self.repository.upsert_eta_record(record)
        except Exception:
            LOGGER.exception("ETA 기록 저장 실패: %s", record.request_id)

    def on_assignment(
        self, mission_id: str, version: int, event_id: str, robot_id: str,
        role: str, target: Point2D, assigned_at: float,
    ) -> None:
        self.live.set_mission_target(mission_id, assigned_at, target)
        try:
            self.repository.insert_assignment(
                mission_id, version, event_id, robot_id, role,
                target, assigned_at,
            )
        except Exception:
            LOGGER.exception("배정 저장 실패: %s", mission_id)

    def _maybe_sample(self, robot) -> None:
        """로봇 상태는 솎아서 저장한다. 10Hz 를 그대로 넣으면 하루에 백만 행이다."""
        last = self._last_sample_at.get(robot.robot_id, 0.0)
        if robot.stamp - last < self.settings.robot_sample_interval_s:
            return
        self._last_sample_at[robot.robot_id] = robot.stamp
        try:
            self.repository.insert_robot_sample(robot)
        except Exception:
            LOGGER.exception("로봇 표본 저장 실패: %s", robot.robot_id)

    # ------------------------------------------------------------------
    # 화면이 보는 현재 상태
    # ------------------------------------------------------------------

    def build_snapshot(self):
        connected = bool(self.bridge and self.bridge.connected)
        # 검출 수는 영상 토픽이 아니라 EmergencyEvent 에서 온다. 영상 갈래와
        # 이벤트를 여기서 맞붙인다. stream_id 와 camera_id 가 같은 이름을
        # 쓰기로 한 약속이 이 연결의 전부다.
        from dataclasses import replace

        counts = self.live.detection_counts()
        streams = [
            replace(health, detections=counts.get(health.stream_id, 0))
            for health in self.frames.health()
        ]
        return self.live.snapshot(streams, connected)

    def shutdown(self) -> None:
        if self.bridge is not None:
            self.bridge.stop()
        self.repository.close()
