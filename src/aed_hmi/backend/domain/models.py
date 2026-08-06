"""화면과 저장소가 주고받는 타입. ROS 를 모른다.

여기 있는 것만 WebSocket 과 REST 로 나간다. frontend/src/types 와 1:1 이다.
한쪽을 고치면 다른 쪽도 고쳐야 하므로, 필드 이름을 임의로 바꾸지 않는다.

시각은 전부 UTC epoch 초(float)로 통일한다. ROS 의 builtin_interfaces/Time,
SQLite 의 REAL, 자바스크립트의 Date 사이를 오갈 때 이게 가장 덜 깨진다.
"""

from dataclasses import dataclass, field
from typing import Optional

from .enums import EventStatus, MissionState, RobotAvailability, RobotRole


@dataclass(frozen=True)
class Point2D:
    """지도 평면 좌표. 고도는 쓰지 않는다."""

    x: float
    y: float


@dataclass(frozen=True)
class RobotSnapshot:
    """RobotState.msg 한 건. 관제 화면의 로봇 카드 하나에 대응한다."""

    robot_id: str
    stamp: float
    position: Point2D
    yaw_deg: float
    battery_percentage: float
    availability: RobotAvailability
    role: RobotRole
    mission_id: str
    is_docked: bool
    network_ok: bool
    localization_ok: bool
    nav2_ok: bool
    emergency_stop: bool
    path_valid: bool
    estimated_path_cost: float
    last_heartbeat: float
    detail: str
    # 아래 둘은 RobotState.msg 에 없다. 연속된 pose 로 백엔드가 계산한다.
    speed_mps: float = 0.0
    # 하트비트가 끊긴 지 몇 초 됐는지. 화면에서 통신 상태 판단에 쓴다.
    heartbeat_age_s: float = 0.0

    @property
    def healthy(self) -> bool:
        """운영자가 "이 로봇 지금 쓸 수 있나"를 한 눈에 보는 기준."""
        return (
            self.network_ok
            and self.localization_ok
            and self.nav2_ok
            and not self.emergency_stop
        )


@dataclass(frozen=True)
class EmergencyEventSnapshot:
    """EmergencyEvent.msg 한 건. 신고 또는 웹캠 검출로 생긴다."""

    event_id: str
    detected_at: float
    location: Point2D
    frame_id: str
    confidence: float
    consecutive_detections: int
    status: EventStatus
    source_id: str
    camera_id: str
    zone_id: str


@dataclass(frozen=True)
class MissionEvent:
    """MissionStatus.msg 한 건. 임무의 상태 전이 하나를 뜻한다."""

    mission_id: str
    event_id: str
    robot_id: str
    assignment_version: int
    state: MissionState
    stamp: float
    reason: str


@dataclass(frozen=True)
class MissionSummary:
    """한 임무의 전 과정 요약. 이력 조회와 통계의 단위다.

    called_at 부터 arrived_at 까지가 운영자가 가장 궁금해하는 값이다.
    도착 못 한 임무는 arrived_at 이 None 이고, 그것도 의미 있는 결과다.
    """

    mission_id: str
    event_id: str
    robot_id: str
    target: Point2D
    called_at: float
    dispatched_at: Optional[float]
    arrived_at: Optional[float]
    final_state: MissionState
    assignment_version: int
    reassignment_count: int
    failure_reasons: list[str] = field(default_factory=list)

    # 진행 중인 임무에만 채워진다. 끝난 임무는 실제 도착 시각이 있으므로
    # 추정이 필요 없고, 남겨두면 어느 쪽이 사실인지 헷갈린다.
    eta_seconds: Optional[float] = None
    eta_distance_m: Optional[float] = None
    eta_confident: bool = False

    @property
    def response_seconds(self) -> Optional[float]:
        """신고에서 AED 도착까지. 이 시스템의 존재 이유를 재는 값이다."""
        if self.arrived_at is None:
            return None
        return self.arrived_at - self.called_at

    @property
    def eta_at(self) -> Optional[float]:
        """도착 예상 시각(epoch). 화면에서 시계로 보여주기 위한 값."""
        if self.eta_seconds is None:
            return None
        import time

        return time.time() + self.eta_seconds


@dataclass(frozen=True)
class StreamHealth:
    """영상 한 갈래의 상태. 화면 타일 하나에 대응한다."""

    stream_id: str
    label: str
    kind: str          # "robot" 또는 "webcam"
    online: bool
    fps: float
    last_frame_at: Optional[float]
    detections: int    # 최근 프레임의 YOLO 검출 수


@dataclass(frozen=True)
class SystemSnapshot:
    """WebSocket 으로 주기 전송하는 전체 상태. 화면 한 장 그리는 데 필요한 전부."""

    stamp: float
    robots: list[RobotSnapshot]
    active_event: Optional[EmergencyEventSnapshot]
    active_missions: list[MissionSummary]
    streams: list[StreamHealth]
    ros_connected: bool
