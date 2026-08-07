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
class EtaRecord:
    """도착 예상과 실제를 한 쌍으로 잰 결과.

    출동 한 건이 끝날 때 한 번 생긴다. 예상이 얼마나 맞았는지는 이 값을
    쌓아야만 알 수 있고, 그래야 예상에 쓰는 계수(순항 속도, 우회 계수)를
    근거를 갖고 고칠 수 있다.

    `request_id` 는 보내는 쪽의 이벤트 식별자다. 임무 식별자는 거기에
    `-aed` 를 붙인 것이라 따로 싣지 않는다.
    """

    request_id: str
    robot_id: str
    predicted_sec: float
    actual_sec: float
    status: str
    stamp: float

    @property
    def mission_id(self) -> str:
        return f"{self.request_id}-aed"

    @property
    def error_sec(self) -> float:
        """양수면 예상보다 늦게 도착했다는 뜻이다.

        보내는 쪽도 error_sec 을 실어 보내지만 다시 계산한다. 두 값이
        다르면 어느 쪽이 맞는지 알 수 없고, 여기서 세 수를 모두 갖고
        있으므로 굳이 남의 뺄셈을 믿을 이유가 없다.
        """
        return self.actual_sec - self.predicted_sec

    @classmethod
    def from_json(cls, payload: str) -> Optional["EtaRecord"]:
        """`/emergency/eta/result` 의 JSON 문자열을 읽는다.

        이 값만 `std_msgs/String` 에 JSON 으로 온다. 다른 토픽은 .msg 가
        칸과 형을 보장하지만 여기는 보내는 쪽이 무엇을 넣든 통과한다.
        그래서 받는 자리에서 한 번 검사한다. **이 시스템에서 형이 보장되지
        않은 값이 들어오는 유일한 통로다.**

        깨진 것이 오면 None 을 준다. 통계 한 건을 잃는 것이 관제 화면을
        끄는 것보다 낫다.

        ROS 를 모르는 자리에 두는 이유: 받는 것이 문자열이라 ROS 타입이
        필요 없고, 목업도 같은 함수를 거쳐야 실제와 같은 경로가 된다.
        """
        import json
        import logging

        logger = logging.getLogger(__name__)

        try:
            data = json.loads(payload)
        except (json.JSONDecodeError, TypeError):
            logger.warning("ETA 결과가 JSON 이 아니다: %.120s", payload)
            return None
        if not isinstance(data, dict):
            logger.warning("ETA 결과가 객체가 아니다: %.120s", payload)
            return None

        required = ("request_id", "robot_id", "predicted_eta_sec",
                    "actual_arrival_sec", "stamp_sec")
        missing = [key for key in required if key not in data]
        if missing:
            logger.warning("ETA 결과에 빠진 칸: %s", ", ".join(missing))
            return None

        try:
            record = cls(
                request_id=str(data["request_id"]),
                robot_id=str(data["robot_id"]),
                predicted_sec=float(data["predicted_eta_sec"]),
                actual_sec=float(data["actual_arrival_sec"]),
                status=str(data.get("status", "")),
                stamp=float(data["stamp_sec"]),
            )
        except (TypeError, ValueError):
            logger.warning("ETA 결과의 숫자 칸을 못 읽었다: %.120s", payload)
            return None

        if not record.request_id:
            logger.warning("ETA 결과에 request_id 가 비었다")
            return None
        # 음수 시간은 시계가 어긋났다는 뜻이다. 평균을 끌고 가므로 버린다.
        if record.predicted_sec < 0 or record.actual_sec < 0:
            logger.warning(
                "ETA 결과의 시간이 음수다: %s predicted=%.3f actual=%.3f",
                record.request_id, record.predicted_sec, record.actual_sec,
            )
            return None
        return record


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
