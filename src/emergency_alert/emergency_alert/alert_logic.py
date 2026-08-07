"""ROS에 의존하지 않는 경보 패턴 및 MissionStatus 전이 정책."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Iterable, Optional


class AlertCommand(Enum):
    """경보 출력 노드가 수행할 수 있는 단일 명령."""

    IGNORE = "ignore"
    STOP = "stop"
    START_TRAVEL = "start_travel"
    PLAY_ARRIVAL = "play_arrival"
    PLAY_INTERRUPTED = "play_interrupted"


class MissionPhase(Enum):
    """MissionStatus 숫자를 경보 관점의 단계로 단순화한 값."""

    ASSIGNED = "assigned"
    DISPATCHING = "dispatching"
    EN_ROUTE = "en_route"
    ARRIVED = "arrived"
    COMPLETED = "completed"
    INTERRUPTED = "interrupted"
    UNRELATED = "unrelated"


_PROGRESS_RANK = {
    MissionPhase.ASSIGNED: 1,
    MissionPhase.DISPATCHING: 2,
    MissionPhase.EN_ROUTE: 3,
}


@dataclass(frozen=True)
class TonePattern:
    """한 번에 재생할 주파수 배열과 각 음의 최대 재생시간."""

    frequencies: tuple[int, ...]
    note_duration: float

    def __post_init__(self) -> None:
        """빈 패턴, 비양수 주파수 및 비양수 재생시간을 거부한다."""
        if not self.frequencies:
            raise ValueError("tone pattern must contain at least one frequency")
        if any(frequency <= 0 for frequency in self.frequencies):
            raise ValueError("tone frequencies must be positive")
        if not isfinite(self.note_duration) or self.note_duration <= 0.0:
            raise ValueError("note_duration must be positive")

    @classmethod
    def from_values(
        cls, frequencies: Iterable[int], note_duration: float
    ) -> "TonePattern":
        """ROS 파라미터 등의 반복 가능한 값을 검증된 불변 패턴으로 바꾼다."""
        return cls(tuple(int(value) for value in frequencies), note_duration)

    @property
    def total_duration(self) -> float:
        """전체 음계가 한 번 재생되는 데 필요한 최대 시간을 반환한다."""
        return len(self.frequencies) * self.note_duration


class MissionAlertPolicy:
    """로봇별 MissionStatus를 중복·역행 없이 경보 명령으로 변환한다."""

    def __init__(self, robot_id: str) -> None:
        """대상 로봇과 이벤트별 최신 version 및 활성 임무 상태를 초기화한다."""
        if not robot_id:
            raise ValueError("robot_id is required")
        self.robot_id = robot_id
        self.latest_versions = {}
        self.active_key: Optional[tuple[str, int]] = None
        self.last_phase_by_key = {}
        self.terminal_keys = set()
        self.travel_alarm_active = False

    def handle(
        self,
        *,
        robot_id: str,
        event_id: str,
        assignment_version: int,
        phase: MissionPhase,
    ) -> AlertCommand:
        """새 상태의 소유자·version·중복 여부를 검사해 필요한 명령만 반환한다."""
        if robot_id != self.robot_id or not event_id:
            return AlertCommand.IGNORE
        if phase is MissionPhase.UNRELATED:
            return AlertCommand.IGNORE

        version = int(assignment_version)
        latest = self.latest_versions.get(event_id, -1)
        if version < latest:
            return AlertCommand.IGNORE
        is_new_version = version > latest
        if version > latest:
            self.latest_versions[event_id] = version

        key = (event_id, version)
        if key in self.terminal_keys:
            return AlertCommand.IGNORE
        if self.last_phase_by_key.get(key) is phase:
            return AlertCommand.IGNORE
        previous_phase = self.last_phase_by_key.get(key)
        if (
            previous_phase in _PROGRESS_RANK
            and phase in _PROGRESS_RANK
            and _PROGRESS_RANK[phase] < _PROGRESS_RANK[previous_phase]
        ):
            return AlertCommand.IGNORE
        self.last_phase_by_key[key] = phase

        if phase in (MissionPhase.ASSIGNED, MissionPhase.DISPATCHING):
            should_stop = self.travel_alarm_active
            self.active_key = key
            self.travel_alarm_active = False
            return AlertCommand.STOP if should_stop else AlertCommand.IGNORE

        if phase is MissionPhase.EN_ROUTE:
            if self.active_key not in (None, key) and not is_new_version:
                return AlertCommand.IGNORE
            self.active_key = key
            self.travel_alarm_active = True
            return AlertCommand.START_TRAVEL

        if phase in (MissionPhase.ARRIVED, MissionPhase.COMPLETED):
            if self.active_key not in (None, key):
                return AlertCommand.IGNORE
            self.terminal_keys.add(key)
            self.active_key = None
            self.travel_alarm_active = False
            return AlertCommand.PLAY_ARRIVAL

        if phase is MissionPhase.INTERRUPTED:
            if self.active_key not in (None, key):
                return AlertCommand.IGNORE
            self.terminal_keys.add(key)
            self.active_key = None
            self.travel_alarm_active = False
            return AlertCommand.PLAY_INTERRUPTED

        return AlertCommand.IGNORE

    def mark_output_stopped(self) -> None:
        """외부 안전 타임아웃 등으로 실제 반복 출력이 멈췄음을 정책에 반영한다."""
        self.travel_alarm_active = False
