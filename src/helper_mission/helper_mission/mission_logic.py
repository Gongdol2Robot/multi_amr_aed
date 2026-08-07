"""ROS-independent helper mission decision helpers."""

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, Optional


@dataclass(frozen=True)
class Candidate:
    """The fields needed to choose a robot for a helper mission."""

    robot_id: str
    available: bool
    network_ok: bool
    localization_ok: bool
    nav2_ok: bool
    emergency_stop: bool
    path_valid: bool
    battery_percentage: float
    path_cost: float


def select_helper_robot(
    candidates: Iterable[Candidate], excluded_robot_id: str
) -> Optional[str]:
    """
    Select one healthy reserve robot deterministically.

    A non-negative path cost wins first. When no planner cost is available,
    battery and robot ID provide a stable fallback order.
    """
    eligible = [
        candidate
        for candidate in candidates
        if candidate.robot_id != excluded_robot_id
        and candidate.available
        and candidate.network_ok
        and candidate.localization_ok
        and candidate.nav2_ok
        and not candidate.emergency_stop
        and candidate.path_valid
        and candidate.battery_percentage >= 20.0
    ]
    if not eligible:
        return None

    def rank(candidate: Candidate):
        """경로비용·배터리·로봇 ID 순으로 비교 가능한 정렬 키를 만든다."""
        has_cost = isfinite(candidate.path_cost) and candidate.path_cost >= 0.0
        return (
            0 if has_cost else 1,
            candidate.path_cost if has_cost else 0.0,
            -candidate.battery_percentage,
            candidate.robot_id,
        )

    return min(eligible, key=rank).robot_id


class PresenceGate:
    """Require fresh, repeated helper observations for a hold duration."""

    def __init__(
        self,
        minimum_evidence: int,
        maximum_distance: float,
        hold_seconds: float,
        stale_seconds: float,
    ) -> None:
        """검출 근거·거리·연속 시간·데이터 만료 기준을 저장한다."""
        self.minimum_evidence = minimum_evidence
        self.maximum_distance = maximum_distance
        self.hold_seconds = hold_seconds
        self.stale_seconds = stale_seconds
        self._qualifying_since = None
        self._last_observed_at = None
        self._distance = float("inf")

    @property
    def distance(self) -> float:
        """가장 최근에 관측한 로봇과 구조 인력 사이 거리를 반환한다."""
        return self._distance

    def observe(
        self,
        *,
        helper_count: int,
        evidence_count: int,
        distance: float,
        observed_at: float,
    ) -> None:
        """새 검출값이 기준을 만족하는지 평가하고 연속 관측 시작점을 갱신한다."""
        qualifies = (
            helper_count > 0
            and evidence_count >= self.minimum_evidence
            and isfinite(distance)
            and 0.0 <= distance <= self.maximum_distance
        )
        self._last_observed_at = observed_at
        self._distance = distance
        if qualifies:
            if self._qualifying_since is None:
                self._qualifying_since = observed_at
        else:
            self._qualifying_since = None

    def confirmed(self, now: float) -> bool:
        """최신 검출이 만료되지 않고 요구 시간 동안 연속됐는지 반환한다."""
        if self._last_observed_at is None or self._qualifying_since is None:
            return False
        if now - self._last_observed_at > self.stale_seconds:
            self._qualifying_since = None
            return False
        return now - self._qualifying_since >= self.hold_seconds
