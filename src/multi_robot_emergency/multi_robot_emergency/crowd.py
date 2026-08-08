"""Filtering for the crowd stage decided by the vision node.

[CODE REVIEW]
Vision이 정한 crowd level에 시간 필터만 적용한다. AMR이 person_count로 상태를
다시 판정하지 않으며, person_count는 진단/표시용으로만 저장한다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CrowdSnapshot:
    """Stable crowd state used for one candidate-ranking cycle."""

    level: int
    name: str
    person_count: int
    fresh: bool
    age_sec: float


class CrowdStateFilter:
    """Debounce vision-owned stage strings without counting people."""

    def __init__(
        self,
        *,
        level_names: list[str],
        state_timeout_sec: float,
        increase_confirm_sec: float,
        decrease_hold_sec: float,
    ) -> None:
        if len(level_names) < 2:
            raise ValueError(
                "crowd level_names must contain at least two stages"
            )
        normalized = [self._normalize(value) for value in level_names]
        if any(not value for value in normalized):
            raise ValueError("crowd stage names must not be empty")
        if len(set(normalized)) != len(normalized):
            raise ValueError("crowd stage names must be unique")
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in (
                state_timeout_sec,
                increase_confirm_sec,
                decrease_hold_sec,
            )
        ):
            raise ValueError("crowd durations must be finite and non-negative")
        if state_timeout_sec <= 0.0:
            raise ValueError("crowd state timeout must be positive")

        self.level_names = tuple(normalized)
        self.level_by_name = {
            name: level for level, name in enumerate(self.level_names)
        }
        self.state_timeout_sec = state_timeout_sec
        self.increase_confirm_sec = increase_confirm_sec
        self.decrease_hold_sec = decrease_hold_sec
        self.received_at: float | None = None
        self.stable_level = -1
        self.candidate_level: int | None = None
        self.candidate_since: float | None = None
        self.person_count = 0

    def update_level(self, raw_level: str, now: float) -> CrowdSnapshot:
        """Consume the final stage selected and published by vision."""
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        normalized = self._normalize(raw_level)
        desired = self.level_by_name.get(normalized)
        # The vision package publishes severity as the strings "0".."3".
        # Keep the AMR-owned stage names and speed table while accepting that
        # wire format. Vision's JSON time multiplier is intentionally unused.
        if desired is None and normalized.isdecimal():
            numeric_level = int(normalized)
            if 0 <= numeric_level < len(self.level_names):
                desired = numeric_level
        self.received_at = now
        if desired is None:
            self.stable_level = -1
            self.candidate_level = None
            self.candidate_since = None
            return self.snapshot(now)
        if desired == self.stable_level:
            self.candidate_level = None
            self.candidate_since = None
            return self.snapshot(now)

        # [CODE REVIEW] 위험 단계 상승은 confirm 시간 후 반영하고,
        # 단계 하강은 더 긴 hold 시간 후 반영해 카메라 한두 프레임의 튐을 억제한다.
        if self.stable_level < 0 and desired == 0:
            required = 0.0
        elif self.stable_level >= 0 and desired < self.stable_level:
            required = self.decrease_hold_sec
        else:
            required = self.increase_confirm_sec
        if self.candidate_level != desired:
            self.candidate_level = desired
            self.candidate_since = now
        if now - self.candidate_since >= required:
            self.stable_level = desired
            self.candidate_level = None
            self.candidate_since = None
        return self.snapshot(now)

    def update_person_count(self, person_count: int) -> None:
        """Store count for diagnostics only; it never decides crowd state."""
        if person_count < 0:
            raise ValueError("person_count must be non-negative")
        self.person_count = int(person_count)

    def snapshot(self, now: float) -> CrowdSnapshot:
        """Return UNKNOWN when the latest crowd-stage message is stale."""
        # [CODE REVIEW] timeout 동안 새 crowd level이 없으면 마지막 값을 계속 쓰지 않고
        # UNKNOWN/fresh=False로 바꿔 오래된 카메라 상태가 경로 판단에 남지 않게 한다.
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if self.received_at is None:
            return CrowdSnapshot(
                -1, "UNKNOWN", self.person_count, False, math.inf
            )
        age = max(0.0, now - self.received_at)
        if age > self.state_timeout_sec or self.stable_level < 0:
            return CrowdSnapshot(
                -1, "UNKNOWN", self.person_count, False, age
            )
        return CrowdSnapshot(
            self.stable_level,
            self.level_names[self.stable_level],
            self.person_count,
            True,
            age,
        )

    @staticmethod
    def _normalize(value: str) -> str:
        return str(value).strip().upper()
