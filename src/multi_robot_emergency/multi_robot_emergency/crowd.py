"""Filtering for the crowd stage decided by the vision node."""

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
        desired = self.level_by_name.get(self._normalize(raw_level))
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
