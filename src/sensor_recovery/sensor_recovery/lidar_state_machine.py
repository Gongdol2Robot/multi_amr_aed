"""Pure LiDAR liveness state machine, independent of ROS runtime.

Kept free of rclpy so the transition logic can be unit tested without
spinning a node, mirroring mission_manager/role_assignment.py.

[CODE REVIEW]
ROS Topic 수신과 시간 기반 상태 판정을 분리한 순수 상태 머신이다.
STARTING -> ALIVE -> FAULT -> RECOVERING -> ALIVE 흐름을 ROS 없이 검증한다.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class LidarState(str, Enum):
    STARTING = "STARTING"
    ALIVE = "ALIVE"
    FAULT = "FAULT"
    RECOVERING = "RECOVERING"


@dataclass(frozen=True)
class LidarWatchdogConfig:
    scan_timeout_sec: float
    startup_grace_sec: float
    recovery_duration_sec: float


@dataclass
class LidarMonitor:
    """Tracks one robot's LiDAR liveness independently of any other robot."""

    config: LidarWatchdogConfig
    start_time: float
    state: LidarState = LidarState.STARTING
    last_scan_time: Optional[float] = None
    recovery_start_time: Optional[float] = None

    def on_scan_received(self, now: float) -> Optional[LidarState]:
        """Record a new scan and return the new state, or None if unchanged."""
        # [CODE REVIEW] scan 한 건이라도 수신되면 STARTING은 ALIVE가 되고,
        # FAULT 상태였다면 바로 정상으로 확정하지 않고 RECOVERING 구간을 거친다.
        previous = self.state
        self.last_scan_time = now
        if self.state in (LidarState.STARTING, LidarState.ALIVE):
            self.state = LidarState.ALIVE
        elif self.state == LidarState.FAULT:
            self.state = LidarState.RECOVERING
            self.recovery_start_time = now
        # RECOVERING: stays RECOVERING; on_tick evaluates the recovery window.
        return self.state if self.state != previous else None

    def on_tick(self, now: float) -> Optional[LidarState]:
        """Re-evaluate timeouts on a timer and return the new state, or None."""
        # [CODE REVIEW] 메시지 callback이 아니라 주기 timer에서 timeout을 판정한다.
        # 복구 중 scan이 다시 끊기면 FAULT로 돌아가고, 연속 수신 시간까지 채워야 ALIVE가 된다.
        previous = self.state
        if self.state == LidarState.STARTING:
            grace_elapsed = self._elapsed(self.start_time, now)
            if self.last_scan_time is None and grace_elapsed >= self.config.startup_grace_sec:
                self.state = LidarState.FAULT
        elif self.state == LidarState.ALIVE:
            if self._scan_age(now) >= self.config.scan_timeout_sec:
                self.state = LidarState.FAULT
                self.recovery_start_time = None
        elif self.state == LidarState.RECOVERING:
            if self._scan_age(now) >= self.config.scan_timeout_sec:
                # Reception stopped again before recovery was confirmed.
                self.state = LidarState.FAULT
                self.recovery_start_time = None
            elif self._elapsed(self.recovery_start_time, now) >= self.config.recovery_duration_sec:
                self.state = LidarState.ALIVE
                self.recovery_start_time = None
        # FAULT: no-op; only on_scan_received moves it to RECOVERING.
        return self.state if self.state != previous else None

    def _scan_age(self, now: float) -> float:
        if self.last_scan_time is None:
            return float("inf")
        return self._elapsed(self.last_scan_time, now)

    @staticmethod
    def _elapsed(start: Optional[float], now: float) -> float:
        """Elapsed time since start, clamped to 0 to tolerate clock jumps back."""
        if start is None:
            return float("inf")
        return max(0.0, now - start)
