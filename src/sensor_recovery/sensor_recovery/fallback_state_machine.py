"""Fallback path-follower state machine.

    IDLE -> STARTING -> ACTIVE <-> BLOCKED -> SUCCEEDED
    (STARTING/ACTIVE) -> BLOCKED when odom or depth is temporarily unavailable
    (STARTING/ACTIVE/BLOCKED) -> FAILED on a terminal safety condition

Kept ROS-free like lidar_state_machine.py — the node computes the input
booleans/durations each tick (odom stale, depth blocked, stuck, path
deviation, arrival) and this module only decides the resulting state.

[CODE REVIEW]
센서/제어 계산 결과를 입력 snapshot으로 받아 fallback 상태만 결정하는 순수 함수다.
일시적인 정지는 BLOCKED, 복구 불가능한 안전 조건은 FAILED로 구분한다.
"""

from dataclasses import dataclass
from enum import Enum


class FallbackState(str, Enum):
    IDLE = "IDLE"
    STARTING = "STARTING"
    ACTIVE = "ACTIVE"
    BLOCKED = "BLOCKED"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class FallbackTickInputs:
    """Snapshot of everything the state machine needs to decide one tick."""

    has_plan: bool
    has_anchor: bool
    odom_stale: bool
    depth_blocked: bool
    blocked_duration_sec: float
    blocked_timeout_sec: float
    stuck: bool
    path_deviation_m: float
    max_path_deviation_m: float
    arrived: bool


def next_fallback_state(current: FallbackState, inputs: FallbackTickInputs) -> FallbackState:
    """Pure state transition for one control tick. Terminal states don't change."""
    # [CODE REVIEW] SUCCEEDED/FAILED는 terminal 상태라 이후 센서 값이 변해도 되돌리지 않는다.
    if current in (FallbackState.SUCCEEDED, FallbackState.FAILED):
        return current

    # plan/anchor 손실, stuck, 과도한 경로 이탈, depth 장시간 차단은
    # 스스로 안전하게 계속 갈 수 없는 조건이므로 FAILED 후 대체 로봇을 요청한다.
    if not inputs.has_plan or not inputs.has_anchor:
        return FallbackState.FAILED
    # A delayed odom packet is a recoverable communication interruption, not
    # evidence that the route itself has failed.  BLOCKED makes the caller
    # publish zero velocity until fresh odom arrives, then ACTIVE resumes.
    # odom 지연과 짧은 depth 차단은 일시적인 통신/장애물 상황일 수 있으므로
    # 0속도 BLOCKED로 기다렸다가 입력이 정상화되면 ACTIVE로 복귀한다.
    if inputs.odom_stale:
        return FallbackState.BLOCKED
    if inputs.stuck:
        return FallbackState.FAILED
    if inputs.path_deviation_m > inputs.max_path_deviation_m:
        return FallbackState.FAILED
    if inputs.depth_blocked and inputs.blocked_duration_sec > inputs.blocked_timeout_sec:
        return FallbackState.FAILED

    if inputs.arrived:
        return FallbackState.SUCCEEDED
    if inputs.depth_blocked:
        return FallbackState.BLOCKED
    return FallbackState.ACTIVE
