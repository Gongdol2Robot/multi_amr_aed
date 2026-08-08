"""Fallback path-follower state machine.

    IDLE -> STARTING -> ACTIVE <-> BLOCKED -> SUCCEEDED
    (STARTING/ACTIVE) -> BLOCKED when odom or depth is temporarily unavailable
    (STARTING/ACTIVE/BLOCKED) -> FAILED on a terminal safety condition

Kept ROS-free like lidar_state_machine.py — the node computes the input
booleans/durations each tick (odom stale, depth blocked, stuck, path
deviation, arrival) and this module only decides the resulting state.
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
    if current in (FallbackState.SUCCEEDED, FallbackState.FAILED):
        return current

    if not inputs.has_plan or not inputs.has_anchor:
        return FallbackState.FAILED
    # A delayed odom packet is a recoverable communication interruption, not
    # evidence that the route itself has failed.  BLOCKED makes the caller
    # publish zero velocity until fresh odom arrives, then ACTIVE resumes.
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
