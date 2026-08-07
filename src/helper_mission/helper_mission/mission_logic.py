"""ROS 없이 시험할 수 있는 현장 구조 인력 탐색 판정 함수."""

from math import isfinite


def arrival_dispatch_allowed(
    event_id: str,
    *,
    canceled_events,
    handled_events,
    pending_events,
    active_events,
) -> bool:
    """ARRIVED가 새 탐색 문맥을 만들어도 되는지 중복·취소 상태로 판정한다."""
    return bool(event_id) and not (
        event_id in canceled_events
        or event_id in handled_events
        or event_id in pending_events
        or event_id in active_events
    )


def dispatch_response_is_current(
    *,
    event_exists: bool,
    canceled: bool,
    context_serial: int | None,
    response_serial: int,
    dispatching: bool,
) -> bool:
    """비동기 Action 수락 응답이 아직 현재 요청에 해당하는지 판정한다."""
    return (
        event_exists
        and not canceled
        and dispatching
        and context_serial == response_serial
    )


def helper_confirmation_is_fresh(
    *,
    confirmed: bool,
    observed_at: float | None,
    now: float,
    stale_seconds: float,
) -> bool:
    """Vision의 true 관측이 유효 시간 안에 들어왔는지 반환한다.

    오래된 true 값을 계속 믿으면 카메라가 끊긴 뒤에도 회전을 멈출 수 있으므로
    수신 시각과 현재 시각이 유한하고 관측 나이가 제한 이내일 때만 인정한다.
    """
    if stale_seconds <= 0.0:
        raise ValueError("stale_seconds must be positive")
    if not confirmed or observed_at is None:
        return False
    if not isfinite(observed_at) or not isfinite(now):
        return False
    age = now - observed_at
    return 0.0 <= age <= stale_seconds


def vision_stream_timed_out(
    *,
    search_started_at: float,
    last_observed_at: float | None,
    now: float,
    timeout_seconds: float,
) -> bool:
    """Vision 메시지가 지정 시간 동안 끊겼는지 안전 정지용으로 판정한다.

    아직 메시지를 받지 못했다면 탐색 시작 시각을 기준으로 하고, 한 번이라도
    받았다면 검출값이 true인지 false인지와 무관하게 마지막 수신 시각을 쓴다.
    따라서 false 메시지가 계속 오면 카메라는 정상 동작 중인 것으로 처리한다.
    """
    if timeout_seconds <= 0.0:
        raise ValueError("timeout_seconds must be positive")
    if not isfinite(search_started_at) or not isfinite(now):
        return True
    reference = (
        search_started_at
        if last_observed_at is None
        else last_observed_at
    )
    if not isfinite(reference):
        return True
    age = now - reference
    return age < 0.0 or age >= timeout_seconds
