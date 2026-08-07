"""ROS 없이 시험할 수 있는 현장 구조 인력 탐색 판정 함수."""

from math import isfinite


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
    """Vision 신호가 지정 시간 동안 없었는지 안전 정지용으로 판정한다.

    아직 한 번도 수신하지 못했다면 탐색 시작 시각을, 한 번이라도 수신했다면
    마지막 수신 시각을 기준으로 한다. false 메시지도 카메라 생존 신호이므로
    last_observed_at을 갱신하면 타임아웃을 막는다.
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
