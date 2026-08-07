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
