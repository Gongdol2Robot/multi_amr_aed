"""ROS와 YOLO에 의존하지 않는 검출 후처리 로직."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Box:
    """모델 종류와 무관하게 후처리에서 사용하는 검출 bbox.

    Ultralytics 전용 객체를 이 단순 자료형으로 바꾸면 ROI와 IoU 로직을
    ROS, PyTorch, GPU 없이 단위 테스트할 수 있다. 좌표는 원본 영상의 픽셀
    기준이며 왼쪽 위가 (x1, y1), 오른쪽 아래가 (x2, y2)이다.
    """

    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float = 0.0

    @property
    def center(self) -> tuple[float, float]:
        """bbox의 중심 픽셀 좌표를 반환한다.

        혼잡도 ROI 포함 여부는 bbox 일부가 아니라 중심점 기준으로 판단한다.
        경계에 걸친 사람을 매 프레임 다르게 세는 현상을 줄이기 위한 기준이다.
        """
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)


class TemporalConfirmation:
    """최근 프레임의 다수결로 순간적인 오검출을 응급상황과 분리한다.

    예를 들어 window_size=10, required_hits=6이면 연속일 필요 없이 최근
    10프레임 중 6프레임에서 검출되어야 확정된다. 단순 연속 검출보다 한두
    프레임의 가림이나 confidence 하락에 덜 민감하다.
    """

    def __init__(self, window_size: int, required_hits: int) -> None:
        """확인 창 크기와 확정에 필요한 최소 검출 횟수를 설정한다.

        Raises:
            ValueError: 창 크기가 1보다 작거나 required_hits가 범위를 벗어난 경우.
        """
        if window_size < 1:
            raise ValueError("window_size must be at least 1")
        if not 1 <= required_hits <= window_size:
            raise ValueError("required_hits must be between 1 and window_size")
        self._history: deque[bool] = deque(maxlen=window_size)
        self.required_hits = required_hits

    def update(self, detected: bool) -> bool:
        """현재 프레임 결과를 창에 추가하고 응급상황 확정 여부를 반환한다."""
        self._history.append(bool(detected))
        return sum(self._history) >= self.required_hits

    @property
    def hit_count(self) -> int:
        """현재 시간 창 안에서 검출에 성공한 프레임 수를 반환한다."""
        return sum(self._history)


def intersection_over_union(first: Box, second: Box) -> float:
    """두 bbox가 겹치는 정도인 Intersection over Union을 계산한다.

    반환 범위는 0.0~1.0이다. COCO person bbox가 fallen_person bbox와 많이
    겹치면 동일 대상을 두 모델이 검출한 것으로 보고 인파 수에서 제외한다.
    """
    left = max(first.x1, second.x1)
    top = max(first.y1, second.y1)
    right = min(first.x2, second.x2)
    bottom = min(first.y2, second.y2)
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first.x2 - first.x1) * max(0.0, first.y2 - first.y1)
    second_area = max(0.0, second.x2 - second.x1) * max(
        0.0, second.y2 - second.y1
    )
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def point_inside_normalized_roi(
    point: tuple[float, float],
    frame_size: tuple[int, int],
    roi: Sequence[float],
) -> bool:
    """픽셀 점이 정규화된 [left, top, right, bottom] ROI 안인지 확인한다.

    ROI를 0.0~1.0 비율로 저장하므로 카메라 해상도가 바뀌어도 같은 영역
    비율을 유지할 수 있다. frame_size 순서는 (width, height)이다.
    """
    width, height = frame_size
    if len(roi) != 4 or width <= 0 or height <= 0:
        return False
    x, y = point
    return (
        roi[0] * width <= x <= roi[2] * width
        and roi[1] * height <= y <= roi[3] * height
    )


def count_crowd_people(
    people: Iterable[Box],
    fallen: Iterable[Box],
    frame_size: tuple[int, int],
    roi: Sequence[float],
    overlap_threshold: float,
) -> int:
    """AMR 통로 ROI 안에서 실제 혼잡도에 포함할 사람 수를 계산한다.

    처리 순서:
    1. bbox 중심이 ROI 밖인 person을 제외한다.
    2. fallen bbox와 IoU가 임계값 이상인 person을 동일 대상으로 보고 제외한다.
    3. 남은 COCO person만 통행을 방해할 수 있는 인파로 센다.
    """
    fallen_boxes = tuple(fallen)
    count = 0
    for person in people:
        if not point_inside_normalized_roi(person.center, frame_size, roi):
            continue
        if any(
            intersection_over_union(person, fallen_box) >= overlap_threshold
            for fallen_box in fallen_boxes
        ):
            continue
        count += 1
    return count


def classify_crowd(person_count: int) -> int:
    """ROI 내 사람 수를 0~3 혼잡 등급으로 변환한다.

    3등급은 사람이 정확히 3명인 경우뿐 아니라 3명 이상인 모든 경우를
    포함한다. open 카메라의 NOT_APPLICABLE 처리는 추론 파이프라인에서 한다.
    """
    if person_count < 0:
        raise ValueError("person_count must be non-negative")
    return min(person_count, 3)


def crowd_time_multiplier(person_count: int) -> float | None:
    """사람 수에 따른 이동 시간 배율을 반환한다.

    0명은 패널티 없음, 1명은 10%, 2명은 20%를 가산한다. 3명 이상은
    통행 불가이므로 계산 가능한 이동 시간이 없음을 뜻하는 None을 반환한다.
    """
    level = classify_crowd(person_count)
    if level == 3:
        return None
    return 1.0 + level * 0.1


def apply_crowd_time_penalty(base_time: float, person_count: int) -> float | None:
    """기본 이동 시간에 혼잡 패널티를 적용한다."""
    if base_time < 0.0:
        raise ValueError("base_time must be non-negative")
    multiplier = crowd_time_multiplier(person_count)
    return None if multiplier is None else base_time * multiplier
