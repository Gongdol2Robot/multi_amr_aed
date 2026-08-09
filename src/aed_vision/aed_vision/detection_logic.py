"""ROS와 YOLO에 의존하지 않는 검출 후처리 로직."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
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

    @property
    def bottom_center(self) -> tuple[float, float]:
        """사람이 바닥과 닿는 지점을 근사하는 bbox 하단 중심을 반환한다."""
        return ((self.x1 + self.x2) / 2.0, self.y2)

    @property
    def aspect_ratio(self) -> float:
        """bbox의 가로/세로 비율을 0으로 나누지 않고 반환한다."""
        width = max(0.0, self.x2 - self.x1)
        height = max(1.0, self.y2 - self.y1)
        return width / height


def is_fallen_bbox_candidate(box: Box, aspect_threshold: float) -> bool:
    """가로로 긴 mannequin bbox를 Pose 실패 시 낙상 후보로 유지한다."""
    if aspect_threshold <= 0.0:
        raise ValueError("aspect_threshold must be positive")
    return box.aspect_ratio >= aspect_threshold


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
        # deque(maxlen=window_size)라 창이 가득 차면 append 시 가장 오래된
        # 프레임이 자동으로 밀려난다 — 별도 pop 없이 슬라이딩 윈도우가 된다.
        self._history.append(bool(detected))
        return sum(self._history) >= self.required_hits

    @property
    def hit_count(self) -> int:
        """현재 시간 창 안에서 검출에 성공한 프레임 수를 반환한다."""
        return sum(self._history)


def update_presence_confirmation(
    confirmation: TemporalConfirmation, detected: bool
) -> bool:
    """현재 프레임 검출과 시간 창 조건을 모두 만족할 때만 true를 반환한다.

    과거 hit가 창에 남아 있더라도 현재 프레임에 사람이 없으면 즉시 false가
    된다. false도 confirmation에 넣어 오래된 hit가 정상적으로 밀려나게 한다.
    """
    detected_now = bool(detected)
    window_confirmed = confirmation.update(detected_now)
    return detected_now and window_confirmed


def intersection_over_union(first: Box, second: Box) -> float:
    """두 bbox가 겹치는 정도인 Intersection over Union을 계산한다.

    반환 범위는 0.0~1.0이다. COCO person bbox가 fallen_person bbox와 많이
    겹치면 동일 대상을 두 모델이 검출한 것으로 보고 인파 수에서 제외한다.
    """
    # 두 bbox의 겹치는 사각형 좌표: 안쪽 경계끼리 max/min을 취한다.
    left = max(first.x1, second.x1)
    top = max(first.y1, second.y1)
    right = min(first.x2, second.x2)
    bottom = min(first.y2, second.y2)
    # right < left 또는 bottom < top이면(안 겹침) 음수가 나오므로 0으로 clamp.
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
    return len(
        filter_nonfallen_people(
            people, fallen, frame_size, roi, overlap_threshold
        )
    )


def filter_nonfallen_people(
    people: Iterable[Box],
    fallen: Iterable[Box],
    frame_size: tuple[int, int],
    roi: Sequence[float],
    overlap_threshold: float,
) -> list[Box]:
    """ROI 안의 사람 중 쓰러진 대상과 겹치지 않는 사람만 반환한다.

    로봇 카메라에서는 반환된 사람을 구조 인력 후보로 사용한다. 별도 사람
    추적 모델 없이 COCO person 검출을 재사용하되, 환자를 구조 인력으로 잘못
    세지 않도록 fallen_person 상자와 겹치는 검출은 제외한다.
    """
    fallen_boxes = tuple(fallen)
    selected = []
    for person in people:
        if not point_inside_normalized_roi(person.center, frame_size, roi):
            continue
        if any(
            intersection_over_union(person, fallen_box) >= overlap_threshold
            for fallen_box in fallen_boxes
        ):
            continue
        selected.append(person)
    return selected


def filter_helpers_near_fallen(
    helpers: Iterable[Box],
    fallen: Iterable[Box],
    frame_size: tuple[int, int],
    max_distance_ratio: float,
) -> list[Box]:
    """환자와 같은 프레임에서 충분히 가까운 조력자만 반환한다.

    조력자의 bbox 하단 중심과 환자 bbox 중심 사이 픽셀 거리를 화면 대각선으로
    정규화한다. 깊이 토픽 없이 해상도 변화에도 같은 비율 기준을 유지한다.
    환자가 현재 프레임에 없으면 누구도 조력자로 확정하지 않는다.
    """
    width, height = frame_size
    if width <= 0 or height <= 0:
        return []
    if not 0.0 < max_distance_ratio <= 1.0:
        raise ValueError("max_distance_ratio must be in (0, 1]")
    fallen_boxes = tuple(fallen)
    if not fallen_boxes:
        return []
    maximum_pixels = math.hypot(width, height) * max_distance_ratio
    selected = []
    for helper in helpers:
        helper_x, helper_y = helper.bottom_center
        if any(
            math.hypot(
                helper_x - fallen_box.center[0],
                helper_y - fallen_box.center[1],
            )
            <= maximum_pixels
            for fallen_box in fallen_boxes
        ):
            selected.append(helper)
    return selected


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


def apply_crowd_time_penalty(
    base_time: float, person_count: int
) -> float | None:
    """기본 이동 시간에 혼잡 패널티를 적용한다."""
    if base_time < 0.0:
        raise ValueError("base_time must be non-negative")
    multiplier = crowd_time_multiplier(person_count)
    return None if multiplier is None else base_time * multiplier
