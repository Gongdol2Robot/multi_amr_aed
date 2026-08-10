"""모델 검출값을 실제 판정값으로 바꾸는 순수 후처리 로직.

이 파일은 ROS, OpenCV, YOLO에 의존하지 않는다. ``InferencePipeline``이 모델의
출력을 :class:`Box`로 바꾼 뒤 이 함수들을 호출한다. 따라서 여기서는 다음만
판단한다: 박스 중복(IoU), ROI 포함 여부, 조력자 거리, 최근 프레임 확정,
골목 인원수에 따른 혼잡 등급.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from time import monotonic
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
        # 중요: 연속 검출 횟수가 아니다. 최근 N프레임 안의 True 개수가
        # required_hits 이상인지 보는 다수결이므로 잠깐의 가림을 허용한다.
        # deque(maxlen=window_size)라 창이 가득 차면 append 시 가장 오래된
        # 프레임이 자동으로 밀려난다 — 별도 pop 없이 슬라이딩 윈도우가 된다.
        self._history.append(bool(detected))
        return sum(self._history) >= self.required_hits

    @property
    def hit_count(self) -> int:
        """현재 시간 창 안에서 검출에 성공한 프레임 수를 반환한다."""
        return sum(self._history)

    def clear(self) -> None:
        """새 상태 주기를 시작할 수 있도록 과거 프레임 근거를 비운다."""
        self._history.clear()


class FallenStateConfirmation:
    """낙상 확정은 프레임 창으로, 해제는 실제 미검출 시간으로 판단한다.

    처리 FPS가 낮아져도 확정된 사고가 같은 실제 시간 동안 유지되도록 해제만
    monotonic 시간 기준으로 분리한다. 확정 전에는 기존 TemporalConfirmation의
    다중 프레임 기준을 그대로 사용한다.
    """

    def __init__(
        self, window_size: int, required_hits: int, clear_after_seconds: float
    ) -> None:
        if clear_after_seconds <= 0.0:
            raise ValueError("clear_after_seconds must be positive")
        self._confirmation = TemporalConfirmation(window_size, required_hits)
        self.clear_after_seconds = float(clear_after_seconds)
        self._confirmed = False
        self._missing_since: float | None = None

    def update(self, detected: bool, now: float) -> bool:
        detected_now = bool(detected)
        window_confirmed = self._confirmation.update(detected_now)
        if not self._confirmed:
            if window_confirmed:
                self._confirmed = True
                self._missing_since = None
            return self._confirmed

        if detected_now:
            self._missing_since = None
        elif self._missing_since is None:
            self._missing_since = float(now)
        elif float(now) - self._missing_since >= self.clear_after_seconds:
            self._confirmed = False
            self._missing_since = None
            self._confirmation.clear()
        return self._confirmed

    @property
    def hit_count(self) -> int:
        return self._confirmation.hit_count


class StationaryFallConfirmation:
    """동일 낙상 bbox가 일정 시간 정지해 있을 때만 사고를 확정한다.

    bbox IoU로 같은 대상을 추적하고, 연속 관측 사이의 중심 이동과 면적 변화를
    영상 크기로 정규화해 정지 여부를 판단한다. 확정 뒤에는 bbox 움직임으로
    취소하지 않고 기존 안전 정책처럼 일정 시간 완전 미검출일 때만 해제한다.
    """

    def __init__(
        self,
        stationary_seconds: float,
        max_center_motion_ratio: float,
        max_size_change_ratio: float,
        match_iou: float,
        gap_tolerance_seconds: float,
        clear_after_seconds: float,
    ) -> None:
        values = (
            stationary_seconds,
            max_center_motion_ratio,
            max_size_change_ratio,
            match_iou,
            gap_tolerance_seconds,
            clear_after_seconds,
        )
        if any(value <= 0.0 for value in values):
            raise ValueError("stationary confirmation values must be positive")
        if max_center_motion_ratio > 1.0:
            raise ValueError("max_center_motion_ratio must not exceed 1")
        if max_size_change_ratio > 1.0:
            raise ValueError("max_size_change_ratio must not exceed 1")
        if match_iou > 1.0:
            raise ValueError("match_iou must not exceed 1")
        self.stationary_seconds = float(stationary_seconds)
        self.max_center_motion_ratio = float(max_center_motion_ratio)
        self.max_size_change_ratio = float(max_size_change_ratio)
        self.match_iou = float(match_iou)
        self.gap_tolerance_seconds = float(gap_tolerance_seconds)
        self.clear_after_seconds = float(clear_after_seconds)
        self._tracked_box: Box | None = None
        self._anchor_box: Box | None = None
        self._stationary_since: float | None = None
        self._last_seen_at: float | None = None
        self._missing_since: float | None = None
        self._confirmed = False
        self._hit_count = 0
        self.stationary_duration = 0.0
        self.center_motion_ratio = 0.0
        self.size_change_ratio = 0.0

    def update(
        self,
        boxes: Sequence[Box],
        frame_size: tuple[int, int],
        now: float,
    ) -> bool:
        width, height = frame_size
        if width <= 0 or height <= 0:
            raise ValueError("frame_size must be positive")
        current_time = float(now)
        candidates = tuple(boxes)
        if not candidates:
            self._update_missing(current_time)
            return self._confirmed

        self._missing_since = None
        selected = self._select_candidate(candidates)
        if self._tracked_box is None or not self._matches(selected):
            self._start_track(selected, current_time)
            return self._confirmed

        anchor = self._anchor_box or self._tracked_box
        diagonal = math.hypot(width, height)
        self.center_motion_ratio = math.hypot(
            selected.center[0] - anchor.center[0],
            selected.center[1] - anchor.center[1],
        ) / diagonal
        previous_area = self._area(anchor)
        current_area = self._area(selected)
        self.size_change_ratio = abs(current_area - previous_area) / max(
            previous_area, 1.0
        )
        gap = (
            current_time - self._last_seen_at
            if self._last_seen_at is not None else 0.0
        )
        stationary = (
            gap <= self.gap_tolerance_seconds
            and self.center_motion_ratio <= self.max_center_motion_ratio
            and self.size_change_ratio <= self.max_size_change_ratio
        )
        self._tracked_box = selected
        self._last_seen_at = current_time
        if not stationary:
            self._anchor_box = selected
            self._stationary_since = current_time
            self._hit_count = 1
            self.stationary_duration = 0.0
            return self._confirmed

        self._hit_count += 1
        if self._stationary_since is None:
            self._stationary_since = current_time
        self.stationary_duration = current_time - self._stationary_since
        if self.stationary_duration >= self.stationary_seconds:
            self._confirmed = True
        return self._confirmed

    @staticmethod
    def _area(box: Box) -> float:
        return max(0.0, box.x2 - box.x1) * max(0.0, box.y2 - box.y1)

    def _select_candidate(self, candidates: Sequence[Box]) -> Box:
        if self._tracked_box is None:
            return max(candidates, key=lambda box: box.confidence)
        nearest = max(
            candidates,
            key=lambda box: intersection_over_union(box, self._tracked_box),
        )
        if self._matches(nearest):
            return nearest
        return max(candidates, key=lambda box: box.confidence)

    def _matches(self, candidate: Box) -> bool:
        return intersection_over_union(candidate, self._tracked_box) >= (
            self.match_iou
        )

    def _start_track(self, box: Box, now: float) -> None:
        self._tracked_box = box
        self._anchor_box = box
        self._stationary_since = now
        self._last_seen_at = now
        self._hit_count = 1
        self.stationary_duration = 0.0
        self.center_motion_ratio = 0.0
        self.size_change_ratio = 0.0

    def _update_missing(self, now: float) -> None:
        if self._missing_since is None:
            self._missing_since = now
        if self._confirmed:
            if now - self._missing_since >= self.clear_after_seconds:
                self._confirmed = False
                self.clear()
        elif (
            self._last_seen_at is None
            or now - self._last_seen_at > self.gap_tolerance_seconds
        ):
            self.clear()

    def clear(self) -> None:
        self._tracked_box = None
        self._anchor_box = None
        self._stationary_since = None
        self._last_seen_at = None
        self._missing_since = None
        self._hit_count = 0
        self.stationary_duration = 0.0
        self.center_motion_ratio = 0.0
        self.size_change_ratio = 0.0

    @property
    def hit_count(self) -> int:
        return self._hit_count


class CrowdStateStabilizer:
    """순간적인 사람 검출 변화가 로봇의 통행 판단을 흔들지 않게 한다.

    혼잡도가 나빠지는 변화는 짧은 창에서 빠르게 확정하고, 좋아지는 변화는
    더 긴 창과 최소 유지 시간을 모두 통과해야 반영한다. 예를 들어 기본값은
    최근 5회 중 3회 BLOCKED이면 차단하지만, 해제는 최소 3초가 지난 뒤 최근
    10회 중 7회가 더 낮은 등급이어야 한다.

    ``person_count`` 자체를 평균내지는 않는다. 현재 관측 인원은 그대로 공개하고
    로봇 제어에 쓰이는 level/time_multiplier/traversable만 안정화하기 위함이다.
    """

    def __init__(
        self,
        worsening_window: int,
        worsening_hits: int,
        improving_window: int,
        improving_hits: int,
        minimum_hold_seconds: float,
    ) -> None:
        if not 1 <= worsening_hits <= worsening_window:
            raise ValueError("worsening_hits must be between 1 and window")
        if not 1 <= improving_hits <= improving_window:
            raise ValueError("improving_hits must be between 1 and window")
        if minimum_hold_seconds < 0.0:
            raise ValueError("minimum_hold_seconds must be non-negative")
        self.worsening_window = worsening_window
        self.worsening_hits = worsening_hits
        self.improving_window = improving_window
        self.improving_hits = improving_hits
        self.minimum_hold_seconds = minimum_hold_seconds
        self._history: deque[int] = deque(maxlen=max(
            worsening_window, improving_window
        ))
        self._level: int | None = None
        self._changed_at = 0.0

    @property
    def level(self) -> int | None:
        """현재 로봇에 공개할 확정 혼잡 등급을 반환한다."""
        return self._level

    def update(self, observed_level: int, now: float | None = None) -> int:
        """새 관측 등급을 기록하고 안정화된 등급을 반환한다."""
        if not 0 <= observed_level <= 3:
            raise ValueError("observed_level must be between 0 and 3")
        current_time = monotonic() if now is None else float(now)
        self._history.append(observed_level)
        if self._level is None:
            # 시작할 때까지 긴 창을 기다리면 실제로 막힌 길을 CLEAR로 오해할 수
            # 있으므로 첫 관측값으로 상태를 즉시 초기화한다.
            self._set_level(observed_level, current_time)
            return observed_level

        if observed_level > self._level:
            recent = list(self._history)[-self.worsening_window:]
            # 현재보다 높은 등급 중 가장 심각하면서 필요한 횟수를 만족한 상태로
            # 올린다. level>=candidate로 세어 BLOCKED 관측도 CROWDED 근거가 된다.
            for candidate in range(3, self._level, -1):
                if sum(level >= candidate for level in recent) >= (
                    self.worsening_hits
                ):
                    self._set_level(candidate, current_time)
                    break
        elif (
            observed_level < self._level
            and current_time - self._changed_at >= self.minimum_hold_seconds
        ):
            recent = list(self._history)[-self.improving_window:]
            # 낮은 상태일수록 더 엄격한 조건(level<=candidate)을 만족해야 한다.
            # CLEAR 증거가 충분하면 중간 단계를 거치지 않고 바로 CLEAR로 내린다.
            for candidate in range(0, self._level):
                if sum(level <= candidate for level in recent) >= (
                    self.improving_hits
                ):
                    self._set_level(candidate, current_time)
                    break
        return self._level

    def _set_level(self, level: int, changed_at: float) -> None:
        """확정 상태와 최소 유지시간의 시작 시각을 함께 갱신한다."""
        self._level = level
        self._changed_at = changed_at


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


def intersection_over_smaller_area(first: Box, second: Box) -> float:
    """겹친 면적이 두 bbox 중 작은 bbox를 얼마나 덮는지 반환한다."""
    left = max(first.x1, second.x1)
    top = max(first.y1, second.y1)
    right = min(first.x2, second.x2)
    bottom = min(first.y2, second.y2)
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first.x2 - first.x1) * max(0.0, first.y2 - first.y1)
    second_area = max(0.0, second.x2 - second.x1) * max(
        0.0, second.y2 - second.y1
    )
    smaller = min(first_area, second_area)
    return intersection / smaller if smaller > 0.0 else 0.0


def boxes_represent_same_person(
    person: Box, fallen: Box, overlap_threshold: float
) -> bool:
    """서로 다른 모델의 bbox 크기 차이를 허용해 동일 환자인지 판정한다."""
    if intersection_over_union(person, fallen) >= overlap_threshold:
        return True
    # 한 모델이 상자를 훨씬 크게 잡아 IoU가 낮아도 작은 상자의 대부분이
    # 겹치면 같은 대상으로 본다. 0.7은 인접 조력자 제거를 피하는 보수값이다.
    if intersection_over_smaller_area(person, fallen) >= 0.7:
        return True
    person_center = person.center
    fallen_center = fallen.center
    return (
        person.x1 <= fallen_center[0] <= person.x2
        and person.y1 <= fallen_center[1] <= person.y2
        and fallen.x1 <= person_center[0] <= fallen.x2
        and fallen.y1 <= person_center[1] <= fallen.y2
    )


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
    # fallen은 generator일 수도 있으므로 반복문 안에서 여러 번 비교하기 전에
    # tuple로 고정한다. selected는 최종적으로 '환자가 아닌 주변 사람' 목록이다.
    fallen_boxes = tuple(fallen)
    selected: list[Box] = []
    for person in people:
        if not point_inside_normalized_roi(person.center, frame_size, roi):
            continue
        if any(
            boxes_represent_same_person(
                person, fallen_box, overlap_threshold
            )
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


def crowd_metrics(person_count: int) -> tuple[int, float | None, bool]:
    """ROI 인원수에서 혼잡 등급, 이동 시간 배율과 통행 가능 여부를 계산한다.

    0~2명은 등급별로 1.0/1.1/1.2 배의 이동 시간을 사용한다. 3명 이상은
    BLOCKED(3)로 묶고 이동 시간 계산이 불가능하므로 ``None, False``를 반환한다.
    """
    if person_count < 0:
        raise ValueError("person_count must be non-negative")
    # CrowdLevel 메시지 상수와 숫자를 맞춘다: 0 CLEAR, 1 BUSY,
    # 2 CROWDED, 3 BLOCKED. 네 명 이상도 BLOCKED 하나로 포화시킨다.
    level = min(person_count, 3)
    if level == 3:
        return level, None, False
    return level, 1.0 + level * 0.1, True
