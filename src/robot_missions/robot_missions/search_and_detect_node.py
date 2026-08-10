"""Polygon 커버리지 수색과 쓰러진 사람 접근을 수행하는 ROS 2 노드.

역할과 동작 흐름
------------------
1. Mission Manager가 ``search_action_name`` Action으로 보낸 Polygon 꼭짓점을
   map 좌표계로 변환한다.
2. Polygon 내부에 Boustrophedon(왕복 지그재그) waypoint를 생성하고 Nav2의
   ``NavigateThroughPoses`` Action으로 순차 주행한다.
3. 수색 중 YOLO의 ``EmergencyEvent`` 또는 선택적인 ``DetectionSummary``에서
   임계값 이상의 쓰러진 사람을 받으면 coverage goal을 즉시 취소한다.
4. 검출 좌표 앞 ``approach_distance_m`` 지점으로 목표를 바꾸고 Nav2의
   ``NavigateToPose`` Action으로 접근한다. 도착하면 Mission Manager Action을
   성공 처리하고, 미검출 수색 완료/TF 실패/Nav2 실패는 abort 처리한다.

ROS 2 인터페이스
----------------
* Action server: ``nav2_msgs/action/NavigateThroughPoses``
  - 이름: ``search_action_name`` (기본 ``search_and_detect``)
  - ``goal.poses``를 닫히지 않은 Polygon 꼭짓점 배열로 해석한다.
  - 프로젝트에 Polygon 전용 Action이 아직 없으므로 단일 파일만으로 현재
    의존성에서 빌드 가능하게 기존 배열형 Action을 재사용한다. 서버
    이름은
    Nav2 서버 이름과 반드시 다르게 둔다.
* YOLO subscriber: ``aed_interfaces/msg/EmergencyEvent``
  - 이름: ``detection_event_topic`` (기본 ``vision/emergency_event``)
* 선택 subscriber: ``aed_interfaces/msg/DetectionSummary``
  - ``detection_summary_topic``이 빈 문자열이 아닐 때만 생성한다.
* Nav2 clients: ``NavigateThroughPoses`` / ``NavigateToPose``
  - Action 이름은 각각 파라미터로 변경할 수 있다.

주요 파라미터
-------------
모든 토픽/Action/Frame 이름, 카메라 FOV, 검출 거리, tool width, 경로 겹침률,
탐지 신뢰도, 연속 탐지 수, 접근 거리, Nav2/TF/수색 timeout은 ROS 파라미터로
선언되어 YAML 또는 launch에서 덮어쓸 수 있다. 상대 이름을 기본값으로
사용해
``--ros-args -r __ns:=/amr_1``처럼 실행한 namespace를 자연스럽게 따른다.

독립 통합 시험
--------------
ROS/Nav2가 실행된 환경에서 이 파일에 ``--mock``을 추가하면 같은 namespace에
Mock Polygon Action goal과 Mock YOLO 이벤트를 자동으로 발행한다.

.. code-block:: bash

   python3 search_and_detect_node.py --mock \
     --ros-args -r __ns:=/amr_1 -p mock_detection_delay_s:=3.0

주의: ROS 2 Humble/Nav2의 실제 Action 타입 이름은 ``NavigateThroughPoses``와
``NavigateToPose``이다. 요구사항의 ``MapsThroughPoses``/``MapsToPose``는
해당 타입의 오기로 판단하여 실제 Humble API를 사용한다.
"""

from __future__ import annotations

import copy
import math
import sys
import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import Iterable, Optional, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from aed_interfaces.msg import DetectionSummary, EmergencyEvent
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class SearchState(Enum):
    """Search and Detect 노드의 명시적 실행 상태."""

    IDLE = auto()
    SEARCHING_COVERAGE = auto()
    PERSON_DETECTED = auto()
    APPROACHING_TARGET = auto()
    ARRIVED = auto()
    SEARCH_FAILED = auto()


@dataclass(frozen=True)
class Point2D:
    """ROS 메시지와 분리해 경로 계산에 사용하는 불변 2차원 점."""

    x: float
    y: float


@dataclass(frozen=True)
class Detection:
    """신뢰도 검증을 마쳐 latch된 구조 대상 정보."""

    location: PointStamped
    confidence: float
    received_at_s: float


class BoustrophedonPathPlanner:
    """단순/오목 Polygon을 수평 scanline으로 잘라 왕복 경로를 생성한다.

    ROS 타입을 전혀 사용하지 않으므로 일반 Python 단위 테스트에서 경로
    생성 알고리즘만 독립적으로 검증할 수 있다. ``sweep_angle_rad``만큼
    Polygon을 회전시킨 좌표계에서 scanline을 만든 뒤 원래 좌표계로
    되돌린다.

    Args:
        lane_spacing_m: 인접 수색 lane 사이의 거리(m).
        sweep_angle_rad: map x축 기준 수색 진행 방향(rad).
        boundary_margin_m: 각 scanline 끝점을 경계 안쪽으로 당길 거리(m).
        minimum_lane_length_m: 이보다 짧은 교차 구간은 제거한다.
        max_waypoints: 허용 waypoint 수. 0이면 제한하지 않는다.
    """

    _EPSILON = 1.0e-9

    def __init__(
        self,
        lane_spacing_m: float,
        sweep_angle_rad: float = 0.0,
        boundary_margin_m: float = 0.0,
        minimum_lane_length_m: float = 0.05,
        max_waypoints: int = 0,
    ) -> None:
        if lane_spacing_m <= 0.0:
            raise ValueError("lane_spacing_m must be positive")
        if boundary_margin_m < 0.0:
            raise ValueError("boundary_margin_m must be non-negative")
        if minimum_lane_length_m <= 0.0:
            raise ValueError("minimum_lane_length_m must be positive")
        if max_waypoints < 0:
            raise ValueError("max_waypoints must be zero or positive")
        self.lane_spacing_m = lane_spacing_m
        self.sweep_angle_rad = sweep_angle_rad
        self.boundary_margin_m = boundary_margin_m
        self.minimum_lane_length_m = minimum_lane_length_m
        self.max_waypoints = max_waypoints

    def generate(self, polygon: Sequence[Point2D]) -> list[Point2D]:
        """Polygon 내부의 Boustrophedon waypoint를 생성한다.

        Args:
            polygon: 순서대로 연결되는 Polygon 꼭짓점. 마지막 점을 처음 점과
                중복해서 넣어도 되며 내부에서 한 번 제거한다.

        Returns:
            각 lane의 시작/끝을 번갈아 이은 map 좌표 waypoint 목록.

        Raises:
            ValueError: Polygon이 퇴화했거나 경로를 만들 수 없을 때.
        """
        vertices = self._normalize_polygon(polygon)
        rotated = [self._rotate(point, -self.sweep_angle_rad) for point in vertices]
        min_y = min(point.y for point in rotated)
        max_y = max(point.y for point in rotated)

        # 첫 lane을 경계에서 spacing/2만큼 안쪽에 둔다. 폭이 spacing보다 좁은
        # Polygon도 한 번은 훑도록 그 경우에는 중앙 scanline을 사용한다.
        if max_y - min_y <= self.lane_spacing_m:
            scanline_y_values = [(min_y + max_y) * 0.5]
        else:
            scanline_y_values = []
            y = min_y + self.lane_spacing_m * 0.5
            while y < max_y - self._EPSILON:
                scanline_y_values.append(y)
                y += self.lane_spacing_m

        path_rotated: list[Point2D] = []
        forward = True
        for y in scanline_y_values:
            intersections = self._scanline_intersections(rotated, y)
            intervals = []
            # even-odd 규칙으로 교차점 두 개씩을 Polygon 내부 구간으로 본다.
            for index in range(0, len(intersections) - 1, 2):
                left = intersections[index] + self.boundary_margin_m
                right = intersections[index + 1] - self.boundary_margin_m
                if right - left >= self.minimum_lane_length_m:
                    intervals.append((left, right))

            if not intervals:
                continue

            # 한 줄은 좌→우, 다음 줄은 우→좌 순서로 배치해 lane 끝의
            # 불필요한 공회전을 줄인다. 오목 Polygon은 한 scanline에
            # 구간이 여러 개일 수 있어 진행 방향에 맞춰 구간 순서도 함께
            # 뒤집는다.
            if forward:
                for left, right in intervals:
                    path_rotated.extend((Point2D(left, y), Point2D(right, y)))
            else:
                for left, right in reversed(intervals):
                    path_rotated.extend((Point2D(right, y), Point2D(left, y)))
            forward = not forward

        if len(path_rotated) < 2:
            raise ValueError(
                "polygon 내부에서 유효한 coverage lane을 만들 수 없습니다"
            )
        if self.max_waypoints and len(path_rotated) > self.max_waypoints:
            raise ValueError(
                f"생성 waypoint {len(path_rotated)}개가 max_waypoints="
                f"{self.max_waypoints}를 초과합니다"
            )
        return [self._rotate(point, self.sweep_angle_rad) for point in path_rotated]

    @classmethod
    def _normalize_polygon(cls, polygon: Sequence[Point2D]) -> list[Point2D]:
        """중복 끝점을 제거하고 꼭짓점 수와 면적을 검증한다."""
        vertices = list(polygon)
        if len(vertices) >= 2 and cls._distance(vertices[0], vertices[-1]) < cls._EPSILON:
            vertices.pop()
        if len(vertices) < 3:
            raise ValueError(
                "polygon에는 서로 다른 꼭짓점이 최소 3개 필요합니다"
            )
        if any(not math.isfinite(p.x) or not math.isfinite(p.y) for p in vertices):
            raise ValueError("polygon 좌표는 유한한 실수여야 합니다")
        double_area = sum(
            current.x * following.y - following.x * current.y
            for current, following in zip(vertices, vertices[1:] + vertices[:1])
        )
        if abs(double_area) < cls._EPSILON:
            raise ValueError("polygon 면적이 0입니다")
        return vertices

    @classmethod
    def _scanline_intersections(
        cls, polygon: Sequence[Point2D], scanline_y: float
    ) -> list[float]:
        """half-open edge 규칙으로 꼭짓점 중복 없이 교차 x좌표를 구한다."""
        intersections: list[float] = []
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            if abs(start.y - end.y) < cls._EPSILON:
                continue
            low, high = (start, end) if start.y < end.y else (end, start)
            # [low.y, high.y) 규칙은 scanline이 꼭짓점을 지날 때 같은 점을
            # 양쪽 edge에서 두 번 세어 내부/외부 판정이 뒤집히는 것을
            # 막는다.
            if not (low.y <= scanline_y < high.y):
                continue
            ratio = (scanline_y - start.y) / (end.y - start.y)
            intersections.append(start.x + ratio * (end.x - start.x))
        intersections.sort()
        return intersections

    @staticmethod
    def _rotate(point: Point2D, angle_rad: float) -> Point2D:
        """원점 기준으로 2차원 점을 회전한다."""
        cosine = math.cos(angle_rad)
        sine = math.sin(angle_rad)
        return Point2D(
            cosine * point.x - sine * point.y,
            sine * point.x + cosine * point.y,
        )

    @staticmethod
    def _distance(first: Point2D, second: Point2D) -> float:
        """두 점의 유클리드 거리를 반환한다."""
        return math.hypot(first.x - second.x, first.y - second.y)


class DetectionStateController:
    """YOLO 결과의 임계값/연속성 검사와 최초 대상 latch를 담당한다.

    ROS subscription과 분리되어 있어 가짜 ``PointStamped``만 주입해 탐지 상태
    전이를 단위 테스트할 수 있다. 한 임무에서 최초로 확정된 대상을
    유지하여 접근 도중 프레임별 좌표 흔들림 때문에 Nav2 goal이 계속
    바뀌지 않게 한다.

    Args:
        confidence_threshold: 수용할 최소 YOLO 신뢰도.
        required_hits: 확정에 필요한 연속 입력 수.
        reset_timeout_s: 두 입력 사이가 이 값보다 길면 연속 수를 초기화한다.
    """

    def __init__(
        self,
        confidence_threshold: float,
        required_hits: int,
        reset_timeout_s: float,
    ) -> None:
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if required_hits < 1:
            raise ValueError("required_hits must be at least 1")
        if reset_timeout_s <= 0.0:
            raise ValueError("reset_timeout_s must be positive")
        self.confidence_threshold = confidence_threshold
        self.required_hits = required_hits
        self.reset_timeout_s = reset_timeout_s
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        """새 수색 임무를 위해 연속 검출 수와 latch를 초기화한다."""
        with getattr(self, "_lock", threading.Lock()):
            self._hit_count = 0
            self._last_hit_s: Optional[float] = None
            self._target: Optional[Detection] = None

    def register(
        self,
        location: PointStamped,
        confidence: float,
        now_s: float,
    ) -> bool:
        """한 YOLO 결과를 평가하고 이번 호출에서 새로 확정됐는지 반환한다.

        Args:
            location: 검출 대상의 좌표와 frame/stamp.
            confidence: YOLO 검출 신뢰도.
            now_s: 입력을 받은 ROS clock 시각(초).

        Returns:
            필요한 연속 hit 수를 처음 충족해 target을 latch했으면 ``True``.
        """
        with self._lock:
            if self._target is not None:
                return False
            if (
                confidence < self.confidence_threshold
                or not location.header.frame_id
                or not math.isfinite(location.point.x)
                or not math.isfinite(location.point.y)
            ):
                self._hit_count = 0
                self._last_hit_s = None
                return False
            if (
                self._last_hit_s is None
                or now_s - self._last_hit_s > self.reset_timeout_s
            ):
                self._hit_count = 0
            self._last_hit_s = now_s
            self._hit_count += 1
            if self._hit_count < self.required_hits:
                return False
            self._target = Detection(copy.deepcopy(location), confidence, now_s)
            return True

    @property
    def target(self) -> Optional[Detection]:
        """현재 latch된 대상을 반환한다(객체는 외부에서 불변 취급)."""
        with self._lock:
            return self._target


class ApproachPoseCalculator:
    """현재 로봇 위치와 대상 좌표로 안전한 접근 pose를 계산한다."""

    @staticmethod
    def calculate(
        robot: Point2D,
        target: Point2D,
        standoff_m: float,
    ) -> tuple[Point2D, float]:
        """대상을 바라보며 standoff 거리만큼 떨어진 pose를 반환한다.

        Args:
            robot: 현재 로봇의 map 좌표.
            target: 쓰러진 사람의 map 좌표.
            standoff_m: 사람과 최종 정지점 사이 거리(m).

        Returns:
            ``(접근 위치, target을 바라보는 yaw)``.
        """
        if standoff_m < 0.0:
            raise ValueError("standoff_m must be non-negative")
        dx = target.x - robot.x
        dy = target.y - robot.y
        distance = math.hypot(dx, dy)
        if distance < 1.0e-9:
            return robot, 0.0
        yaw = math.atan2(dy, dx)
        # 이미 standoff 안에 있다면 더 가까이 움직이지 않고 제자리에서
        # 대상만 바라보게 한다. 그렇지 않으면 현재→대상 직선상에서 사람
        # 앞에 정지한다.
        travel = max(0.0, distance - standoff_m)
        scale = travel / distance
        return Point2D(robot.x + dx * scale, robot.y + dy * scale), yaw


class SearchAndDetectNode(Node):
    """Mission Manager, YOLO, TF2, Nav2를 연결하는 상태 기반 ROS 2 노드."""

    def __init__(self) -> None:
        """파라미터와 ROS 2 Action/subscription/TF 인터페이스를 초기화한다."""
        super().__init__("search_and_detect_node")
        self._callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self._load_and_validate_parameters()

        self._lock = threading.RLock()
        self._state = SearchState.IDLE
        self._goal_reserved = False
        self._active_server_goal = None
        self._active_nav_goal = None
        self._search_started_s: Optional[float] = None
        self._failure_reason = ""

        self._detector = DetectionStateController(
            self.detection_confidence_threshold,
            self.detection_required_hits,
            self.detection_reset_timeout_s,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._coverage_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.navigate_through_poses_action,
            callback_group=self._callback_group,
        )
        self._approach_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action,
            callback_group=self._callback_group,
        )
        self._search_server = ActionServer(
            self,
            NavigateThroughPoses,
            self.search_action_name,
            execute_callback=self._execute_search,
            goal_callback=self._on_search_goal,
            cancel_callback=self._on_search_cancel,
            callback_group=self._callback_group,
        )

        self._event_subscription = self.create_subscription(
            EmergencyEvent,
            self.detection_event_topic,
            self._on_emergency_event,
            10,
            callback_group=self._callback_group,
        )
        self._summary_subscription = None
        if self.detection_summary_topic:
            self._summary_subscription = self.create_subscription(
                DetectionSummary,
                self.detection_summary_topic,
                self._on_detection_summary,
                10,
                callback_group=self._callback_group,
            )

        # Action execute callback이 Nav2 결과를 기다리는 동안에도 timeout을
        # 감시하려면 별도 timer callback이 필요하므로 Reentrant group에 둔다.
        self._watchdog = self.create_timer(
            0.1, self._check_search_timeout, callback_group=self._callback_group
        )
        self.get_logger().info(
            "SearchAndDetect ready: namespace=%s search_action=%s "
            "coverage_action=%s approach_action=%s detection=%s"
            % (
                self.get_namespace(),
                self.search_action_name,
                self.navigate_through_poses_action,
                self.navigate_to_pose_action,
                self.detection_event_topic,
            )
        )

    def _declare_parameters(self) -> None:
        """외부 설정에서 바꿀 수 있는 모든 통합 파라미터를 선언한다."""
        defaults = (
            # 상대 이름은 실행 namespace(/amr_1, /amr_2)를 자동으로 따른다.
            ("search_action_name", "search_and_detect"),
            ("detection_event_topic", "vision/emergency_event"),
            ("detection_summary_topic", ""),
            ("navigate_through_poses_action", "navigate_through_poses"),
            ("navigate_to_pose_action", "navigate_to_pose"),
            ("map_frame", "map"),
            ("base_frame", "base_link"),
            # coverage 폭 = min(tool width, FOV 기반 관측 폭 * (1-overlap)).
            ("camera_fov_deg", 69.0),
            ("detection_range_m", 2.0),
            ("tool_width_m", 0.6),
            ("path_overlap_ratio", 0.20),
            ("sweep_angle_deg", 0.0),
            ("boundary_margin_m", 0.10),
            ("minimum_lane_length_m", 0.20),
            ("max_waypoints", 0),
            ("detection_confidence_threshold", 0.60),
            ("detection_required_hits", 1),
            ("detection_reset_timeout_s", 1.0),
            ("detection_max_age_s", 2.0),
            ("approach_distance_m", 0.70),
            ("nav2_server_timeout_s", 3.0),
            ("tf_timeout_s", 0.5),
            ("search_timeout_s", 180.0),
        )
        self.declare_parameters("", defaults)

    def _load_and_validate_parameters(self) -> None:
        """파라미터를 읽고 잘못된 안전 설정을 조기에 거부한다."""
        string_names = (
            "search_action_name",
            "detection_event_topic",
            "detection_summary_topic",
            "navigate_through_poses_action",
            "navigate_to_pose_action",
            "map_frame",
            "base_frame",
        )
        for name in string_names:
            setattr(self, name, str(self.get_parameter(name).value).strip())
        required_strings = (
            "search_action_name",
            "detection_event_topic",
            "navigate_through_poses_action",
            "navigate_to_pose_action",
            "map_frame",
            "base_frame",
        )
        if any(not getattr(self, name) for name in required_strings):
            raise ValueError(
                "필수 topic/action/frame 파라미터는 빈 문자열일 수 없습니다"
            )
        if self.search_action_name == self.navigate_through_poses_action:
            raise ValueError(
                "search_action_name과 Nav2 coverage Action 이름은 달라야 합니다"
            )

        float_names = (
            "camera_fov_deg",
            "detection_range_m",
            "tool_width_m",
            "path_overlap_ratio",
            "sweep_angle_deg",
            "boundary_margin_m",
            "minimum_lane_length_m",
            "detection_confidence_threshold",
            "detection_reset_timeout_s",
            "detection_max_age_s",
            "approach_distance_m",
            "nav2_server_timeout_s",
            "tf_timeout_s",
            "search_timeout_s",
        )
        for name in float_names:
            setattr(self, name, float(self.get_parameter(name).value))
        self.max_waypoints = int(self.get_parameter("max_waypoints").value)
        self.detection_required_hits = int(
            self.get_parameter("detection_required_hits").value
        )

        if not 0.0 < self.camera_fov_deg < 180.0:
            raise ValueError("camera_fov_deg must be in (0, 180)")
        if self.detection_range_m <= 0.0 or self.tool_width_m <= 0.0:
            raise ValueError("detection_range_m and tool_width_m must be positive")
        if not 0.0 <= self.path_overlap_ratio < 1.0:
            raise ValueError("path_overlap_ratio must be in [0, 1)")
        if self.detection_max_age_s <= 0.0:
            raise ValueError("detection_max_age_s must be positive")
        if self.nav2_server_timeout_s <= 0.0 or self.tf_timeout_s <= 0.0:
            raise ValueError("Nav2/TF timeout must be positive")
        if self.search_timeout_s < 0.0:
            raise ValueError("search_timeout_s must be zero or positive")

    def _on_search_goal(self, goal_request) -> GoalResponse:
        """Polygon과 단일 임무 제약을 검사해 goal을 수락/거절한다."""
        if len(goal_request.poses) < 3:
            self.get_logger().warning("Search goal rejected: polygon needs >= 3 poses")
            return GoalResponse.REJECT
        if any(not pose.header.frame_id for pose in goal_request.poses):
            self.get_logger().warning("Search goal rejected: polygon frame_id is empty")
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                self.get_logger().warning("Search goal rejected: another search is active")
                return GoalResponse.REJECT
            # goal_callback과 execute_callback 사이에 두 번째 요청이 끼어드는 race를
            # 막기 위해 수락 시점에 슬롯을 미리 예약한다.
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _on_search_cancel(self, _goal_handle) -> CancelResponse:
        """Mission Manager 취소와 함께 Nav2 goal도 즉시 취소한다."""
        self.get_logger().warning("Mission Manager requested search cancellation")
        self._request_active_nav_cancel("mission manager cancel")
        return CancelResponse.ACCEPT

    async def _execute_search(self, goal_handle):
        """한 Polygon 수색, 탐지 전환, 대상 접근 Action을 끝까지 실행한다."""
        result = NavigateThroughPoses.Result()
        with self._lock:
            self._active_server_goal = goal_handle
            self._failure_reason = ""
            self._search_started_s = self._now_seconds()
            self._detector.reset()
            self._set_state_locked(SearchState.IDLE)

        try:
            polygon = self._polygon_in_map(goal_handle.request.poses)
            planner = BoustrophedonPathPlanner(
                lane_spacing_m=self._effective_lane_spacing(),
                sweep_angle_rad=math.radians(self.sweep_angle_deg),
                boundary_margin_m=self.boundary_margin_m,
                minimum_lane_length_m=self.minimum_lane_length_m,
                max_waypoints=self.max_waypoints,
            )
            coverage_points = planner.generate(polygon)
            coverage_poses = self._points_to_poses(coverage_points)
            self.get_logger().info(
                f"Coverage path generated: {len(coverage_poses)} poses, "
                f"lane_spacing={planner.lane_spacing_m:.2f} m"
            )

            with self._lock:
                self._set_state_locked(SearchState.SEARCHING_COVERAGE)
            coverage_status = await self._navigate_coverage(coverage_poses, goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warning("Search action canceled by Mission Manager")
                return result

            with self._lock:
                watchdog_reason = self._failure_reason
            if watchdog_reason:
                return self._abort_search(goal_handle, result, watchdog_reason)

            detection = self._detector.target
            if detection is None:
                if coverage_status == GoalStatus.STATUS_SUCCEEDED:
                    reason = "coverage search completed without detecting a fallen person"
                else:
                    reason = f"Nav2 coverage failed with status={coverage_status}"
                return self._abort_search(goal_handle, result, reason)

            # 검출 callback은 먼저 coverage goal cancel을 요청한다. 취소 결과를
            # await한 뒤에만 NavigateToPose를 보내 두 Nav2 goal이 동시에 로봇을
            # 제어하는 경쟁 상태를 방지한다.
            try:
                target = self._point_in_map(detection.location)
                approach_pose = self._make_approach_pose(target)
            except (TransformException, ValueError) as error:
                return self._abort_search(
                    goal_handle, result, f"target TF/approach calculation failed: {error}"
                )

            with self._lock:
                self._set_state_locked(SearchState.APPROACHING_TARGET)
            approach_status = await self._navigate_to_target(approach_pose, goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warning("Approach canceled by Mission Manager")
                return result
            if approach_status != GoalStatus.STATUS_SUCCEEDED:
                return self._abort_search(
                    goal_handle,
                    result,
                    f"Nav2 target approach failed with status={approach_status}",
                )

            with self._lock:
                self._set_state_locked(SearchState.ARRIVED)
            goal_handle.succeed()
            self.get_logger().info("Fallen person reached; search action succeeded")
            return result
        except TransformException as error:
            return self._abort_search(goal_handle, result, f"polygon TF timeout: {error}")
        except (ValueError, RuntimeError) as error:
            return self._abort_search(goal_handle, result, str(error))
        # Action server worker가 예외로 유실되지 않게 반드시 abort로 종결한다.
        except Exception as error:
            self.get_logger().error(f"Unexpected search error: {error}")
            return self._abort_search(goal_handle, result, f"unexpected error: {error}")
        finally:
            with self._lock:
                self._active_nav_goal = None
                self._active_server_goal = None
                self._search_started_s = None
                self._goal_reserved = False

    async def _navigate_coverage(
        self, poses: Sequence[PoseStamped], server_goal_handle
    ) -> int:
        """NavigateThroughPoses goal을 전송하고 최종 GoalStatus를 반환한다."""
        if not self._coverage_client.wait_for_server(
            timeout_sec=self.nav2_server_timeout_s
        ):
            raise RuntimeError("Nav2 NavigateThroughPoses action server unavailable")
        if server_goal_handle.is_cancel_requested:
            return GoalStatus.STATUS_CANCELED

        goal = NavigateThroughPoses.Goal()
        goal.poses = list(poses)
        goal.behavior_tree = ""
        future = self._coverage_client.send_goal_async(
            goal,
            feedback_callback=lambda message: self._forward_nav_feedback(
                message.feedback, server_goal_handle
            ),
        )
        nav_goal_handle = await future
        if not nav_goal_handle.accepted:
            raise RuntimeError("Nav2 rejected coverage goal")
        with self._lock:
            self._active_nav_goal = nav_goal_handle
            should_cancel = (
                self._detector.target is not None
                or server_goal_handle.is_cancel_requested
                or bool(self._failure_reason)
            )
        if should_cancel:
            nav_goal_handle.cancel_goal_async()
        wrapped_result = await nav_goal_handle.get_result_async()
        with self._lock:
            if self._active_nav_goal is nav_goal_handle:
                self._active_nav_goal = None
        return wrapped_result.status

    async def _navigate_to_target(
        self, pose: PoseStamped, server_goal_handle
    ) -> int:
        """NavigateToPose로 대상 앞 접근 pose까지 이동하고 상태를 반환한다."""
        if not self._approach_client.wait_for_server(
            timeout_sec=self.nav2_server_timeout_s
        ):
            raise RuntimeError("Nav2 NavigateToPose action server unavailable")
        if server_goal_handle.is_cancel_requested:
            return GoalStatus.STATUS_CANCELED

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ""
        future = self._approach_client.send_goal_async(
            goal,
            feedback_callback=lambda message: self._forward_nav_feedback(
                message.feedback, server_goal_handle
            ),
        )
        nav_goal_handle = await future
        if not nav_goal_handle.accepted:
            raise RuntimeError("Nav2 rejected target approach goal")
        with self._lock:
            self._active_nav_goal = nav_goal_handle
            should_cancel = server_goal_handle.is_cancel_requested
        if should_cancel:
            nav_goal_handle.cancel_goal_async()
        wrapped_result = await nav_goal_handle.get_result_async()
        with self._lock:
            if self._active_nav_goal is nav_goal_handle:
                self._active_nav_goal = None
        return wrapped_result.status

    def _on_emergency_event(self, message: EmergencyEvent) -> None:
        """YOLO EmergencyEvent에서 검출 위치와 신뢰도를 수신한다."""
        if message.status not in (EmergencyEvent.DETECTED, EmergencyEvent.CONFIRMED):
            return
        location = PointStamped()
        location.header = message.location.header
        location.point = message.location.point
        self._consider_detection(location, float(message.confidence), message.detected_at)

    def _on_detection_summary(self, message: DetectionSummary) -> None:
        """선택적인 DetectionSummary 입력을 동일한 탐지기로 전달한다."""
        if message.fallen_count == 0:
            return
        self._consider_detection(
            message.fallen_location,
            float(message.top_fallen_confidence),
            message.stamp,
        )

    def _consider_detection(self, location: PointStamped, confidence: float, stamp) -> None:
        """상태/신선도/신뢰도를 검사하고 Nav2 cancel로 전환한다."""
        with self._lock:
            if self._state != SearchState.SEARCHING_COVERAGE:
                return
        now_s = self._now_seconds()
        message_s = self._stamp_seconds(stamp)
        # bag 재생/통신 지연으로 오래된 detection이 도착해 현재 임무를 잘못
        # 가로채는 것을 막는다. stamp가 0이면 수신 시각을 사용한다.
        if message_s > 0.0 and now_s - message_s > self.detection_max_age_s:
            self.get_logger().warning(
                f"Ignoring stale detection: age={now_s - message_s:.2f} s"
            )
            return
        if not self._detector.register(location, confidence, now_s):
            return

        with self._lock:
            self._set_state_locked(SearchState.PERSON_DETECTED)
        self.get_logger().warning(
            f"Fallen person detected (confidence={confidence:.3f}); "
            "canceling coverage goal immediately"
        )
        self._request_active_nav_cancel("person detected")

    def _polygon_in_map(self, poses: Iterable[PoseStamped]) -> list[Point2D]:
        """Action Goal의 여러 PoseStamped 꼭짓점을 map 좌표로 변환한다."""
        polygon = []
        for pose in poses:
            point = PointStamped()
            point.header = pose.header
            point.point = pose.pose.position
            mapped = self._point_in_map(point)
            polygon.append(Point2D(mapped.point.x, mapped.point.y))
        return polygon

    def _point_in_map(self, point: PointStamped) -> PointStamped:
        """PointStamped를 map으로 바꾸며 TF timeout을 처리한다."""
        if not point.header.frame_id:
            raise ValueError("point frame_id is empty")
        if point.header.frame_id == self.map_frame:
            return copy.deepcopy(point)
        query_time = Time.from_msg(point.header.stamp)
        transform = self._tf_buffer.lookup_transform(
            self.map_frame,
            point.header.frame_id,
            query_time,
            timeout=Duration(seconds=self.tf_timeout_s),
        )
        return do_transform_point(point, transform)

    def _make_approach_pose(self, target_message: PointStamped) -> PoseStamped:
        """현재 base pose로 사람 앞 standoff PoseStamped를 만든다."""
        transform = self._tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout_s),
        )
        robot = Point2D(
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        target = Point2D(target_message.point.x, target_message.point.y)
        position, yaw = ApproachPoseCalculator.calculate(
            robot, target, self.approach_distance_m
        )
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position.x
        pose.pose.position.y = position.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _points_to_poses(self, points: Sequence[Point2D]) -> list[PoseStamped]:
        """각 waypoint가 다음 점을 바라보는 Pose 배열을 만든다."""
        poses = []
        now = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            if index + 1 < len(points):
                following = points[index + 1]
            else:
                following = points[index - 1]
            yaw = math.atan2(following.y - point.y, following.x - point.x)
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = now
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            poses.append(pose)
        return poses

    def _effective_lane_spacing(self) -> float:
        """FOV 관측 폭과 tool width 중 보수적인 lane 간격을 계산한다."""
        half_fov_rad = math.radians(self.camera_fov_deg) * 0.5
        visible_width = 2.0 * self.detection_range_m * math.tan(half_fov_rad)
        fov_spacing = visible_width * (1.0 - self.path_overlap_ratio)
        return min(self.tool_width_m, fov_spacing)

    def _forward_nav_feedback(self, nav_feedback, server_goal_handle) -> None:
        """Nav2 feedback의 공통 필드를 Mission Manager Action으로 전달한다."""
        if server_goal_handle is None or not server_goal_handle.is_active:
            return
        feedback = NavigateThroughPoses.Feedback()
        # NavigateToPose와 NavigateThroughPoses feedback은 대부분 같은 필드를
        # 공유하지만 poses_remaining은 후자에만 있으므로 hasattr로
        # 안전하게 매핑한다.
        common_fields = (
            "current_pose",
            "navigation_time",
            "estimated_time_remaining",
            "number_of_recoveries",
            "distance_remaining",
            "number_of_poses_remaining",
        )
        for field in common_fields:
            if hasattr(nav_feedback, field) and hasattr(feedback, field):
                setattr(feedback, field, getattr(nav_feedback, field))
        server_goal_handle.publish_feedback(feedback)

    def _request_active_nav_cancel(self, reason: str) -> None:
        """현재 Nav2 goal handle이 있으면 비동기 cancel을 요청한다."""
        with self._lock:
            nav_goal = self._active_nav_goal
        if nav_goal is None:
            return
        try:
            nav_goal.cancel_goal_async()
            self.get_logger().info(f"Nav2 cancel requested: {reason}")
        except Exception as error:
            self.get_logger().error(f"Failed to request Nav2 cancel: {error}")

    def _check_search_timeout(self) -> None:
        """coverage 수색 제한시간 초과 시 Nav2 goal을 취소해 Action을 깨운다."""
        if self.search_timeout_s == 0.0:
            return
        with self._lock:
            if (
                self._state != SearchState.SEARCHING_COVERAGE
                or self._search_started_s is None
                or self._failure_reason
            ):
                return
            elapsed = self._now_seconds() - self._search_started_s
            if elapsed <= self.search_timeout_s:
                return
            self._failure_reason = f"search timeout after {elapsed:.1f} s"
            self._set_state_locked(SearchState.SEARCH_FAILED)
        self.get_logger().error(self._failure_reason)
        self._request_active_nav_cancel("search timeout")

    def _abort_search(self, goal_handle, result, reason: str):
        """상태/로그/Action terminal transition을 한곳에서 일관되게 처리한다."""
        with self._lock:
            self._failure_reason = reason
            self._set_state_locked(SearchState.SEARCH_FAILED)
        if goal_handle.is_active:
            goal_handle.abort()
        self.get_logger().error(f"Search failed: {reason}")
        return result

    def _set_state_locked(self, next_state: SearchState) -> None:
        """lock을 잡은 호출자 안에서 상태를 변경하고 전이를 기록한다."""
        if self._state != next_state:
            self.get_logger().info(f"state: {self._state.name} -> {next_state.name}")
            self._state = next_state

    def _now_seconds(self) -> float:
        """ROS clock 현재 시각을 부동소수 초로 반환한다."""
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        """builtin_interfaces/Time을 초로 바꾼다."""
        return float(stamp.sec) + float(stamp.nanosec) / 1.0e9

    def destroy_node(self) -> None:
        """종료 시 진행 중 goal을 취소하고 Action 자원을 해제한다."""
        self._request_active_nav_cancel("node shutdown")
        self._search_server.destroy()
        self._coverage_client.destroy()
        self._approach_client.destroy()
        super().destroy_node()


class MockInputInjector(Node):
    """``--mock`` 실행에서 Polygon Goal과 YOLO 검출을 주입하는 시험용 노드."""

    def __init__(self) -> None:
        """Mock 파라미터, Action client, 검출 publisher를 생성한다."""
        super().__init__("search_and_detect_mock_injector")
        self.declare_parameter("search_action_name", "search_and_detect")
        self.declare_parameter("detection_event_topic", "vision/emergency_event")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter(
            "mock_polygon_xy", [-2.0, -1.5, 2.0, -1.5, 2.0, 1.5, -2.0, 1.5]
        )
        self.declare_parameter("mock_detection_xy", [0.8, 0.4])
        self.declare_parameter("mock_detection_confidence", 0.95)
        self.declare_parameter("mock_detection_delay_s", 3.0)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._polygon_xy = [
            float(value) for value in self.get_parameter("mock_polygon_xy").value
        ]
        self._detection_xy = [
            float(value) for value in self.get_parameter("mock_detection_xy").value
        ]
        if len(self._polygon_xy) < 6 or len(self._polygon_xy) % 2:
            raise ValueError("mock_polygon_xy needs at least three x,y pairs")
        if len(self._detection_xy) != 2:
            raise ValueError("mock_detection_xy must contain x,y")

        self._action_client = ActionClient(
            self,
            NavigateThroughPoses,
            str(self.get_parameter("search_action_name").value),
        )
        self._event_publisher = self.create_publisher(
            EmergencyEvent,
            str(self.get_parameter("detection_event_topic").value),
            10,
        )
        self._goal_sent = False
        self._detection_sent = False
        self._detection_timer = None
        self._goal_timer = self.create_timer(0.5, self._try_send_goal)

    def _try_send_goal(self) -> None:
        """Action server가 준비되면 Mock Polygon을 한 번 전송한다."""
        if self._goal_sent or not self._action_client.server_is_ready():
            return
        self._goal_sent = True
        goal = NavigateThroughPoses.Goal()
        for x, y in zip(self._polygon_xy[0::2], self._polygon_xy[1::2]):
            pose = PoseStamped()
            pose.header.frame_id = self._map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            goal.poses.append(pose)
        self.get_logger().info(f"Injecting mock polygon with {len(goal.poses)} vertices")
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        """Mock Goal 수락 후 설정된 지연시간 뒤 YOLO 이벤트 timer를 시작한다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Mock search goal was rejected")
            return
        self.get_logger().info("Mock search goal accepted")
        goal_handle.get_result_async().add_done_callback(self._on_goal_result)
        delay = max(0.01, float(self.get_parameter("mock_detection_delay_s").value))
        self._detection_timer = self.create_timer(delay, self._publish_detection)

    def _publish_detection(self) -> None:
        """현재 YOLO 인터페이스와 같은 EmergencyEvent를 한 번 발행한다."""
        if self._detection_sent:
            return
        self._detection_sent = True
        if self._detection_timer is not None:
            self.destroy_timer(self._detection_timer)
            self._detection_timer = None
        now = self.get_clock().now().to_msg()
        message = EmergencyEvent()
        message.event_id = "mock-fallen-person"
        message.detected_at = now
        message.location.header.stamp = now
        message.location.header.frame_id = self._map_frame
        message.location.point.x = self._detection_xy[0]
        message.location.point.y = self._detection_xy[1]
        message.confidence = float(
            self.get_parameter("mock_detection_confidence").value
        )
        message.consecutive_detections = 1
        message.status = EmergencyEvent.CONFIRMED
        message.source_id = self.get_name()
        message.camera_id = "mock_yolo"
        message.zone_id = "mock_search_area"
        self._event_publisher.publish(message)
        self.get_logger().info(
            f"Injected mock YOLO detection at {tuple(self._detection_xy)}"
        )

    def _on_goal_result(self, future) -> None:
        """독립 시험 결과의 Action 상태를 로그로 출력한다."""
        wrapped_result = future.result()
        self.get_logger().info(f"Mock search finished: status={wrapped_result.status}")


def main(args=None) -> None:
    """노드와 선택적 Mock injector를 MultiThreadedExecutor에서 실행한다.

    ``--mock``은 ROS 인자가 아니므로 rclpy 초기화 전에 제거한다. 실제 배포 시
    해당 플래그 없이 실행하면 SearchAndDetectNode 하나만 생성된다.
    """
    raw_args = list(sys.argv if args is None else args)
    mock_mode = "--mock" in raw_args
    ros_args = [argument for argument in raw_args if argument != "--mock"]
    rclpy.init(args=ros_args)
    search_node = None
    mock_node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        search_node = SearchAndDetectNode()
        executor.add_node(search_node)
        if mock_mode:
            mock_node = MockInputInjector()
            executor.add_node(mock_node)
            search_node.get_logger().warning(
                "Mock mode enabled: Polygon and YOLO event will be injected"
            )
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if mock_node is not None:
            mock_node.destroy_node()
        if search_node is not None:
            search_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
