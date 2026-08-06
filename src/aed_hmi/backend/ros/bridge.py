"""ROS 2 구독을 맡는다. 백엔드에서 rclpy 를 아는 곳은 여기와 converters 뿐이다.

rclpy 는 자기 스레드에서 돌고, FastAPI 는 asyncio 로 돈다. 둘을 직접
붙이면 이벤트 루프가 막힌다. 그래서 여기서는 콜백으로 도메인 객체만
넘기고, 그것을 asyncio 로 옮기는 일은 stream/hub 가 맡는다.
"""

import logging
import threading
from typing import Callable, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ..domain.models import (
    EmergencyEventSnapshot,
    EtaRecord,
    MissionEvent,
    RobotSnapshot,
)
from . import topics
from .converters import (
    SpeedEstimator,
    to_assignment,
    to_emergency_event,
    to_mission_event,
    to_robot_snapshot,
)

LOGGER = logging.getLogger(__name__)

# 구독은 붙었는데 값이 안 오는지 몇 초 뒤에 확인할지.
CONNECTION_CHECK_S = 8.0


class RosBridge:
    """aed_interfaces 토픽을 구독해 도메인 객체로 넘긴다.

    on_robot / on_event / on_mission 은 ROS 실행 스레드에서 불린다.
    받는 쪽에서 오래 걸리는 일을 하면 구독이 밀리므로, 큐에 넣고 즉시 반환한다.
    """

    def __init__(
        self,
        on_robot: Callable[[RobotSnapshot], None],
        on_event: Callable[[EmergencyEventSnapshot], None],
        on_mission: Callable[[MissionEvent], None],
        on_frame: Callable[[str, bytes], None],
        on_person_count: Callable[[str, int], None],
        on_eta_record: Callable[[EtaRecord], None],
        on_assignment: Callable[..., None],
        streams=topics.DEFAULT_STREAMS,
    ) -> None:
        self._on_robot = on_robot
        self._on_event = on_event
        self._on_mission = on_mission
        self._on_frame = on_frame
        self._on_person_count = on_person_count
        self._on_eta_record = on_eta_record
        self._on_assignment = on_assignment
        self._streams = streams
        self._speeds: dict[str, SpeedEstimator] = {}
        self._node: Optional[Node] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._started = threading.Event()
        # 지난번에 발행자를 못 찾은 토픽. 바뀔 때만 로그를 남기기 위한 것이다.
        self._missing_topics: frozenset = frozenset({"__초기값__"})

    @property
    def connected(self) -> bool:
        return self._started.is_set()

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="ros-bridge", daemon=True
        )
        self._thread.start()
        # 노드가 뜨기 전에 API 가 요청을 받으면 connected 가 거짓이 된다.
        # 짧게 기다려 그 창을 줄인다.
        self._started.wait(timeout=5.0)

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._started.clear()

    def _run(self) -> None:
        rclpy.init()
        self._node = Node("aed_hmi_bridge")
        self._subscribe()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        # 구독이 안 붙어도 ROS 2 는 아무 말이 없다. 몇 초 뒤에 직접 세어
        # 본다. "ROS 수신" 이라 떠 있는데 화면만 비는 상황을 막는다.
        self._node.create_timer(CONNECTION_CHECK_S, self._report_matches)
        self._started.set()
        try:
            self._executor.spin()
        except Exception:
            self._started.clear()

    def _subscribe(self) -> None:
        from aed_interfaces.msg import (
            EmergencyEvent, MissionAssignment, MissionStatus, RobotState,
        )
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String, UInt32

        node = self._node
        node.create_subscription(
            RobotState, topics.ROBOT_STATE_TOPIC,
            self._handle_robot_state, topics.state_qos(),
        )
        node.create_subscription(
            MissionStatus, topics.MISSION_STATUS_TOPIC,
            self._handle_mission_status, topics.state_qos(),
        )

        # 이벤트는 두 경로로 온다.
        #  - vision_detector 가 카메라마다 직접 내는 것
        #  - mission_manager 쪽이 보는 공용 토픽
        # 지금은 둘을 잇는 노드가 없어서 카메라 토픽만 실제로 값이 온다.
        # 둘 다 구독해 두면 나중에 연결되어도 화면은 그대로 동작한다.
        node.create_subscription(
            EmergencyEvent, topics.AGGREGATE_EVENT_TOPIC,
            self._handle_emergency_event, topics.state_qos(),
        )
        for camera_id in topics.VISION_CAMERA_IDS:
            node.create_subscription(
                EmergencyEvent,
                topics.vision_topic(camera_id, "emergency_event"),
                self._handle_emergency_event, topics.state_qos(),
            )
            # 화면에 몇 명 잡혔는지 띄우기 위한 값. EmergencyEvent 는 확정
            # 전후로만 오지만 person_count 는 매 프레임 나온다.
            node.create_subscription(
                UInt32, topics.vision_topic(camera_id, "person_count"),
                lambda message, stream_id=camera_id:
                    self._on_person_count(stream_id, int(message.data)),
                topics.state_qos(),
            )

        # 배정. 목표 좌표가 실려 오는 유일한 메시지라, 이걸 안 받으면
        # 화면의 목표 좌표와 도착 예상이 영영 빈다. 로봇마다 따로 온다.
        for robot_id in topics.ROBOT_IDS:
            node.create_subscription(
                MissionAssignment, topics.assignment_topic(robot_id),
                self._handle_assignment, topics.state_qos(),
            )

        # 예상과 실제를 재는 쪽이 내는 결과. 이 토픽만 std_msgs/String 에
        # JSON 이라 형이 보장되지 않으므로, converters 에서 한 번 검사한다.
        # QoS 도 다르다. TRANSIENT_LOCAL 로 맞추지 않으면 연결이 안 맺어지고
        # 경고도 없이 아무것도 안 온다.
        node.create_subscription(
            String, topics.ETA_RESULT_TOPIC,
            self._handle_eta_result, topics.latched_qos(),
        )

        for source in self._streams:
            # 기본 인자로 stream_id 를 묶는다. 안 하면 모든 콜백이 마지막
            # 반복의 값을 보게 된다.
            node.create_subscription(
                CompressedImage, source.topic,
                lambda message, stream_id=source.stream_id:
                    self._on_frame(stream_id, bytes(message.data)),
                topics.image_qos(),
            )

    def _report_matches(self) -> None:
        """발행자를 못 찾은 토픽을 알린다.

        QoS 가 안 맞으면 ROS 2 는 연결을 안 맺고 경고도 안 낸다. 특히
        발행이 BEST_EFFORT 인데 구독이 RELIABLE 이면 그렇다. 화면에는
        붙은 것처럼 보이므로, 여기서 이름을 대고 알려 준다.

        바뀔 때만 적는다. 매번 적으면 로그가 같은 줄로 덮이고, 정작
        로봇이 들어오거나 빠진 순간을 못 찾는다.
        """
        watched = [
            topics.ROBOT_STATE_TOPIC,
            topics.MISSION_STATUS_TOPIC,
            topics.AGGREGATE_EVENT_TOPIC,
            topics.ETA_RESULT_TOPIC,
        ] + [topics.assignment_topic(r) for r in topics.ROBOT_IDS] \
          + [source.topic for source in self._streams]

        missing = frozenset(
            name for name in watched
            if self._node.count_publishers(name) == 0
        )
        if missing == self._missing_topics:
            return
        self._missing_topics = missing

        if not missing:
            LOGGER.info("구독 %d개 모두 발행자를 찾았다", len(watched))
            return
        LOGGER.warning(
            "발행자를 못 찾은 토픽 %d/%d: %s",
            len(missing), len(watched), ", ".join(sorted(missing)),
        )
        LOGGER.warning(
            "ros2 topic list 에는 보이는데 여기 있다면 QoS 불일치다. "
            "발행이 BEST_EFFORT 이면 "
            "AED_HMI_STATE_RELIABILITY=best_effort 로 다시 띄운다."
        )

    # ------------------------------------------------------------------
    # 콜백. ROS 스레드에서 불린다.
    # ------------------------------------------------------------------

    def _handle_eta_result(self, message) -> None:
        # 이 토픽만 형이 보장되지 않는다. 검사는 EtaRecord.from_json 이
        # 하고, 깨진 것은 로그를 남기고 None 이 온다. 통계 한 건을 잃는
        # 것이 관제를 끄는 것보다 낫다.
        record = EtaRecord.from_json(message.data)
        if record is not None:
            self._on_eta_record(record)

    def _handle_assignment(self, message) -> None:
        self._on_assignment(**to_assignment(message))

    def _handle_robot_state(self, message) -> None:
        estimator = self._speeds.setdefault(message.robot_id, SpeedEstimator())
        now = self._now()
        stamp = float(message.stamp.sec) + float(message.stamp.nanosec) * 1e-9
        speed = estimator.update(
            stamp, message.pose.pose.position.x, message.pose.pose.position.y
        )
        self._on_robot(to_robot_snapshot(message, speed, now))

    def _handle_emergency_event(self, message) -> None:
        self._on_event(to_emergency_event(message))

    def _handle_mission_status(self, message) -> None:
        self._on_mission(to_mission_event(message))

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9
