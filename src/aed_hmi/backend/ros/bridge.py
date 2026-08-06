"""ROS 2 구독을 맡는다. 백엔드에서 rclpy 를 아는 곳은 여기와 converters 뿐이다.

rclpy 는 자기 스레드에서 돌고, FastAPI 는 asyncio 로 돈다. 둘을 직접
붙이면 이벤트 루프가 막힌다. 그래서 여기서는 콜백으로 도메인 객체만
넘기고, 그것을 asyncio 로 옮기는 일은 stream/hub 가 맡는다.
"""

import threading
from typing import Callable, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ..domain.models import EmergencyEventSnapshot, MissionEvent, RobotSnapshot
from . import topics
from .converters import (
    SpeedEstimator,
    to_emergency_event,
    to_mission_event,
    to_robot_snapshot,
)


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
        streams=topics.DEFAULT_STREAMS,
    ) -> None:
        self._on_robot = on_robot
        self._on_event = on_event
        self._on_mission = on_mission
        self._on_frame = on_frame
        self._on_person_count = on_person_count
        self._streams = streams
        self._speeds: dict[str, SpeedEstimator] = {}
        self._node: Optional[Node] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._started = threading.Event()

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
        self._started.set()
        try:
            self._executor.spin()
        except Exception:
            self._started.clear()

    def _subscribe(self) -> None:
        from aed_interfaces.msg import EmergencyEvent, MissionStatus, RobotState
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import UInt32

        node = self._node
        node.create_subscription(
            RobotState, topics.ROBOT_STATE_TOPIC,
            self._handle_robot_state, topics.STATE_QOS,
        )
        node.create_subscription(
            MissionStatus, topics.MISSION_STATUS_TOPIC,
            self._handle_mission_status, topics.STATE_QOS,
        )

        # 이벤트는 두 경로로 온다.
        #  - vision_detector 가 카메라마다 직접 내는 것
        #  - mission_manager 쪽이 보는 공용 토픽
        # 지금은 둘을 잇는 노드가 없어서 카메라 토픽만 실제로 값이 온다.
        # 둘 다 구독해 두면 나중에 연결되어도 화면은 그대로 동작한다.
        node.create_subscription(
            EmergencyEvent, topics.AGGREGATE_EVENT_TOPIC,
            self._handle_emergency_event, topics.STATE_QOS,
        )
        for camera_id in topics.VISION_CAMERA_IDS:
            node.create_subscription(
                EmergencyEvent,
                topics.vision_topic(camera_id, "emergency_event"),
                self._handle_emergency_event, topics.STATE_QOS,
            )
            # 화면에 몇 명 잡혔는지 띄우기 위한 값. EmergencyEvent 는 확정
            # 전후로만 오지만 person_count 는 매 프레임 나온다.
            node.create_subscription(
                UInt32, topics.vision_topic(camera_id, "person_count"),
                lambda message, stream_id=camera_id:
                    self._on_person_count(stream_id, int(message.data)),
                topics.STATE_QOS,
            )

        for source in self._streams:
            # 기본 인자로 stream_id 를 묶는다. 안 하면 모든 콜백이 마지막
            # 반복의 값을 보게 된다.
            node.create_subscription(
                CompressedImage, source.topic,
                lambda message, stream_id=source.stream_id:
                    self._on_frame(stream_id, bytes(message.data)),
                topics.IMAGE_QOS,
            )

    # ------------------------------------------------------------------
    # 콜백. ROS 스레드에서 불린다.
    # ------------------------------------------------------------------

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
