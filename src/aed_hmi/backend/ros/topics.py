"""구독할 토픽 이름과 QoS 를 한 곳에 모은다.

토픽 이름이 코드 여기저기 흩어지면, 네임스페이스가 바뀔 때 어디를 고쳐야
하는지 알 수 없게 된다. 이름은 실제 발행자에 맞춰야 하므로, 바꿀 때는
그쪽 코드를 보고 맞춘다.

  고정 웹캠 검출 : aed_vision/vision_detector.py  (camera_id 별 절대 토픽)
  로봇 상태/임무 : mission_manager, robot_missions (/aed/... 공용 토픽)
  로봇 카메라   : turtlebot4 기본 OAK-D
  ETA 측정      : multi_robot_emergency/mission_manager.py

**이 파일은 rclpy 없이도 import 되어야 한다.** context.py 가 화면 구성을
얻으려고 여기를 읽는데, 그때 rclpy 를 끌어오면 ROS 가 없는 PC 에서는
--mock 조차 못 뜬다. 화면만 만드는 사람이 ROS 를 깔아야 할 이유는 없다.
그래서 QoS 는 모듈을 읽을 때가 아니라 실제로 구독할 때 만든다.
"""

import os
from dataclasses import dataclass


def state_qos():
    """상태·이벤트용. 놓치면 안 되므로 신뢰성 있게 받는다."""
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def image_qos():
    """영상용. 최신 프레임만 의미가 있어 밀리면 버린다.

    vision_detector 의 CAMERA_QOS 와 맞춘다. 다르면 구독이 아예 안 붙는다.
    """
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def latched_qos():
    """ETA 측정 결과용.

    multi_robot_emergency/mission_manager.py 의 eta_result_qos 와 같아야
    한다. durability 가 다르면 ROS 2 는 연결을 아예 안 맺고, 경고도 없이
    아무것도 안 온다.

    TRANSIENT_LOCAL 이라 관제가 나중에 떠도 최근 10건을 받는다. 도착은
    몇 분에 한 번뿐이라 놓치면 다음 것을 한참 기다려야 한다.
    """
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

# mission_manager 가 보는 공용 토픽.
ROBOT_STATE_TOPIC = "/aed/robot_state"
MISSION_STATUS_TOPIC = "/aed/mission_status"
AGGREGATE_EVENT_TOPIC = "/aed/emergency_event"

# 예상과 실제를 함께 재는 쪽(multi_robot_emergency)이 내는 토픽.
# 로봇별 Float32 도 있지만 그건 안 받는다. 어느 요청의 값인지가 안 실려
# 있어서, 두 요청이 겹치면 어느 쪽 값인지 가릴 수 없다. 요청 id 가 들어
# 있는 result 하나만 받는다.
ETA_RESULT_TOPIC = "/emergency/eta/result"

# vision_detector 가 카메라마다 따로 내는 토픽. 노트북 두 대가 같은
# ROS_DOMAIN_ID 를 써도 섞이지 않도록 절대 경로를 쓴다.
VISION_CAMERA_IDS = ("camera_open", "camera_alley")


def vision_topic(camera_id: str, suffix: str) -> str:
    return f"/{camera_id}/vision/{suffix}"


@dataclass(frozen=True)
class StreamSource:
    """화면 타일 하나가 구독할 영상 토픽.

    kind 가 "robot" 이면 로봇에 달린 OAK-D Pro, "webcam" 이면 천장 고정
    웹캠이다. detects 는 그 갈래가 검출까지 하는지를 뜻한다. 검출 노드가
    아직 없는 갈래는 영상만 보여주고 검출 표시를 하지 않는다.
    """

    stream_id: str
    label: str
    kind: str
    topic: str
    detects: bool


def _topic(stream_id: str, default: str) -> str:
    """환경변수로 덮어쓸 수 있게 둔다.

    카메라를 어느 노트북에 붙이느냐에 따라 camera_id 가 달라질 수 있고,
    로봇 검출 노드가 준비되면 원본 대신 debug 영상을 봐야 한다. 그때마다
    코드를 고치지 않도록 한다.
      AED_HMI_STREAM_ROBOT1=/robot1/vision/debug/compressed
    """
    return os.environ.get(f"AED_HMI_STREAM_{stream_id.upper()}", default)


# 4분할 화면. 고정 웹캠 2대는 이미 검출까지 하고, 로봇 2대는 아직 원본만
# 나온다. 로봇 쪽 검출 노드가 붙으면 위 환경변수로 토픽만 바꿔 끼우면 된다.
DEFAULT_STREAMS = (
    StreamSource(
        "camera_open", "고정 웹캠 · 개방구역", "webcam",
        _topic("camera_open", vision_topic("camera_open", "debug/compressed")),
        detects=True,
    ),
    StreamSource(
        "camera_alley", "고정 웹캠 · 골목", "webcam",
        _topic("camera_alley",
               vision_topic("camera_alley", "debug/compressed")),
        detects=True,
    ),
    StreamSource(
        "robot1", "TurtleBot 1 · OAK-D", "robot",
        _topic("robot1", "/robot1/oakd/rgb/image_raw/compressed"),
        detects=False,
    ),
    StreamSource(
        "robot2", "TurtleBot 2 · OAK-D", "robot",
        _topic("robot2", "/robot2/oakd/rgb/image_raw/compressed"),
        detects=False,
    ),
)
