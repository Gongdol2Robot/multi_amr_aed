"""구독할 토픽 이름과 QoS 를 한 곳에 모은다.

토픽 이름이 코드 여기저기 흩어지면, 네임스페이스가 바뀔 때 어디를 고쳐야
하는지 알 수 없게 된다. 이름은 실제 발행자에 맞춰야 하므로, 바꿀 때는
그쪽 코드를 보고 맞춘다.

  고정 웹캠 검출 : aed_vision/vision_detector.py  (camera_id 별 절대 토픽)
  로봇 상태/임무 : mission_manager, robot_missions (/aed/... 공용 토픽)
  로봇 카메라   : turtlebot4 기본 OAK-D
  ETA 측정      : multi_robot_emergency/mission_manager.py

**이 파일은 rclpy 없이도 import 되어야 한다.** 토픽 이름과 영상 구성을
검증하는 단위 테스트에서 ROS 초기화가 필요하지 않아야 한다. 그래서 QoS는
모듈을 읽을 때가 아니라 실제로 구독할 때 만든다.
"""

import os
from dataclasses import dataclass


def state_qos():
    """상태·이벤트용. 기본은 RELIABLE 이다.

    상태 전이(MissionStatus, EmergencyEvent)는 한 번만 오므로 놓치면 그
    기록이 영영 없다. 그래서 기본을 RELIABLE 로 둔다. 지금 발행하는
    mission_manager(depth 20)와 vision_detector(depth 10) 둘 다 RELIABLE 이라
    맞는다.

    다만 터틀봇 위에서 도는 노드는 사정이 다르다. Create3 와 센서 토픽이
    전부 `qos_profile_sensor_data`(BEST_EFFORT)라, 그 위에서 상태를 내는
    노드도 같은 QoS 를 쓰기 쉽다. **발행이 BEST_EFFORT 인데 구독이
    RELIABLE 이면 ROS 2 는 연결을 아예 안 맺고 경고도 안 낸다.** 화면에는
    "ROS 수신" 이라 떠 있는데 로봇 칸만 영영 비는 모습이 된다.

    그때 코드를 고치지 않고 넘길 수 있게 환경변수로 연다.

        AED_HMI_STATE_RELIABILITY=best_effort python3 -m backend.main

    BEST_EFFORT 로 내리면 어느 발행자와도 붙는다(RELIABLE 발행자와도 붙는다).
    대신 상태 전이를 놓칠 수 있으므로, 안 붙을 때만 쓴다.
    """
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    wanted = os.environ.get("AED_HMI_STATE_RELIABILITY", "reliable").lower()
    reliability = (
        ReliabilityPolicy.BEST_EFFORT if wanted == "best_effort"
        else ReliabilityPolicy.RELIABLE
    )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=reliability,
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

# mission_manager 가 로봇마다 따로 내는 배정 지시. 목표 좌표는 여기에만
# 실려 온다. MissionStatus 에는 상태만 있고 좌표가 없어서, 이걸 안 받으면
# 화면의 목표 좌표와 도착 예상이 영영 빈다.
#
# DeliverAed action 으로 바뀌면 이 토픽은 사라지고 goal 이 같은 값을 싣는다.


def assignment_topic(robot_id: str) -> str:
    return f"/{robot_id}/mission_assignment"


ROBOT_IDS = ("robot1", "robot2")

# 예상과 실제를 함께 재는 쪽(multi_robot_emergency)이 내는 토픽.
# 로봇별 Float32 도 있지만 그건 안 받는다. 어느 요청의 값인지가 안 실려
# 있어서, 두 요청이 겹치면 어느 쪽 값인지 가릴 수 없다. 요청 id 가 들어
# 있는 result 하나만 받는다.
ETA_RESULT_TOPIC = "/emergency/eta/result"


def predicted_eta_topic(robot_id: str) -> str:
    return f"/emergency/eta/predicted/{robot_id}"


def lidar_state_topic(robot_id: str) -> str:
    """sensor_recovery/lidar_watchdog_node.py가 내는 4단계 상태."""
    return f"/{robot_id}/lidar_state"


def fallback_state_topic(robot_id: str) -> str:
    """fallback_path_follower.py의 Depth/cmd_vel 제어 상태."""
    return f"/{robot_id}/fallback_state"


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
    compressed: bool = True


def _topic(stream_id: str, default: str) -> str:
    """환경변수로 덮어쓸 수 있게 둔다.

    카메라를 어느 노트북에 붙이느냐에 따라 camera_id 가 달라질 수 있고,
    로봇 검출 노드가 준비되면 원본 대신 debug 영상을 봐야 한다. 그때마다
    코드를 고치지 않도록 한다.
      AED_HMI_STREAM_ROBOT1=/robot1/vision/debug/compressed
    """
    return os.environ.get(f"AED_HMI_STREAM_{stream_id.upper()}", default)


# 4분할 화면. 모든 타일은 vision_detector가 압축한 debug 영상을 받는다.
# 로봇 raw preview를 HMI와 vision이 각각 원격 구독하면 같은 비압축 프레임이
# 두 번 전송되므로, HMI는 vision의 압축 결과만 받아 무선 중복을 없앤다.
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
        _topic("robot1", "/robot1/vision/debug/compressed"),
        detects=True,
    ),
    StreamSource(
        "robot2", "TurtleBot 2 · OAK-D", "robot",
        _topic("robot2", "/robot2/vision/debug/compressed"),
        detects=True,
    ),
)
