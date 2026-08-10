"""실시간 영상 토픽에 공통으로 사용하는 ROS 2 전송 정책.

영상은 늦게 도착한 과거 프레임을 완벽히 전달하는 것보다 현재 장면을 빨리 보는
것이 중요하다. 따라서 큐는 한 장만 두고, 재전송을 기다리지 않는 BEST_EFFORT를
사용한다. 명령·응급 이벤트처럼 유실되면 안 되는 토픽에는 이 QoS를 쓰지 않는다.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # 큐에 프레임을 쌓지 않는다 — 처리가 늦어지면 최신 프레임으로 덮어쓴다.
    reliability=ReliabilityPolicy.BEST_EFFORT,  # 재전송 대신 항상 최신 프레임 우선.
    durability=DurabilityPolicy.VOLATILE,  # 늦게 붙는 구독자에게 과거 프레임을 주지 않는다.
)
