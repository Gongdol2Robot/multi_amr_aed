"""실시간 카메라 영상에 사용하는 공통 ROS 2 QoS."""

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
