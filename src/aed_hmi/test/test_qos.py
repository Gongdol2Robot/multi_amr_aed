from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from backend.ros import topics


def test_hmi_state_subscriptions_are_best_effort() -> None:
    qos = topics.state_qos()

    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_hmi_latched_subscriptions_keep_best_effort_reliability() -> None:
    qos = topics.latched_qos()

    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_operator_event_publisher_remains_reliable() -> None:
    qos = topics.operator_event_qos()

    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE
