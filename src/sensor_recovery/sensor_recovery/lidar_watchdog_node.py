"""Watch each robot's /scan liveness independently and publish LiDAR status."""

from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from sensor_recovery.lidar_state_machine import (
    LidarMonitor,
    LidarState,
    LidarWatchdogConfig,
)

# Latched: a subscriber (ros2 topic echo, Mission Manager) that connects
# after the last transition must still see the current state immediately
# instead of waiting for the next one.
_STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@dataclass
class RobotWatch:
    monitor: LidarMonitor
    alive_publisher: Publisher
    state_publisher: Publisher


class LidarWatchdogNode(Node):
    """Detect per-robot LiDAR timeout/recovery and publish alive/state topics."""

    def __init__(self) -> None:
        super().__init__("lidar_watchdog_node")

        # 5.0: 이 랩 환경의 디스커버리 서버 핑이 가끔 불안정해서 실제
        # LiDAR는 멀쩡한데도 1초 넘게 /scan이 안 들어올 때가 있다(네트워크
        # 지터, 하드웨어 문제 아님) — 너무 타이트하면 그런 순간마다 진짜
        # 고장처럼 FAULT/RECOVERING을 반복하게 됨(2026-08-07 robot2 실측).
        self.declare_parameter("scan_timeout_sec", 5.0)
        self.declare_parameter("startup_grace_sec", 3.0)
        self.declare_parameter("recovery_duration_sec", 3.0)
        self.declare_parameter("watchdog_period_sec", 0.1)
        self.declare_parameter("status_publish_period_sec", 1.0)
        self.declare_parameter("robot_names", ["robot1", "robot2"])
        self.declare_parameter("scan_topic_suffix", "scan")

        self.config = LidarWatchdogConfig(
            scan_timeout_sec=self._positive_param("scan_timeout_sec", 5.0),
            startup_grace_sec=self._positive_param("startup_grace_sec", 3.0),
            recovery_duration_sec=self._positive_param("recovery_duration_sec", 3.0),
        )
        watchdog_period = self._positive_param("watchdog_period_sec", 0.1)
        status_publish_period = self._positive_param("status_publish_period_sec", 1.0)

        robot_names = list(self.get_parameter("robot_names").value)
        if not robot_names:
            raise ValueError("robot_names parameter must not be empty")
        scan_suffix = str(self.get_parameter("scan_topic_suffix").value)

        now = self._now_sec()
        self._robots: Dict[str, RobotWatch] = {}
        for robot_name in robot_names:
            monitor = LidarMonitor(config=self.config, start_time=now)
            alive_publisher = self.create_publisher(
                Bool, f"/{robot_name}/lidar_alive", _STATUS_QOS
            )
            state_publisher = self.create_publisher(
                String, f"/{robot_name}/lidar_state", _STATUS_QOS
            )
            # Bind robot_name by default argument so each subscription callback
            # keeps its own robot_name instead of sharing the loop variable.
            self.create_subscription(
                LaserScan,
                f"/{robot_name}/{scan_suffix}",
                lambda msg, robot_name=robot_name: self._on_scan(robot_name, msg),
                qos_profile_sensor_data,
            )
            self._robots[robot_name] = RobotWatch(
                monitor, alive_publisher, state_publisher
            )
            self._publish_state(robot_name, monitor.state)

        self._timer = self.create_timer(watchdog_period, self._on_timer)
        self._status_timer = self.create_timer(
            status_publish_period, self._publish_all_states
        )

    def _positive_param(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().error(
                f"Parameter '{name}'={value} must be positive; using default {default}"
            )
            return default
        return value

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, robot_name: str, _msg: LaserScan) -> None:
        watch = self._robots[robot_name]
        previous = watch.monitor.state
        new_state = watch.monitor.on_scan_received(self._now_sec())
        if new_state is not None:
            self._handle_transition(robot_name, previous, new_state)

    def _on_timer(self) -> None:
        now = self._now_sec()
        for robot_name, watch in self._robots.items():
            previous = watch.monitor.state
            new_state = watch.monitor.on_tick(now)
            if new_state is not None:
                self._handle_transition(robot_name, previous, new_state)

    def _handle_transition(
        self, robot_name: str, previous: LidarState, new_state: LidarState
    ) -> None:
        if new_state == LidarState.FAULT:
            self.get_logger().warning(f"[{robot_name}] LiDAR timeout detected")
        elif new_state == LidarState.RECOVERING:
            self.get_logger().info(f"[{robot_name}] LiDAR data received again")
        elif new_state == LidarState.ALIVE and previous == LidarState.RECOVERING:
            self.get_logger().info(f"[{robot_name}] LiDAR recovery confirmed")

        self.get_logger().info(
            f"[{robot_name}] LiDAR state: {previous.value} -> {new_state.value}"
        )

        if new_state == LidarState.FAULT:
            self.handle_lidar_fault(robot_name)
        elif new_state == LidarState.ALIVE and previous == LidarState.RECOVERING:
            self.handle_lidar_recovery(robot_name)

        self._publish_state(robot_name, new_state)

    def _publish_state(self, robot_name: str, state: LidarState) -> None:
        watch = self._robots[robot_name]
        watch.alive_publisher.publish(Bool(data=state == LidarState.ALIVE))
        watch.state_publisher.publish(String(data=state.value))

    def _publish_all_states(self) -> None:
        """Heartbeat: re-publish the current state even without a transition."""
        for robot_name, watch in self._robots.items():
            self._publish_state(robot_name, watch.monitor.state)

    def handle_lidar_fault(self, robot_name: str) -> None:
        """Extension point: Nav2 goal cancel / Mission Manager fault report."""

    def handle_lidar_recovery(self, robot_name: str) -> None:
        """Extension point: Mission Manager recovery report."""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
