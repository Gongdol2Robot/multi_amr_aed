"""Legacy compatibility entry point; not part of the current runtime.

[CODE REVIEW]
초기 센서 상태 모니터 구상의 호환 대안으로 실행 이름만 남겨둔 노드다. 현재는
구독·판정·복구 기능이 없으며 운영 경로에서는 실행하지 않는다. 실제 기능은
``lidar_watchdog``과 ``lidar_fallback_controller``가 담당한다.
"""

import rclpy
from rclpy.node import Node


class SensorHealthMonitor(Node):
    """Keep the unused legacy executable available for compatibility."""

    def __init__(self) -> None:
        super().__init__("sensor_health_monitor")
        self.get_logger().info(
            "sensor_recovery compatibility entry point started; operational "
            "recovery runs in lidar_watchdog/lidar_fallback_controller"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorHealthMonitor()
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
