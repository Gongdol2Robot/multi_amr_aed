"""Compatibility entry point for the sensor_recovery package."""

import rclpy
from rclpy.node import Node


class SensorHealthMonitor(Node):
    """Keep the legacy executable available for existing launch files."""

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
