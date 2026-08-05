"""ROS 2 node scaffold for sensor_recovery."""

import rclpy
from rclpy.node import Node


class SensorHealthMonitor(Node):
    """Package entry point; feature callbacks are implemented by its owner."""

    def __init__(self) -> None:
        super().__init__("sensor_health_monitor")
        self.get_logger().info("sensor_recovery scaffold started")


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
