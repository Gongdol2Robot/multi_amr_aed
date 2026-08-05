"""ROS 2 node scaffold for event_logger."""

import rclpy
from rclpy.node import Node


class EventLogger(Node):
    """Package entry point; feature callbacks are implemented by its owner."""

    def __init__(self) -> None:
        super().__init__("event_logger")
        self.get_logger().info("event_logger scaffold started")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EventLogger()
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
