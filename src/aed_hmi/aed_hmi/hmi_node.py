"""ROS 2 node scaffold for aed_hmi."""

import rclpy
from rclpy.node import Node


class HmiNode(Node):
    """Package entry point; feature callbacks are implemented by its owner."""

    def __init__(self) -> None:
        super().__init__("hmi_node")
        self.get_logger().info("aed_hmi scaffold started")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HmiNode()
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
