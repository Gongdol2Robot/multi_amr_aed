"""ROS 2 node scaffold for amr_recovery."""

import rclpy
from rclpy.node import Node


class RecoveryManager(Node):
    """Package entry point; feature callbacks are implemented by its owner."""

    def __init__(self) -> None:
        super().__init__("recovery_manager")
        self.get_logger().info("amr_recovery scaffold started")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryManager()
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
