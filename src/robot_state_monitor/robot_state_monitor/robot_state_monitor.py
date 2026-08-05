"""ROS 2 node scaffold for robot_state_monitor."""

import rclpy
from rclpy.node import Node


class RobotStateMonitor(Node):
    """Package entry point; feature callbacks are implemented by its owner."""

    def __init__(self) -> None:
        super().__init__("robot_state_monitor")
        self.get_logger().info("robot_state_monitor scaffold started")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStateMonitor()
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
