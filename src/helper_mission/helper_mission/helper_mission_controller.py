"""ROS 2 node scaffold for helper_mission."""

import rclpy
from rclpy.node import Node


class HelperMissionController(Node):
    """Package entry point; feature callbacks are implemented by its owner."""

    def __init__(self) -> None:
        super().__init__("helper_mission_controller")
        self.get_logger().info("helper_mission scaffold started")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HelperMissionController()
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
