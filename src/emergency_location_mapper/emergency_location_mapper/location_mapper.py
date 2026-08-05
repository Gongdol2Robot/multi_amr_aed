"""ROS 2 node scaffold for emergency_location_mapper."""

import rclpy
from rclpy.node import Node


class LocationMapper(Node):
    """Package entry point; feature callbacks are implemented by its owner."""

    def __init__(self) -> None:
        super().__init__("location_mapper")
        self.get_logger().info("emergency_location_mapper scaffold started")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocationMapper()
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
