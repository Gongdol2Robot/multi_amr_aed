"""Emit compact Nav2 command and pose traces during active navigation."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _yaw(z: float, w: float) -> float:
    """Return planar yaw in radians for a normalized quaternion."""
    return 2.0 * math.atan2(z, w)


class NavDiagnostics(Node):
    """Log input/output velocity, odometry, AMCL pose, and global path."""

    def __init__(self) -> None:
        super().__init__('nav_diagnostics')
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('active_timeout_sec', 2.0)

        self._cmd_nav: Twist | None = None
        self._cmd_out: Twist | None = None
        self._odom: Odometry | None = None
        self._amcl: PoseWithCovarianceStamped | None = None
        self._plan: Path | None = None
        self._last_cmd_nav_at = 0.0
        self._last_plan_at = 0.0
        self._previous_odom_xy: tuple[float, float] | None = None

        self.create_subscription(
            Twist, 'cmd_vel_nav', self._on_cmd_nav, qos_profile_sensor_data
        )
        self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_out, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, 'odom', self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._on_amcl,
            qos_profile_sensor_data,
        )
        self.create_subscription(Path, 'plan', self._on_plan, 10)

        period = float(self.get_parameter('log_period_sec').value)
        self.create_timer(max(period, 0.1), self._log_trace)

    def _on_cmd_nav(self, message: Twist) -> None:
        self._cmd_nav = message
        self._last_cmd_nav_at = time.monotonic()

    def _on_cmd_out(self, message: Twist) -> None:
        self._cmd_out = message

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message

    def _on_amcl(self, message: PoseWithCovarianceStamped) -> None:
        self._amcl = message

    def _on_plan(self, message: Path) -> None:
        self._plan = message
        self._last_plan_at = time.monotonic()

    @staticmethod
    def _twist_text(message: Twist | None) -> str:
        if message is None:
            return 'unavailable'
        return f'v={message.linear.x:+.3f},w={message.angular.z:+.3f}'

    @staticmethod
    def _pose_text(message: object | None) -> str:
        if message is None:
            return 'unavailable'
        pose = message.pose.pose
        yaw = _yaw(pose.orientation.z, pose.orientation.w)
        return f'x={pose.position.x:+.3f},y={pose.position.y:+.3f},yaw={yaw:+.2f}'

    def _odom_step(self) -> float | None:
        if self._odom is None:
            return None
        point = (
            self._odom.pose.pose.position.x,
            self._odom.pose.pose.position.y,
        )
        previous = self._previous_odom_xy
        self._previous_odom_xy = point
        if previous is None:
            return None
        return math.hypot(point[0] - previous[0], point[1] - previous[1])

    def _plan_text(self) -> str:
        if self._plan is None or not self._plan.poses:
            return 'unavailable'
        first = self._plan.poses[0].pose.position
        last = self._plan.poses[-1].pose.position
        return (
            f'n={len(self._plan.poses)},'
            f'start=({first.x:+.2f},{first.y:+.2f}),'
            f'end=({last.x:+.2f},{last.y:+.2f})'
        )

    def _log_trace(self) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter('active_timeout_sec').value)
        if (
            now - self._last_cmd_nav_at > timeout
            and now - self._last_plan_at > timeout
        ):
            self._previous_odom_xy = None
            return

        step = self._odom_step()
        step_text = 'unavailable' if step is None else f'{step:.3f}m'
        self.get_logger().info(
            'NAV_TRACE '
            f'cmd_nav=[{self._twist_text(self._cmd_nav)}] '
            f'cmd_out=[{self._twist_text(self._cmd_out)}] '
            f'odom=[{self._pose_text(self._odom)}] '
            f'amcl=[{self._pose_text(self._amcl)}] '
            f'odom_step={step_text} plan=[{self._plan_text()}]'
        )


def main() -> None:
    """Run the navigation diagnostics node."""
    rclpy.init()
    node = NavDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
