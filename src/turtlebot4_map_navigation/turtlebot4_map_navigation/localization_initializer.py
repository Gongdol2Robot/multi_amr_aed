"""Gate Nav2 startup until TurtleBot 4 localization is actually ready."""

from __future__ import annotations

import math
import json
from pathlib import Path
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class LocalizationInitializer(Node):
    """Activate localization and exit only after the map TF is usable."""

    def __init__(self) -> None:
        super().__init__('localization_initializer')
        self.declare_parameter('auto_initial_pose', True)
        self.declare_parameter('use_saved_initial_pose', True)
        self.declare_parameter('use_dock_initial_pose', True)
        self.declare_parameter('dock_poses_yaml', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw_deg', 0.0)
        self.declare_parameter('lifecycle_timeout_sec', 60.0)
        self.declare_parameter('sensor_timeout_sec', 60.0)
        self.declare_parameter('initial_pose_timeout_sec', 300.0)
        self.declare_parameter('tf_timeout_sec', 30.0)

        self.scan_received = False
        self.odometry_received = False
        self.amcl_pose_received = False

        self.create_subscription(
            LaserScan,
            'scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            'odom',
            self._odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._amcl_pose_callback,
            10,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_state_clients = {
            name: self.create_client(GetState, f'{name}/get_state')
            for name in ('map_server', 'amcl')
        }
        self.change_state_clients = {
            name: self.create_client(ChangeState, f'{name}/change_state')
            for name in ('map_server', 'amcl')
        }

    def _scan_callback(self, _message: LaserScan) -> None:
        self.scan_received = True

    def _odometry_callback(self, _message: Odometry) -> None:
        self.odometry_received = True

    def _amcl_pose_callback(
        self,
        _message: PoseWithCovarianceStamped,
    ) -> None:
        self.amcl_pose_received = True

    def _spin_for(self, seconds: float) -> None:
        end_time = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=min(0.1, seconds))

    def _call_service(self, client: object, request: object) -> object | None:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as error:
            self.get_logger().warning(f'Lifecycle service failed: {error}')
            return None

    def _get_lifecycle_state(self, node_name: str) -> int | None:
        response = self._call_service(
            self.get_state_clients[node_name],
            GetState.Request(),
        )
        return None if response is None else int(response.current_state.id)

    def _change_lifecycle_state(
        self,
        node_name: str,
        transition_id: int,
    ) -> bool:
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self._call_service(
            self.change_state_clients[node_name],
            request,
        )
        return response is not None and bool(response.success)

    def ensure_lifecycle_active(self, node_name: str) -> None:
        """Wait for or recover a localization lifecycle node to active."""
        timeout = float(self.get_parameter('lifecycle_timeout_sec').value)
        deadline = time.monotonic() + timeout
        services = (
            self.get_state_clients[node_name],
            self.change_state_clients[node_name],
        )
        self.get_logger().info(f'Waiting for {node_name} lifecycle services...')
        while rclpy.ok() and time.monotonic() < deadline:
            if all(client.wait_for_service(timeout_sec=0.2) for client in services):
                break
        else:
            raise RuntimeError(f'{node_name} lifecycle services are unavailable')

        last_state = None
        while rclpy.ok() and time.monotonic() < deadline:
            state = self._get_lifecycle_state(node_name)
            if state != last_state and state is not None:
                self.get_logger().info(
                    f'{node_name} lifecycle state ID: {state}'
                )
                last_state = state
            if state == State.PRIMARY_STATE_ACTIVE:
                return
            if state == State.PRIMARY_STATE_UNCONFIGURED:
                self._change_lifecycle_state(
                    node_name,
                    Transition.TRANSITION_CONFIGURE,
                )
            elif state == State.PRIMARY_STATE_INACTIVE:
                self._change_lifecycle_state(
                    node_name,
                    Transition.TRANSITION_ACTIVATE,
                )
            self._spin_for(0.25)
        raise RuntimeError(f'{node_name} did not become active')

    def wait_for_robot_data(self) -> None:
        """Wait for both scan and odometry before accepting a pose."""
        timeout = float(self.get_parameter('sensor_timeout_sec').value)
        deadline = time.monotonic() + timeout
        self.get_logger().info('Waiting for /scan and /odom...')
        while rclpy.ok() and time.monotonic() < deadline:
            if self.scan_received and self.odometry_received:
                self.get_logger().info('Scan and odometry are ready.')
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = []
        if not self.scan_received:
            missing.append('scan')
        if not self.odometry_received:
            missing.append('odom')
        raise RuntimeError(f'Missing robot data: {", ".join(missing)}')

    def _load_initial_pose(self) -> tuple[float, float, float]:
        use_dock_pose = bool(
            self.get_parameter('use_dock_initial_pose').value
        )
        if use_dock_pose:
            config_path = Path(
                str(self.get_parameter('dock_poses_yaml').value)
            )
            robot_id = self.get_namespace().strip('/').split('/')[-1]
            try:
                with config_path.open('r', encoding='utf-8') as stream:
                    config = yaml.safe_load(stream) or {}
                dock_pose = config['robots'][robot_id]
                x = float(dock_pose['x'])
                y = float(dock_pose['y'])
                yaw = math.radians(float(dock_pose['yaw_deg']))
            except (OSError, KeyError, TypeError, ValueError) as error:
                raise RuntimeError(
                    f'Cannot load {robot_id} Dock pose from '
                    f'{config_path}: {error}'
                ) from error
            self.get_logger().info(
                f'Loaded {robot_id} shared-map Dock pose: '
                f'x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg'
            )
            return x, y, yaw

        use_saved_pose = bool(
            self.get_parameter('use_saved_initial_pose').value
        )
        if use_saved_pose:
            map_yaml = Path(str(self.get_parameter('map_yaml').value))
            pose_file = map_yaml.with_suffix('.pose.yaml')
            try:
                with pose_file.open('r', encoding='utf-8') as stream:
                    saved_pose = json.load(stream)
                x = float(saved_pose['x'])
                y = float(saved_pose['y'])
                yaw = float(saved_pose['yaw'])
            except (OSError, KeyError, TypeError, ValueError) as error:
                raise RuntimeError(
                    f'Cannot load saved initial pose {pose_file}: {error}'
                ) from error
            self.get_logger().info(
                'Loaded saved initial pose: '
                f'x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg'
            )
            return x, y, yaw

        x = float(self.get_parameter('initial_x').value)
        y = float(self.get_parameter('initial_y').value)
        yaw = math.radians(
            float(self.get_parameter('initial_yaw_deg').value)
        )
        return x, y, yaw

    def _publish_initial_pose(self, pose: tuple[float, float, float]) -> None:
        x, y, yaw = pose
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self.initial_pose_publisher.publish(message)

    def wait_for_initial_pose(self) -> None:
        """Wait for RViz or repeatedly publish the configured initial pose."""
        auto_pose = bool(self.get_parameter('auto_initial_pose').value)
        timeout = float(self.get_parameter('initial_pose_timeout_sec').value)
        deadline = time.monotonic() + timeout
        last_publish = 0.0
        initial_pose = self._load_initial_pose() if auto_pose else None
        if auto_pose:
            self.get_logger().info('Publishing the configured initial pose.')
        else:
            self.get_logger().info(
                'Use RViz 2D Pose Estimate to set the robot pose and heading.'
            )

        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if auto_pose and now - last_publish >= 1.0:
                self._publish_initial_pose(initial_pose)
                last_publish = now
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.amcl_pose_received:
                self.get_logger().info('AMCL initial pose is ready.')
                return
        raise RuntimeError('AMCL did not produce an initial pose')

    def wait_for_map_transform(self) -> None:
        """Wait for the complete map to base_link transform chain."""
        timeout = float(self.get_parameter('tf_timeout_sec').value)
        deadline = time.monotonic() + timeout
        self.get_logger().info('Waiting for map -> base_link TF...')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tf_buffer.can_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.1),
            ):
                self.get_logger().info(
                    'Localization is ready; starting Nav2 now.'
                )
                return
        raise RuntimeError('map -> base_link TF did not become available')

    def run(self) -> None:
        """Run the complete localization readiness sequence."""
        self.ensure_lifecycle_active('map_server')
        self.ensure_lifecycle_active('amcl')
        self.wait_for_robot_data()
        self.wait_for_initial_pose()
        self.wait_for_map_transform()


def main() -> None:
    """Run the localization startup gate."""
    rclpy.init()
    node = LocalizationInitializer()
    try:
        while rclpy.ok():
            try:
                node.run()
                break
            except RuntimeError as error:
                node.get_logger().error(f'{error}; retrying in 1 second')
                node._spin_for(1.0)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().error(f'Unexpected readiness error: {error}')
        while rclpy.ok():
            node._spin_for(1.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
