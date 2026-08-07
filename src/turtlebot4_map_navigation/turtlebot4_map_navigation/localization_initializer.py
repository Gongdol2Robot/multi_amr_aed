"""Gate Nav2 startup until TurtleBot 4 localization is actually ready."""

from __future__ import annotations

import math
import json
from pathlib import Path
import re
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import DockStatus
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
        self.declare_parameter('lifecycle_fallback_delay_sec', 8.0)
        self.declare_parameter('sensor_timeout_sec', 60.0)
        self.declare_parameter('robot_tf_timeout_sec', 30.0)
        self.declare_parameter('initial_pose_timeout_sec', 300.0)
        self.declare_parameter('tf_timeout_sec', 30.0)

        self.scan_received = False
        self.latest_scan: LaserScan | None = None
        self.odometry_received = False
        self.odometry_pose = (0.0, 0.0, 0.0)
        self.amcl_pose_received = False
        self.dock_status_received = False
        self.is_docked = False

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
            DockStatus,
            'dock_status',
            self._dock_status_callback,
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

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan_received = True
        self.latest_scan = message

    def _odometry_callback(self, message: Odometry) -> None:
        self.odometry_received = True
        pose = message.pose.pose
        quaternion = pose.orientation
        yaw = math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )
        self.odometry_pose = (pose.position.x, pose.position.y, yaw)

    def _dock_status_callback(self, message: DockStatus) -> None:
        self.dock_status_received = True
        self.is_docked = bool(message.is_docked)

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
        """Wait until the standard lifecycle manager activates a node."""
        timeout = float(self.get_parameter('lifecycle_timeout_sec').value)
        deadline = time.monotonic() + timeout
        fallback_at = time.monotonic() + float(
            self.get_parameter('lifecycle_fallback_delay_sec').value
        )
        service = self.get_state_clients[node_name]
        self.get_logger().info(f'Waiting for {node_name} lifecycle services...')
        while rclpy.ok() and time.monotonic() < deadline:
            if service.wait_for_service(timeout_sec=0.2):
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
            if time.monotonic() >= fallback_at:
                change_client = self.change_state_clients[node_name]
                if change_client.wait_for_service(timeout_sec=0.2):
                    if state == State.PRIMARY_STATE_UNCONFIGURED:
                        self.get_logger().warning(
                            f'{node_name}: lifecycle manager stalled; '
                            'retrying configure'
                        )
                        self._change_lifecycle_state(
                            node_name,
                            Transition.TRANSITION_CONFIGURE,
                        )
                    elif state == State.PRIMARY_STATE_INACTIVE:
                        self.get_logger().warning(
                            f'{node_name}: lifecycle manager stalled; '
                            'retrying activate'
                        )
                        self._change_lifecycle_state(
                            node_name,
                            Transition.TRANSITION_ACTIVATE,
                        )
            self._spin_for(0.25)
        raise RuntimeError(f'{node_name} did not become active')

    def wait_for_robot_data(self) -> None:
        """Wait for robot sensors and dock state before accepting a pose."""
        timeout = float(self.get_parameter('sensor_timeout_sec').value)
        deadline = time.monotonic() + timeout
        self.get_logger().info('Waiting for /scan and /odom...')
        while rclpy.ok() and time.monotonic() < deadline:
            dock_ready = (
                not bool(self.get_parameter('use_dock_initial_pose').value)
                or self.dock_status_received
            )
            if self.scan_received and self.odometry_received and dock_ready:
                self.get_logger().info(
                    'Scan, odometry, and dock state are ready.'
                )
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = []
        if not self.scan_received:
            missing.append('scan')
        if not self.odometry_received:
            missing.append('odom')
        if (
            bool(self.get_parameter('use_dock_initial_pose').value)
            and not self.dock_status_received
        ):
            missing.append('dock_status')
        raise RuntimeError(f'Missing robot data: {", ".join(missing)}')

    def wait_for_robot_transforms(self) -> None:
        """Wait for the robot-local TF chain before deriving an initial pose."""
        timeout = float(self.get_parameter('robot_tf_timeout_sec').value)
        deadline = time.monotonic() + timeout
        scan_frame = (
            self.latest_scan.header.frame_id
            if self.latest_scan is not None
            else 'rplidar_link'
        )
        required = (
            ('odom', 'base_link'),
            ('base_link', scan_frame),
        )
        self.get_logger().info(
            'Waiting for robot TF: odom -> base_link -> '
            f'{scan_frame}...'
        )
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(
                self.tf_buffer.can_transform(
                    target,
                    source,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
                for target, source in required
            ):
                self.get_logger().info('Robot-local TF is ready.')
                return
        missing = [
            f'{target} <- {source}'
            for target, source in required
            if not self.tf_buffer.can_transform(target, source, Time())
        ]
        raise RuntimeError(
            f'Robot-local TF did not become available: {", ".join(missing)}'
        )

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
            if self.is_docked:
                pose_label = 'Dock pose'
            else:
                # Create 3 odometry starts at the powered-on Dock pose. Compose
                # that relative motion with the Dock's shared-map transform so
                # localization also starts correctly after driving/turning.
                odom_x, odom_y, odom_yaw = self.odometry_pose
                dock_x, dock_y, dock_yaw = x, y, yaw
                x = (
                    dock_x
                    + math.cos(dock_yaw) * odom_x
                    - math.sin(dock_yaw) * odom_y
                )
                y = (
                    dock_y
                    + math.sin(dock_yaw) * odom_x
                    + math.cos(dock_yaw) * odom_y
                )
                yaw = math.atan2(
                    math.sin(dock_yaw + odom_yaw),
                    math.cos(dock_yaw + odom_yaw),
                )
                pose_label = 'odom-adjusted undocked pose'
                x, y, yaw = self._refine_pose_with_scan((x, y, yaw))
            self.get_logger().info(
                f'Loaded {robot_id} shared-map {pose_label}: '
                f'x={x:.3f}, y={y:.3f}, '
                f'yaw={math.degrees(yaw):.1f} deg'
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

    def _refine_pose_with_scan(
        self, pose: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Locally align an undocked odom estimate with occupied map cells."""
        scan = self.latest_scan
        if scan is None:
            return pose
        try:
            map_yaml = Path(str(self.get_parameter('map_yaml').value))
            with map_yaml.open('r', encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream) or {}
            image_path = map_yaml.parent / str(metadata['image'])
            raw = image_path.read_bytes()
            header = re.match(
                rb'P5\s+(?:#[^\n]*\n)?\s*'
                rb'(\d+)\s+(\d+)\s+(\d+)\s',
                raw,
            )
            if header is None:
                raise ValueError('unsupported PGM header')
            width, height, _ = map(int, header.groups())
            pixels = raw[header.end():]
            resolution = float(metadata['resolution'])
            origin_x = float(metadata['origin'][0])
            origin_y = float(metadata['origin'][1])
            occupied = {
                (cell_x, height - 1 - image_y)
                for image_y in range(height)
                for cell_x in range(width)
                if pixels[image_y * width + cell_x] <= 50
            }
            near_occupied = {
                (cell_x + offset_x, cell_y + offset_y)
                for cell_x, cell_y in occupied
                for offset_x in range(-2, 3)
                for offset_y in range(-2, 3)
            }
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                scan.header.frame_id,
                Time(),
                timeout=Duration(seconds=1.0),
            ).transform
        except Exception as error:
            self.get_logger().warning(
                f'Scan-map initial pose refinement unavailable: {error}'
            )
            return pose

        rotation = transform.rotation
        sensor_yaw = math.atan2(
            2.0 * (
                rotation.w * rotation.z + rotation.x * rotation.y
            ),
            1.0 - 2.0 * (
                rotation.y * rotation.y + rotation.z * rotation.z
            ),
        )
        sensor_cos = math.cos(sensor_yaw)
        sensor_sin = math.sin(sensor_yaw)
        points = []
        for index in range(0, len(scan.ranges), 4):
            distance = scan.ranges[index]
            if not (
                math.isfinite(distance)
                and scan.range_min < distance < scan.range_max
            ):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            points.append(
                (
                    transform.translation.x
                    + sensor_cos * laser_x
                    - sensor_sin * laser_y,
                    transform.translation.y
                    + sensor_sin * laser_x
                    + sensor_cos * laser_y,
                )
            )
        if not points:
            return pose

        def score(candidate: tuple[float, float, float]) -> float:
            candidate_x, candidate_y, candidate_yaw = candidate
            cosine = math.cos(candidate_yaw)
            sine = math.sin(candidate_yaw)
            hits = 0
            for point_x, point_y in points:
                world_x = candidate_x + cosine * point_x - sine * point_y
                world_y = candidate_y + sine * point_x + cosine * point_y
                cell = (
                    int((world_x - origin_x) / resolution),
                    int((world_y - origin_y) / resolution),
                )
                hits += cell in near_occupied
            return hits / len(points)

        original_score = score(pose)
        best_pose = pose
        best_score = original_score
        best_adjustment = 0.0
        for x_step in range(-5, 6):
            for y_step in range(-5, 6):
                for yaw_step in range(-10, 11):
                    candidate = (
                        pose[0] + x_step * 0.05,
                        pose[1] + y_step * 0.05,
                        pose[2] + math.radians(yaw_step * 2.0),
                    )
                    candidate_score = score(candidate)
                    adjustment = (
                        (x_step * 0.05) ** 2
                        + (y_step * 0.05) ** 2
                        + (yaw_step * 0.01) ** 2
                    )
                    if (
                        candidate_score > best_score
                        or (
                            candidate_score == best_score
                            and adjustment < best_adjustment
                        )
                    ):
                        best_pose = candidate
                        best_score = candidate_score
                        best_adjustment = adjustment

        if best_score >= 0.80 and best_score >= original_score + 0.05:
            self.get_logger().info(
                'Refined undocked pose with scan-map fit: '
                f'{original_score * 100:.1f}% -> {best_score * 100:.1f}%'
            )
            return best_pose
        self.get_logger().info(
            f'Keeping odom initial pose; scan-map fit={original_score * 100:.1f}%'
        )
        return pose

    def _publish_initial_pose(self, pose: tuple[float, float, float]) -> None:
        x, y, yaw = pose
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        # Zero requests the latest odom transform. A wall-clock stamp can be a
        # few milliseconds newer than wireless TF and AMCL rejects it.
        message.header.stamp = Time().to_msg()
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
        self.wait_for_robot_transforms()
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
