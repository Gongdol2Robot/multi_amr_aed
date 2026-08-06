"""Reliably activate Nav2 lifecycle nodes on lossy robot Wi-Fi."""

from __future__ import annotations

import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


NAVIGATION_NODES = (
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
)


class NavigationInitializer(Node):
    """Configure and activate every Nav2 node with state-based retries."""

    def __init__(self) -> None:
        super().__init__('navigation_initializer')
        self.declare_parameter('startup_delay_sec', 2.0)
        self.declare_parameter('lifecycle_timeout_sec', 300.0)
        self.declare_parameter('service_call_timeout_sec', 3.0)

        self.get_state_clients = {
            name: self.create_client(GetState, f'{name}/get_state')
            for name in NAVIGATION_NODES
        }
        self.change_state_clients = {
            name: self.create_client(ChangeState, f'{name}/change_state')
            for name in NAVIGATION_NODES
        }

    def _spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _call(self, client: object, request: object) -> object | None:
        future = client.call_async(request)
        timeout = float(self.get_parameter('service_call_timeout_sec').value)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as error:
            self.get_logger().warning(f'Lifecycle service error: {error}')
            return None

    def _get_state(self, name: str) -> int | None:
        response = self._call(
            self.get_state_clients[name],
            GetState.Request(),
        )
        if response is None:
            return None
        return int(response.current_state.id)

    def _change_state(self, name: str, transition: int) -> bool:
        request = ChangeState.Request()
        request.transition.id = transition
        response = self._call(self.change_state_clients[name], request)
        return response is not None and bool(response.success)

    def ensure_active(self, name: str) -> None:
        timeout = float(self.get_parameter('lifecycle_timeout_sec').value)
        deadline = time.monotonic() + timeout
        get_client = self.get_state_clients[name]
        change_client = self.change_state_clients[name]
        last_state = None

        self.get_logger().info(f'Waiting for {name} lifecycle services...')
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                get_client.wait_for_service(timeout_sec=0.25)
                and change_client.wait_for_service(timeout_sec=0.25)
            ):
                break
        else:
            raise RuntimeError(f'{name} lifecycle services are unavailable')

        while rclpy.ok() and time.monotonic() < deadline:
            state = self._get_state(name)
            if state is not None and state != last_state:
                self.get_logger().info(f'{name} lifecycle state ID: {state}')
                last_state = state

            if state == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f'{name} is active.')
                return
            if state == State.PRIMARY_STATE_UNCONFIGURED:
                self._change_state(name, Transition.TRANSITION_CONFIGURE)
            elif state == State.PRIMARY_STATE_INACTIVE:
                self._change_state(name, Transition.TRANSITION_ACTIVATE)

            # A lost response is harmless: read the real state and retry.
            self._spin_for(0.25)

        raise RuntimeError(f'{name} did not become active')

    def run(self) -> None:
        delay = float(self.get_parameter('startup_delay_sec').value)
        self.get_logger().info(
            f'Waiting {delay:.1f}s for Nav2 processes to start...'
        )
        self._spin_for(delay)
        for name in NAVIGATION_NODES:
            self.ensure_active(name)
        self.get_logger().info('All Nav2 lifecycle nodes are active.')


def main() -> None:
    """Run the reliable Nav2 lifecycle startup sequence."""
    rclpy.init()
    node = NavigationInitializer()
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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
