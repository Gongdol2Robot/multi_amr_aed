#!/usr/bin/env python3
"""기동 전 단계별 점검. 처음 실패하는 곳에서 멈춘다.

라즈베리파이나 DDS 가 이상할 때 어느 단계에서 깨졌는지 바로 알기 위한 것이다.
증상만 보고 추측하지 않는다.

    python3 tools/preflight.py
    python3 tools/preflight.py --localization  # AMCL 까지 점검
    python3 tools/preflight.py --nav      # Nav2 까지 점검
    python3 tools/preflight.py --detect   # 검출 노드까지 점검

`ros2 topic list` 는 믿지 않는다. 실제로 구독해서 메시지가 오는지로 판단한다.
"""

import argparse
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

OK = "\033[32mOK\033[0m"
NG = "\033[31m실패\033[0m"

failures = []


def report(stage: str, passed: bool, detail: str = "") -> bool:
    print(f"[{OK if passed else NG}] {stage}" + (f" — {detail}" if detail else ""))
    if not passed:
        failures.append(stage)
    return passed


def check_env() -> bool:
    """discovery server 설정. 이게 없으면 /robot2 가 통째로 안 보인다."""
    server = os.environ.get("ROS_DISCOVERY_SERVER", "")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not server:
        return report(
            "discovery server", False,
            "ROS_DISCOVERY_SERVER 없음 — "
            "source /etc/turtlebot4_discovery/setup.bash",
        )
    return report("discovery server", True, f"{server} (domain {domain or '기본'})")


def check_ping(host: str) -> bool:
    done = subprocess.run(
        ["ping", "-c", "2", "-W", "2", host],
        capture_output=True, text=True,
    )
    return report("로봇 네트워크", done.returncode == 0, host)


def wait_for(node: Node, topic: str, message_type, timeout: float = 8.0):
    """실제로 한 건 받아본다. topic list 결과는 믿지 않는다."""
    got = {}
    node.create_subscription(
        message_type, topic, lambda m: got.setdefault("m", m),
        qos_profile_sensor_data,
    )
    for _ in range(int(timeout / 0.1)):
        rclpy.spin_once(node, timeout_sec=0.1)
        if got:
            return got["m"]
    return None


def check_topics(node: Node, namespace: str) -> bool:
    from irobot_create_msgs.msg import DockStatus
    from sensor_msgs.msg import LaserScan

    prefix = f"/{namespace}"
    scan = wait_for(node, f"{prefix}/scan", LaserScan)
    ok = report(
        f"라이다 {prefix}/scan", scan is not None,
        f"{len(scan.ranges)}점" if scan else "메시지 없음 — 파이 쪽 turtlebot4.service 확인",
    )
    dock = wait_for(node, f"{prefix}/dock_status", DockStatus)
    report(
        f"Create3 {prefix}/dock_status", dock is not None,
        f"is_docked={dock.is_docked}" if dock else "메시지 없음 — 베이스 통신 확인",
    )
    return ok


def check_camera(node: Node, namespace: str) -> bool:
    from sensor_msgs.msg import CompressedImage

    topic = f"/{namespace}/oakd/rgb/image_raw/compressed"
    rgb = wait_for(node, topic, CompressedImage)
    return report(
        f"OAK-D RGB {topic}", rgb is not None,
        f"{len(rgb.data)} bytes" if rgb else "메시지 없음",
    )


def check_tf(node: Node) -> bool:
    """tf 는 네임스페이스 아래에 있다. 리매핑 없이는 절대 안 잡힌다."""
    import rclpy.time
    from tf2_ros import Buffer, TransformListener

    buffer = Buffer()
    TransformListener(buffer, node)
    for _ in range(80):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            p = tf.transform.translation
            return report("tf map→base_link", True, f"({p.x:.2f}, {p.y:.2f})")
        except Exception:
            continue
    return report(
        "tf map→base_link", False,
        "변환 없음 — localization 이 떴는지, tf 리매핑을 넘겼는지 확인",
    )


def check_lifecycle(names) -> bool:
    """inactive 는 아직 올라오는 중일 수 있다. 성급히 실패로 보지 않는다."""
    all_ok = True
    for name in names:
        try:
            done = subprocess.run(
                [
                    "ros2", "lifecycle", "get", name,
                    # roskill 직후 죽은 XML-RPC daemon 소켓을 재사용하면
                    # rclpy.ok() 예외가 난다. DDS에서 직접 노드를 찾는다.
                    "--no-daemon", "--spin-time", "5.0",
                ],
                # Discovery server가 혼잡하면 lifecycle 서비스 응답이 수십 초
                # 늦어진다. 20초는 정상 노드도 실패로 오판한 사례가 있었다.
                capture_output=True, text=True, timeout=80,
            )
            state = done.stdout.strip() or done.stderr.strip()
        except subprocess.TimeoutExpired:
            state = "응답 시간 초과 — 노드가 없거나 lifecycle 서비스가 멈춤"
        passed = state.startswith("active")
        all_ok &= passed
        report(f"lifecycle {name}", passed, state)
    return all_ok


def check_actions(node: Node, namespace: str, names) -> bool:
    from irobot_create_msgs.action import Dock, Undock
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    prefix = f"/{namespace}"
    types = {
        f"{prefix}/dock": Dock,
        f"{prefix}/undock": Undock,
        f"{prefix}/navigate_to_pose": NavigateToPose,
    }
    all_ok = True
    for name in names:
        client = ActionClient(node, types[name], name)
        passed = client.wait_for_server(timeout_sec=20.0)
        all_ok &= passed
        report(f"action {name}", passed, "" if passed else "서버 없음")
    return all_ok


def check_detection(node: Node) -> bool:
    from aed_interfaces.msg import EmergencyEvent

    topic = "/aed/emergency_event"
    event = wait_for(node, topic, EmergencyEvent, timeout=5.0)
    return report(
        f"응급 이벤트 {topic}", event is not None,
        f"event_id={event.event_id} conf={event.confidence:.2f}"
        if event else "이벤트 없음 — 검출 노드와 시험 대상을 확인",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--localization", action="store_true", help="map_server/AMCL 까지 점검"
    )
    parser.add_argument("--nav", action="store_true", help="Nav2 까지 점검")
    parser.add_argument("--detect", action="store_true", help="검출 노드까지 점검")
    parser.add_argument(
        "--host", default=None,
        help="기본값: --namespace의 로봇 번호로 192.168.107.10N 자동 계산",
    )
    parser.add_argument("--namespace", default="robot1")
    args = parser.parse_args()

    if args.host is None:
        digits = "".join(ch for ch in args.namespace if ch.isdigit())
        if not digits:
            print(f"오류: --namespace '{args.namespace}'에서 로봇 번호를 못 찾았습니다. "
                  "--host를 직접 지정하세요.", file=sys.stderr)
            return 1
        args.host = f"192.168.107.{100 + int(digits)}"

    if not check_env():
        return 1
    if not check_ping(args.host):
        return 1

    prefix = f"/{args.namespace}"
    # tf 는 로봇 namespace 아래에 있다. 매번 손으로 붙이지 않도록 여기서 넣는다.
    rclpy.init(args=[
        "--ros-args",
        "-r", f"/tf:=/{args.namespace}/tf",
        "-r", f"/tf_static:=/{args.namespace}/tf_static",
    ])
    node = Node("preflight")
    try:
        if not check_topics(node, args.namespace):
            return 1
        check_camera(node, args.namespace)
        check_actions(
            node, args.namespace, [f"{prefix}/dock", f"{prefix}/undock"]
        )
        if args.localization or args.nav:
            check_lifecycle([f"{prefix}/map_server", f"{prefix}/amcl"])
            check_tf(node)
        if args.nav:
            check_lifecycle([
                f"{prefix}/controller_server",
                f"{prefix}/planner_server",
                f"{prefix}/bt_navigator",
            ])
            check_actions(
                node, args.namespace, [f"{prefix}/navigate_to_pose"]
            )
        if args.detect:
            check_detection(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print()
    if failures:
        print(f"실패 {len(failures)}건: {', '.join(failures)}")
        return 1
    print("전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
