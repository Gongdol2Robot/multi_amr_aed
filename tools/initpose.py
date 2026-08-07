#!/usr/bin/env python3
"""로봇별 초기 위치를 공용 지도 좌표계로 발행하거나 기록한다.

로봇 두 대가 서로 다른 Dock에서 출발하지만 지도는 하나를 공유한다.
지도가 공통이므로 로봇마다 달라지는 것은 초기 위치뿐이다.
그 값을 src/aed_bringup/config/dock_poses.yaml 에 두고 여기서 읽는다.

발행:
  python3 tools/initpose.py 2
    설정에 적힌 robot2의 Dock 위치를 /robot2/initialpose 로 보낸다.

기록:
  python3 tools/initpose.py 1 --record
    현재 map->base_link tf를 읽어 robot1 항목에 써 넣는다.
    RViz의 2D Pose Estimate로 AMCL을 수렴시킨 뒤 실행한다.

  tf가 네임스페이스 안에 발행되므로 --record 는 리매핑이 필요하다.
  이 스크립트가 알아서 붙이므로 --ros-args 를 직접 줄 필요는 없다.

기록 후에는 tools/check_fit.py 로 정합률을 확인해야 값이 믿을 만해진다.
AMCL이 엉뚱한 곳에 수렴한 상태로 기록하면 그 오차가 그대로 굳는다.
"""
import argparse
import math
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(
    REPO, "src", "aed_bringup", "config", "dock_poses.yaml"
)


def load_config():
    if not os.path.exists(CONFIG):
        print(f"실패: {CONFIG} 가 없습니다.")
        sys.exit(1)
    with open(CONFIG, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def update_in_place(key, values):
    """해당 로봇 블록의 값 줄만 바꿔 쓴다.

    yaml.safe_dump 로 통째로 다시 쓰면 측정 절차를 적어 둔 주석이 전부
    사라진다. 그 주석이 이 파일의 핵심이라 줄 단위로 갈아끼운다.
    """
    with open(CONFIG, encoding="utf-8") as handle:
        lines = handle.readlines()

    start = None
    for index, line in enumerate(lines):
        if line.strip() == f"{key}:":
            start = index
            break
    if start is None:
        print(f"실패: {CONFIG} 에서 {key} 블록을 찾지 못했습니다.")
        sys.exit(1)

    indent = len(lines[start]) - len(lines[start].lstrip())
    remaining = dict(values)
    for index in range(start + 1, len(lines)):
        line = lines[index]
        if line.strip() and (len(line) - len(line.lstrip())) <= indent:
            break  # 다음 로봇 블록으로 넘어감
        stripped = line.strip()
        if stripped.startswith("#") or ":" not in stripped:
            continue
        name = stripped.split(":", 1)[0].strip()
        if name in remaining:
            pad = " " * (len(line) - len(line.lstrip()))
            lines[index] = f"{pad}{name}: {remaining.pop(name)}\n"
    if remaining:
        print(f"실패: {key} 블록에 {', '.join(remaining)} 항목이 없습니다.")
        sys.exit(1)

    with open(CONFIG, "w", encoding="utf-8") as handle:
        handle.writelines(lines)


def robot_entry(config, number):
    key = f"robot{number}"
    entry = (config.get("robots") or {}).get(key)
    if entry is None:
        print(f"실패: {CONFIG} 에 {key} 항목이 없습니다.")
        sys.exit(1)
    return key, entry


def publish(number, entry, covariance):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    yaw = math.radians(float(entry["yaw_deg"]))
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.position.x = float(entry["x"])
    message.pose.pose.position.y = float(entry["y"])
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    matrix = [0.0] * 36
    matrix[0] = matrix[7] = float(covariance.get("xy", 0.25))
    matrix[35] = float(covariance.get("yaw", 0.0685))
    message.pose.covariance = matrix

    rclpy.init()
    node = Node("initpose_publisher")
    topic = f"/robot{number}/initialpose"
    # AMCL이 구독을 붙이기 전에 보내면 조용히 사라진다. transient local로
    # 늦게 붙는 구독자에게도 전달되게 하고, 여러 번 발행한다.
    publisher = node.create_publisher(
        PoseWithCovarianceStamped, topic, QoSProfile(depth=1)
    )
    deadline = node.get_clock().now().nanoseconds + 5_000_000_000
    sent = 0
    while node.get_clock().now().nanoseconds < deadline and sent < 5:
        if publisher.get_subscription_count() > 0:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            sent += 1
        rclpy.spin_once(node, timeout_sec=0.3)

    node.destroy_node()
    rclpy.shutdown()

    if sent == 0:
        print(f"실패: {topic} 을 구독하는 노드가 없습니다.")
        print(f"  localization이 떠 있는지 확인하세요 — loc {number}")
        return 1
    print(f"{topic} 로 {sent}회 발행")
    print(f"  x={entry['x']} y={entry['y']} yaw={entry['yaw_deg']}deg")
    if not entry.get("measured", False):
        print("  주의: 이 값은 아직 실측되지 않은 추정값입니다.")
        print(f"  AMCL 수렴 후 --record 로 확정하세요.")
    return 0


def record(number):
    import rclpy
    import rclpy.time
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    namespace = f"/robot{number}"
    rclpy.init(args=[
        "--ros-args",
        "-r", f"/tf:={namespace}/tf",
        "-r", f"/tf_static:={namespace}/tf_static",
    ])
    node = Node("initpose_recorder")
    buffer = Buffer()
    TransformListener(buffer, node)

    transform = None
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            break
        except Exception:
            continue
    node.destroy_node()
    rclpy.shutdown()

    if transform is None:
        print("실패: map->base_link 변환이 없습니다.")
        print(f"  localization이 떠 있는지 확인하세요 — loc {number}")
        print("  RViz의 2D Pose Estimate로 대략 위치를 먼저 주어야 합니다.")
        return 1

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = math.degrees(2 * math.atan2(rotation.z, rotation.w))

    config = load_config()
    key, _ = robot_entry(config, number)
    update_in_place(key, {
        "measured": "true",
        "x": f"{translation.x:.4f}",
        "y": f"{translation.y:.4f}",
        "yaw_deg": f"{yaw:.1f}",
    })

    print(f"{key} 기록 완료: x={translation.x:.3f} y={translation.y:.3f} "
          f"yaw={yaw:.1f}deg")
    print(f"  저장: {CONFIG}")
    print()
    print("다음: python3 tools/check_fit.py --ros-args "
          f"-r /tf:=/robot{number}/tf -r /tf_static:=/robot{number}/tf_static")
    print("  정합률 80% 이상이어야 이 값을 믿을 수 있습니다.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="공용 지도 좌표계로 로봇별 초기 위치를 발행/기록한다."
    )
    parser.add_argument("robot", type=int, help="로봇 번호 (예: 1, 2)")
    parser.add_argument(
        "--record", action="store_true",
        help="현재 tf 위치를 설정 파일에 기록한다",
    )
    args = parser.parse_args()

    if args.record:
        return record(args.robot)

    config = load_config()
    _, entry = robot_entry(config, args.robot)
    return publish(args.robot, entry, config.get("covariance") or {})


if __name__ == "__main__":
    sys.exit(main())
