#!/usr/bin/env python3
"""survey_point.py 의 amcl_pose 판. tf 가 막힐 때 쓴다.

map->base_link tf 를 못 받는 상황이 있었다. Nav2 전체가 도는 중에는 tf
트래픽이 밀려 TransformListener 버퍼가 비어 있는 채로 시간만 흘렀다
(RViz 도 같은 시각 'queue is full' 로 메시지를 버리고 있었다).

amcl_pose 는 그 tf 와 같은 값을 담고 토픽 하나로 오므로 영향을 덜 받는다.
기록 형식은 survey_point.py 와 같아서 pick_pixels.py, fit_homography.py 를
그대로 이어서 쓸 수 있다.

사용:
  CAM_ID=1 CAM_INDEX=2 ROBOT_NS=/robot1 python3 tools/survey_point_amcl.py p2
"""
import csv
import math
import os
import sys

import cv2
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAM_ID = os.environ.get("CAM_ID", "1")
CAM_INDEX = int(os.environ.get("CAM_INDEX", "2"))
ROBOT_NS = os.environ.get("ROBOT_NS", "/robot1")
OUT = os.path.join(REPO, "tools", "survey", f"cam{CAM_ID}")
CSV = os.path.join(OUT, "points.csv")
HEADER = ["label", "map_x", "map_y", "yaw_deg", "pixel_u", "pixel_v"]


def grab_pose(timeout_s=20.0):
    rclpy.init()
    node = Node("survey_point_amcl")
    box = {}
    # Nav2 의 amcl_pose 는 transient local 로 발행된다. 기본 QoS 로 구독하면
    # 호환되지 않아 한 건도 못 받는다(ros2 topic echo 는 자동으로 맞춰준다).
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    node.create_subscription(
        PoseWithCovarianceStamped, f"{ROBOT_NS}/amcl_pose",
        lambda m: box.setdefault("pose", m.pose.pose), qos,
    )
    for _ in range(int(timeout_s * 10)):
        rclpy.spin_once(node, timeout_sec=0.1)
        if "pose" in box:
            break
    node.destroy_node()
    rclpy.shutdown()
    return box.get("pose")


def grab_frame():
    # 기본 백엔드로 열면 이 USB 웹캠은 read() 에서 멈춘다. 노드와 같은 방식.
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    frame = None
    for _ in range(10):
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    return frame


def main():
    if len(sys.argv) < 2:
        print("사용: python3 tools/survey_point_amcl.py <label>")
        return 1
    label = sys.argv[1]
    os.makedirs(OUT, exist_ok=True)

    pose = grab_pose()
    if pose is None:
        print(f"실패: {ROBOT_NS}/amcl_pose 를 못 받았다. localization 확인")
        return 1
    frame = grab_frame()
    if frame is None:
        print(f"실패: /dev/video{CAM_INDEX} 를 못 엶")
        return 1

    x, y = pose.position.x, pose.position.y
    yaw = math.degrees(
        2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
    )
    image_path = os.path.join(OUT, f"{label}.jpg")
    cv2.imwrite(image_path, frame)

    rows = []
    if os.path.exists(CSV):
        with open(CSV, newline="", encoding="utf-8") as handle:
            rows = [r for r in csv.DictReader(handle) if r.get("label") != label]
    rows.append({
        "label": label, "map_x": f"{x:.4f}", "map_y": f"{y:.4f}",
        "yaw_deg": f"{yaw:.1f}", "pixel_u": "", "pixel_v": "",
    })
    with open(CSV, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=HEADER)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[{label}] 기록 완료")
    print(f"  맵 좌표 : x={x:.3f}  y={y:.3f}  yaw={yaw:.1f}deg")
    print(f"  이미지  : {image_path}  ({frame.shape[1]}x{frame.shape[0]})")
    print(f"  CSV     : {CSV}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
