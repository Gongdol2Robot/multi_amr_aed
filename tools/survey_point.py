#!/usr/bin/env python3
"""호모그래피 대응점 측량 도구.

로봇을 웹캠 화면 안에 세워두고 실행하면, 그 순간의
웹캠 프레임과 로봇의 맵 좌표를 함께 기록한다.
둘을 따로 찍으면 로봇이 조금이라도 움직였을 때 어긋나므로 같이 받는다.

사용:
  python3 tools/survey_point.py p1 --ros-args \
    -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
  (로봇을 다른 위치로 옮긴 뒤 라벨만 바꿔 반복)

  tf가 네임스페이스 안에 발행되므로 위 리매핑이 필요하다.

결과: tools/survey/<label>.jpg, tools/survey/points.csv

이미지에서 로봇 바닥 원의 중심 픽셀을 읽어 points.csv에
pixel_u, pixel_v 열을 채우면 호모그래피를 계산할 수 있다.
"""
import csv
import math
import os
import sys

import cv2
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 카메라가 여러 대면 대응점도 카메라마다 따로 모은다. 한 폴더에 섞으면
# 나중에 측량한 카메라가 앞의 결과를 덮어쓴다.
CAM_ID = os.environ.get("CAM_ID", "2")
OUT = os.path.join(REPO, "tools", "survey", f"cam{CAM_ID}")
CSV = os.path.join(OUT, "points.csv")
CAM_INDEX = int(os.environ.get("CAM_INDEX", "2"))


def grab_pose(timeout_s=30.0):
    """map -> base_link 를 tf로 읽는다. amcl_pose는 정지 시 갱신이 없어 못 쓴다."""
    rclpy.init()
    node = Node("survey_pose_grabber")
    buf = Buffer()
    TransformListener(buf, node)
    tf = None
    for _ in range(int(timeout_s * 10)):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            break
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()
    return tf


def grab_frame():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        return None
    frame = None
    for _ in range(10):  # 자동노출 안정화
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    return frame


def main():
    if len(sys.argv) < 2:
        print("사용: python3 tools/survey_point.py <라벨>   예: p1")
        sys.exit(1)
    label = sys.argv[1]
    os.makedirs(OUT, exist_ok=True)

    frame = grab_frame()
    if frame is None:
        print(f"실패: /dev/video{CAM_INDEX} 를 못 엶")
        sys.exit(1)

    tf = grab_pose()
    if tf is None:
        print("실패: map->base_link tf 없음 — loc이 떠 있는지, tf 리매핑을 줬는지 확인")
        sys.exit(1)

    t, q = tf.transform.translation, tf.transform.rotation
    x, y = t.x, t.y
    yaw = math.degrees(2 * math.atan2(q.z, q.w))

    img_path = os.path.join(OUT, f"{label}.jpg")
    cv2.imwrite(img_path, frame)

    new = not os.path.exists(CSV)
    with open(CSV, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["label", "map_x", "map_y", "yaw_deg", "pixel_u", "pixel_v"])
        w.writerow([label, f"{x:.4f}", f"{y:.4f}", f"{yaw:.1f}", "", ""])

    print(f"[{label}] 기록 완료")
    print(f"  맵 좌표 : x={x:.3f}  y={y:.3f}  yaw={yaw:.1f}deg")
    print(f"  이미지  : {img_path}  ({frame.shape[1]}x{frame.shape[0]})")
    print(f"  CSV     : {CSV}")


main()
