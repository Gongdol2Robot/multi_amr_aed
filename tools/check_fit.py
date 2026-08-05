#!/usr/bin/env python3
"""라이다 스캔이 저장된 맵의 벽선과 맞는지 정량 검증한다.

localization(map_server + amcl)을 띄우고 초기 위치를 준 뒤 실행하면,
그 위치가 실제로 맞는지 숫자로 나온다. AMCL이 조금 보정했다는 것만으로는
위치가 맞다는 근거가 안 되기 때문에 이 확인이 필요하다.

map->라이다 프레임 tf로 스캔 끝점을 맵 좌표로 옮긴 뒤,
각 끝점이 점유(벽) 셀 근처에 떨어지는 비율을 센다.

사용:
  python3 tools/check_fit.py --ros-args -r /tf:=/robot2/tf -r /tf_static:=/robot2/tf_static

  tf가 네임스페이스 안에 발행되므로 위 리매핑이 필요하다. 생략하면 tf를 못 받는다.

환경변수로 대상 변경:
  ROBOT_NS=/robot4  MAP_YAML=/path/to/map.yaml  python3 tools/check_fit.py --ros-args ...
"""
import math
import os
import re
import sys

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.environ.get("MAP_YAML", os.path.join(REPO, "maps", "map.yaml"))
ROBOT_NS = os.environ.get("ROBOT_NS", "/robot1")
TOL_CELLS = 2  # 허용 오차, 셀 단위 (해상도 0.05m 기준 10cm)
PASS, WARN = 0.80, 0.55


def load_map():
    meta = yaml.safe_load(open(MAP_YAML))
    pgm = open(os.path.join(os.path.dirname(MAP_YAML), meta["image"]), "rb").read()
    m = re.match(rb"P5\s+(?:#[^\n]*\n)?\s*(\d+)\s+(\d+)\s+(\d+)\s", pgm)
    w, h = int(m.group(1)), int(m.group(2))
    px = pgm[m.end():]
    # pgm은 위->아래로 저장되고 맵 좌표는 아래->위라 y를 뒤집는다
    occ = {
        (x, h - 1 - y)
        for y in range(h)
        for x in range(w)
        if px[y * w + x] <= 50  # 어두운 셀 = 점유
    }
    return meta, occ


class Checker(Node):
    def __init__(self):
        super().__init__("scan_map_fit_checker")
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.scan = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(LaserScan, f"{ROBOT_NS}/scan", self.cb, qos)

    def cb(self, msg):
        self.scan = msg


def main():
    meta, occ = load_map()
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]

    rclpy.init()
    node = Checker()
    tf = None
    for _ in range(600):  # 최대 60초 대기
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan is not None:
            try:
                tf = node.buf.lookup_transform(
                    "map", node.scan.header.frame_id, rclpy.time.Time()
                )
                break
            except Exception:
                pass
    if node.scan is None:
        print(f"실패: {ROBOT_NS}/scan 을 못 받음 — 로봇 연결 확인")
        sys.exit(1)
    if tf is None:
        print("실패: map->라이다 tf 없음 — localization이 떠 있는지, tf 리매핑을 줬는지 확인")
        sys.exit(1)

    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    hit = miss = 0
    s = node.scan
    for i, r in enumerate(s.ranges):
        if not (s.range_min < r < s.range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = s.angle_min + i * s.angle_increment + yaw
        cx = int((t.x + r * math.cos(a) - ox) / res)
        cy = int((t.y + r * math.sin(a) - oy) / res)
        near = any(
            (cx + dx, cy + dy) in occ
            for dx in range(-TOL_CELLS, TOL_CELLS + 1)
            for dy in range(-TOL_CELLS, TOL_CELLS + 1)
        )
        hit += near
        miss += not near

    total = hit + miss
    if total == 0:
        print("실패: 유효한 스캔 점이 없음")
        sys.exit(1)

    ratio = hit / total
    print(f"맵: {MAP_YAML}")
    print(f"라이다 위치(map): x={t.x:.3f} y={t.y:.3f} yaw={math.degrees(yaw):.1f}deg")
    print(f"유효 스캔 점: {total}")
    print(f"벽에 맞음 : {hit:5d}  {ratio*100:5.1f}%")
    print(f"안 맞음   : {miss:5d}  {(1-ratio)*100:5.1f}%")
    print()
    if ratio >= PASS:
        print("판정: 정합 良 — localization 신뢰 가능")
    elif ratio >= WARN:
        print("판정: 애매 — 미탐색 영역이 많거나 위치가 조금 틀어짐")
    else:
        print("판정: 불일치 — 초기 위치가 틀림. 전역 위치추정 필요")
        print("  ros2 service call {}/reinitialize_global_localization "
              "std_srvs/srv/Empty '{{}}'".format(ROBOT_NS))
        print("  이후 로봇을 제자리 회전시키면 수렴한다")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ratio >= PASS else 1)


main()
