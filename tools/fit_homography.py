#!/usr/bin/env python3
"""points.csv의 대응점으로 호모그래피 행렬을 계산하고 오차를 검증한다.

pick_pixels.py 나 survey_point.py 로 모은 (pixel_u, pixel_v) <-> (map_x, map_y)
쌍에서 바닥 평면의 픽셀->맵 변환 행렬을 구한다.

사용:
  python3 tools/fit_homography.py            # 계산하고 오차만 출력
  python3 tools/fit_homography.py --write    # 통과하면 설정 파일까지 저장

오차는 각 기준점을 행렬로 변환해 실제 맵 좌표와 비교한 잔차다. 4점만 쓰면
잔차가 항상 0이라 검증이 되지 않으므로, 가능하면 5점 이상 찍는다.

환경변수: CAM_INDEX(기본 2), CAM_WIDTH(640), CAM_HEIGHT(480)
결과: src/aed_vision/config/homography.yaml
"""
import csv
import math
import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 카메라별로 행렬이 따로 나온다. 카메라마다 보는 각도가 달라 하나로 못 쓴다.
# CAM_ID 는 논리 번호(EmergencyEvent.camera_id 와 맞춘다),
# CAM_INDEX 는 /dev/videoN 의 N 이다. 둘은 다를 수 있다.
CAM_ID = os.environ.get("CAM_ID", "2")
CSV_PATH = os.path.join(REPO, "tools", "survey", f"cam{CAM_ID}", "points.csv")
CONFIG_PATH = os.path.join(
    REPO, "src", "aed_vision", "config", f"homography_cam{CAM_ID}.yaml"
)
CAM_INDEX = int(os.environ.get("CAM_INDEX", "2"))
CAM_WIDTH = int(os.environ.get("CAM_WIDTH", "640"))
CAM_HEIGHT = int(os.environ.get("CAM_HEIGHT", "480"))

PASS_M, WARN_M = 0.05, 0.12  # 평균 잔차 판정 기준(m)


def load_points():
    if not os.path.exists(CSV_PATH):
        print(f"실패: {CSV_PATH} 가 없습니다. 먼저 pick_pixels.py 를 실행하세요.")
        sys.exit(1)
    points = []
    with open(CSV_PATH, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                points.append({
                    "label": row["label"],
                    "pixel": (float(row["pixel_u"]), float(row["pixel_v"])),
                    "map": (float(row["map_x"]), float(row["map_y"])),
                })
            except (KeyError, TypeError, ValueError):
                continue  # 픽셀이 아직 안 채워진 행
    return points


def collinear_triples(points):
    """세 점이 거의 한 직선 위에 있으면 행렬이 불안정해진다."""
    bad = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            for k in range(j + 1, len(points)):
                (x1, y1), (x2, y2), (x3, y3) = (
                    points[i]["pixel"], points[j]["pixel"], points[k]["pixel"]
                )
                area = abs(
                    (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)
                ) / 2.0
                if area < 200.0:  # 픽셀^2
                    bad.append(
                        (points[i]["label"], points[j]["label"],
                         points[k]["label"])
                    )
    return bad


def survey_area(points):
    """측량 영역의 경계. 안쪽에 찍은 점은 꼭짓점이 아니므로 볼록껍질을 쓴다."""
    maps = np.array([p["map"] for p in points], dtype=np.float32)
    hull = cv2.convexHull(maps.reshape(-1, 1, 2))
    return [(float(x), float(y)) for x, y in hull.reshape(-1, 2)]


def format_config(matrix, points, area):
    lines = [
        "# tools/fit_homography.py 가 생성한 파일입니다.",
        "# 현장 재측량 없이 손으로 고치지 마세요.",
        "camera:",
        # EmergencyEvent.camera_id 와 맞춘다. 어느 카메라의 검출인지에 따라
        # 적용할 행렬이 달라진다.
        f"  id: \"cam{CAM_ID}\"",
        f"  device: {CAM_INDEX}",
        f"  width: {CAM_WIDTH}",
        f"  height: {CAM_HEIGHT}",
        "",
        "# 픽셀(u, v, 1) -> 맵(x, y, w)",
        "homography:",
    ]
    for row in matrix:
        cells = ", ".join(f"{value:.10g}" for value in row)
        lines.append(f"  - [{cells}]")
    lines += [
        "",
        "# 좌표 변환을 신뢰할 수 있는 범위. 이 밖의 검출은 외삽이라 버립니다.",
        "survey_area:",
    ]
    for x, y in area:
        lines.append(f"  - [{x:.4f}, {y:.4f}]")
    lines += [
        "",
        "# 측량 기록. 재계산할 때 쓰는 원본이며 런타임에는 참고용입니다.",
        "correspondences:",
    ]
    for point in points:
        lines += [
            f"  - label: {point['label']}",
            f"    pixel: [{point['pixel'][0]:.1f}, {point['pixel'][1]:.1f}]",
            f"    map: [{point['map'][0]:.4f}, {point['map'][1]:.4f}]",
        ]
    return "\n".join(lines) + "\n"


def main():
    points = load_points()
    print(f"대응점: {CSV_PATH}")
    print(f"픽셀이 채워진 점: {len(points)}개")
    if len(points) < 4:
        print(f"실패: 최소 4점이 필요합니다. {4 - len(points)}개 더 찍으세요.")
        return 1

    bad = collinear_triples(points)
    if bad:
        for triple in bad[:5]:
            print(f"주의: {' / '.join(triple)} 가 거의 일직선입니다")
        print("  → 세 점이 한 줄에 몰리면 행렬이 불안정합니다. 넓게 다시 찍으세요.")

    pixels = np.array([p["pixel"] for p in points], dtype=np.float64)
    maps = np.array([p["map"] for p in points], dtype=np.float64)
    matrix, _ = cv2.findHomography(pixels, maps, 0)  # 0 = 최소제곱
    if matrix is None:
        print("실패: 행렬을 못 구했습니다. 점이 퇴화(일직선/중복)했는지 확인하세요.")
        return 1

    inverse = np.linalg.inv(matrix)
    errors = []
    print()
    print("점별 잔차 (행렬로 변환한 값 - 실제 측량값)")
    print(f"{'label':<10}{'pixel':>14}{'map(측량)':>20}"
          f"{'map(변환)':>20}{'오차 m':>10}")
    for point in points:
        u, v = point["pixel"]
        map_x, map_y = point["map"]
        projected = matrix @ np.array([u, v, 1.0])
        x, y = projected[0] / projected[2], projected[1] / projected[2]
        error = math.hypot(x - map_x, y - map_y)
        errors.append(error)
        pixel_text = f"{u:.0f},{v:.0f}"
        surveyed_text = f"{map_x:.3f},{map_y:.3f}"
        projected_text = f"{x:.3f},{y:.3f}"
        print(f"{point['label']:<10}{pixel_text:>14}{surveyed_text:>20}"
              f"{projected_text:>20}{error:>10.3f}")

    mean_error = sum(errors) / len(errors)
    max_error = max(errors)

    # 화면 중앙 1픽셀이 맵에서 몇 m인지 — 분해능 감각을 잡기 위한 값
    center = (CAM_WIDTH / 2.0, CAM_HEIGHT / 2.0)
    p0 = matrix @ np.array([center[0], center[1], 1.0])
    p1 = matrix @ np.array([center[0] + 1.0, center[1], 1.0])
    meters_per_pixel = math.hypot(
        p1[0] / p1[2] - p0[0] / p0[2], p1[1] / p1[2] - p0[1] / p0[2]
    )

    print()
    print(f"평균 잔차: {mean_error:.3f} m")
    print(f"최대 잔차: {max_error:.3f} m")
    print(f"화면 중앙 1픽셀 ≈ {meters_per_pixel * 100:.1f} cm")
    if len(points) == 4:
        print("주의: 4점은 잔차가 항상 0이라 검증이 되지 않습니다. "
              "1~2점 더 찍어 확인하세요.")
    print()

    if mean_error <= PASS_M:
        print("판정: 양호 — 사용 가능")
    elif mean_error <= WARN_M:
        print("판정: 애매 — 큰 오차를 내는 점을 다시 측량하세요")
    else:
        print("판정: 불량 — 대응이 틀렸을 가능성이 큽니다")
        print("  픽셀과 맵 좌표의 짝이 뒤바뀌지 않았는지, "
              "모든 점이 바닥 평면 위인지 확인하세요")

    if "--write" not in sys.argv[1:]:
        print()
        print("설정 파일로 저장하려면: python3 tools/fit_homography.py --write")
        return 0 if mean_error <= WARN_M else 1

    if mean_error > WARN_M:
        print()
        print("잔차가 너무 커서 저장하지 않았습니다. 재측량 후 다시 실행하세요.")
        return 1

    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, "w", encoding="utf-8") as handle:
        handle.write(format_config(matrix, points, survey_area(points)))
    print()
    print(f"저장: {CONFIG_PATH}")
    print("colcon build 후 aed_vision 노드가 이 파일을 읽습니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
