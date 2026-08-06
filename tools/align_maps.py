#!/usr/bin/env python3
"""두 지도를 겹쳐 맞춰 좌표계 사이의 변환을 구한다.

로봇마다 따로 SLAM을 돌리면 각 지도의 원점이 그 로봇의 출발 지점이 되어
좌표계가 서로 다르다. 같은 공간을 찍은 지도라면 벽 모양을 맞춰서
두 좌표계 사이의 회전과 평행이동을 복원할 수 있다.

그러면 한쪽 지도에서만 아는 위치(예: 그 로봇의 출발 지점 = 원점)를
공용 좌표계 값으로 옮길 수 있다.

사용:
  python3 tools/align_maps.py maps/map1.yaml maps/map.yaml

  첫 번째가 옮길 지도(source), 두 번째가 기준 지도(target)다.

출력은 source 좌표를 target 좌표로 바꾸는 변환이다.
  p_target = R(theta) * p_source + (tx, ty)

source의 원점 (0,0) 은 그대로 (tx, ty) 로 간다. 즉 source 지도를 만든
로봇의 SLAM 시작 지점이 공용 좌표계에서 어디인지 바로 알 수 있다.

정합률이 낮으면 결과를 믿으면 안 된다. 지도 하나에 드리프트가 있으면
벽이 두껍게 그려져 맞춰지지 않는다.
"""
import math
import os
import sys

import cv2
import numpy as np
import yaml

GRID = 0.05          # 맞춤 계산에 쓸 격자 크기(m)
ANGLE_RANGE = 20.0   # 탐색할 회전 범위(+-, deg)
ANGLE_STEP = 0.25
TOL_CELLS = 2        # 정합 판정 허용 오차(셀)


def load_map(path):
    with open(path, encoding="utf-8") as handle:
        meta = yaml.safe_load(handle)
    image_path = os.path.join(os.path.dirname(path), meta["image"])
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"실패: {image_path} 를 못 읽음")
        sys.exit(1)
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin"][0]), float(meta["origin"][1])
    height, width = image.shape

    # pgm 은 위에서 아래로 저장되고 맵 좌표는 아래에서 위로 커진다.
    rows, cols = np.nonzero(image < 50)          # 어두운 셀 = 벽
    xs = origin_x + (cols + 0.5) * resolution
    ys = origin_y + (height - 1 - rows + 0.5) * resolution
    return np.stack([xs, ys], axis=1), meta


def rasterize(points, bounds):
    min_x, min_y, max_x, max_y = bounds
    width = int(math.ceil((max_x - min_x) / GRID)) + 1
    height = int(math.ceil((max_y - min_y) / GRID)) + 1
    grid = np.zeros((height, width), dtype=np.float32)
    cols = ((points[:, 0] - min_x) / GRID).astype(int)
    rows = ((points[:, 1] - min_y) / GRID).astype(int)
    keep = (
        (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    )
    grid[rows[keep], cols[keep]] = 1.0
    return grid


def best_shift(source_grid, target_grid):
    """FFT 순환 상관으로 최적 평행이동을 찾는다. 격자 단위로 반환."""
    spectrum = np.fft.rfft2(target_grid) * np.conj(np.fft.rfft2(source_grid))
    correlation = np.fft.irfft2(spectrum, s=target_grid.shape)
    index = int(np.argmax(correlation))
    row, col = np.unravel_index(index, correlation.shape)
    score = float(correlation[row, col])
    height, width = correlation.shape
    # 순환 상관이라 절반을 넘으면 음수 이동이다.
    if row > height // 2:
        row -= height
    if col > width // 2:
        col -= width
    return int(col), int(row), score


def match_ratio(source_points, target_points):
    """옮긴 벽 점이 기준 지도 벽 근처에 떨어지는 비율."""
    keys = {
        (int(round(x / GRID)), int(round(y / GRID)))
        for x, y in target_points
    }
    hit = 0
    for x, y in source_points:
        cx, cy = int(round(x / GRID)), int(round(y / GRID))
        found = False
        for dx in range(-TOL_CELLS, TOL_CELLS + 1):
            for dy in range(-TOL_CELLS, TOL_CELLS + 1):
                if (cx + dx, cy + dy) in keys:
                    found = True
                    break
            if found:
                break
        hit += found
    return hit / max(len(source_points), 1)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    source_path, target_path = sys.argv[1], sys.argv[2]
    source, source_meta = load_map(source_path)
    target, target_meta = load_map(target_path)
    print(f"source: {source_path}  벽 점 {len(source)}개  "
          f"origin={source_meta['origin'][:2]}")
    print(f"target: {target_path}  벽 점 {len(target)}개  "
          f"origin={target_meta['origin'][:2]}")

    # 회전은 source 좌표계 원점을 중심으로 한다. 그래야 원점이 어디로
    # 가는지가 곧 평행이동 값이 된다.
    all_points = np.vstack([source, target])
    margin = 1.5
    bounds = (
        all_points[:, 0].min() - margin, all_points[:, 1].min() - margin,
        all_points[:, 0].max() + margin, all_points[:, 1].max() + margin,
    )
    target_grid = rasterize(target, bounds)

    best = None
    angles = np.arange(-ANGLE_RANGE, ANGLE_RANGE + ANGLE_STEP, ANGLE_STEP)
    for angle in angles:
        radians = math.radians(angle)
        cos_a, sin_a = math.cos(radians), math.sin(radians)
        rotated = np.stack([
            source[:, 0] * cos_a - source[:, 1] * sin_a,
            source[:, 0] * sin_a + source[:, 1] * cos_a,
        ], axis=1)
        source_grid = rasterize(rotated, bounds)
        shift_x, shift_y, score = best_shift(source_grid, target_grid)
        if best is None or score > best[0]:
            best = (score, angle, shift_x * GRID, shift_y * GRID, rotated)

    score, angle, tx, ty, rotated = best
    moved = rotated + np.array([tx, ty])
    ratio = match_ratio(moved, target)

    print()
    print("=== 정렬 결과 ===")
    print(f"회전   theta = {angle:+.2f} deg")
    print(f"평행이동  t  = ({tx:+.3f}, {ty:+.3f}) m")
    print(f"벽 정합률    = {ratio * 100:.1f}%  (허용 오차 {TOL_CELLS}셀 "
          f"= {TOL_CELLS * GRID * 100:.0f}cm)")
    print()
    print("변환식:  p_target = R(theta) * p_source + t")
    print()
    print(f"source 지도를 만든 로봇의 SLAM 시작 지점은")
    print(f"  target 좌표계에서  x={tx:.3f}  y={ty:.3f}  yaw={angle:.1f}deg")
    print()
    if ratio >= 0.80:
        print("판정: 정렬 良 — 이 변환을 초기 추정값으로 쓸 수 있다")
    elif ratio >= 0.55:
        print("판정: 애매 — 지도 한쪽에 드리프트가 있다. 참고값으로만 쓴다")
    else:
        print("판정: 불일치 — 같은 공간이 아니거나 드리프트가 심하다")
    print()
    print("주의: 이 값은 지도끼리 맞춘 추정이다. 실제 로봇 위치는 AMCL을")
    print("수렴시킨 뒤 tools/check_fit.py 로 검증해서 확정해야 한다.")
    return 0 if ratio >= 0.55 else 1


if __name__ == "__main__":
    sys.exit(main())
