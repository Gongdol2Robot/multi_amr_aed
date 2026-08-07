#!/usr/bin/env python3
"""측량한 호모그래피가 실제로 쓸 만한지 교차검증하고 그림으로 남긴다.

fit_homography.py 가 내는 잔차는 행렬을 맞출 때 쓴 바로 그 점들의 오차라
항상 낙관적이다. 5점으로 8자유도 행렬을 맞추면 남는 여유가 2뿐이어서,
잔차가 작다는 것이 새로운 검출도 정확하다는 뜻이 되지 못한다.

그래서 여기서는 세 가지를 따로 본다.

  1. leave-one-out: 점 하나를 빼고 나머지로 행렬을 맞춘 뒤, 뺀 점을 얼마나
     맞히는지 본다. 측량에 쓰지 않은 위치에서의 오차이므로 이것이 실제
     검출에서 기대할 수 있는 정확도다.
  2. 측량 영역의 크기: 이 영역 밖의 검출은 외삽이라 신뢰할 수 없다.
     화면에서 몇 %를 덮는지가 곧 쓸 수 있는 검출의 비율이다.
  3. 픽셀 민감도: 화면 위치마다 1픽셀이 맵에서 몇 cm인지. 원근 때문에
     먼 쪽일수록 커지고, 그만큼 bbox 하단이 흔들릴 때 위치가 크게 튄다.

사용:
  python3 tools/verify_homography.py            # cam1, cam2 모두
  python3 tools/verify_homography.py --cam 1    # 하나만
  python3 tools/verify_homography.py --no-plot  # 숫자만
"""
import argparse
import csv
import math
import os
import sys

import cv2
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SURVEY_DIR = os.path.join(REPO, "tools", "survey")
CONFIG_DIR = os.path.join(REPO, "src", "aed_vision", "config")
IMAGE_DIR = os.path.join(REPO, "docs", "images")
MAP_YAML = os.path.join(REPO, "maps", "map.yaml")

# leave-one-out 판정 기준(m). 사람 한 명의 폭이 대략 0.5m 이므로, 그보다 큰
# 오차는 "쓰러진 사람 옆"이 아니라 "다른 자리"를 가리키게 된다.
LOO_PASS, LOO_WARN = 0.15, 0.30


def load_points(cam):
    path = os.path.join(SURVEY_DIR, f"cam{cam}", "points.csv")
    points = []
    with open(path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                points.append({
                    "label": row["label"],
                    "pixel": (float(row["pixel_u"]), float(row["pixel_v"])),
                    "map": (float(row["map_x"]), float(row["map_y"])),
                })
            except (KeyError, TypeError, ValueError):
                continue
    return points, path


def project(matrix, u, v):
    q = matrix @ np.array([u, v, 1.0])
    return q[0] / q[2], q[1] / q[2]


def leave_one_out(points):
    """각 점을 한 번씩 빼고 맞혀 본다.

    빠진 점이 볼록껍질의 꼭짓점이면 남은 점들의 바깥이 되어 외삽이 된다.
    안쪽 점이 빠졌을 때와 오차가 크게 다르므로 둘을 구분해 기록한다.
    """
    maps = np.array([p["map"] for p in points], dtype=np.float32)
    hull = cv2.convexHull(maps.reshape(-1, 1, 2)).reshape(-1, 2)
    # convexHull 은 float32 로 돌려주므로 원본 float64 와 그대로 비교하면
    # 같은 점도 다르게 잡힌다. mm 단위로 끊어서 맞춘다.
    def key(xy):
        return (round(float(xy[0]), 3), round(float(xy[1]), 3))
    corners = {key(v) for v in hull}

    results = []
    for i, target in enumerate(points):
        rest = [p for j, p in enumerate(points) if j != i]
        pixels = np.array([p["pixel"] for p in rest], dtype=np.float64)
        maps_rest = np.array([p["map"] for p in rest], dtype=np.float64)
        matrix, _ = cv2.findHomography(pixels, maps_rest, 0)
        if matrix is None:
            continue
        x, y = project(matrix, *target["pixel"])
        gx, gy = target["map"]
        results.append({
            "label": target["label"],
            "predicted": (x, y),
            "truth": (gx, gy),
            "error": math.hypot(x - gx, y - gy),
            "is_corner": key(target["map"]) in corners,
        })
    return results


def pixel_sensitivity(matrix, u, v):
    """(u,v) 에서 1픽셀 이동이 맵에서 몇 m인지, 가로/세로 각각."""
    base = np.array(project(matrix, u, v))
    right = np.array(project(matrix, u + 1.0, v))
    down = np.array(project(matrix, u, v + 1.0))
    return np.linalg.norm(right - base), np.linalg.norm(down - base)


def report(cam, verbose=True):
    points, csv_path = load_points(cam)
    config_path = os.path.join(CONFIG_DIR, f"homography_cam{cam}.yaml")
    config = yaml.safe_load(open(config_path, encoding="utf-8"))
    matrix = np.array(config["homography"], dtype=np.float64)
    area = np.array(config["survey_area"], dtype=np.float64)
    width = int(config["camera"]["width"])
    height = int(config["camera"]["height"])

    residuals = [
        math.hypot(*(np.array(project(matrix, *p["pixel"])) - np.array(p["map"])))
        for p in points
    ]
    loo = leave_one_out(points)
    inner = [r for r in loo if not r["is_corner"]]
    outer = [r for r in loo if r["is_corner"]]

    # 측량 영역을 화면으로 되돌려 몇 %를 덮는지 본다.
    inverse = np.linalg.inv(matrix)
    polygon = np.array(
        [project(inverse, x, y) for x, y in area], dtype=np.float32
    )
    pixel_area = cv2.contourArea(polygon)
    map_area = cv2.contourArea(area.astype(np.float32))

    if verbose:
        print(f"===== cam{cam} =====")
        print(f"측량 기록: {csv_path}")
        print(f"설정 파일: {config_path}")
        print()
        print(f"[1] 재투영 잔차 (맞출 때 쓴 점) "
              f"평균 {np.mean(residuals):.3f} m / 최대 {max(residuals):.3f} m")
        print("    이 값은 낙관적이다. 아래 leave-one-out 이 실제 성능이다.")
        print()
        print("[2] leave-one-out (그 점을 빼고 맞힌 결과)")
        for r in loo:
            place = "꼭짓점(빼면 외삽)" if r["is_corner"] else "안쪽 점(내삽)"
            print(f"    {r['label']:<4} {place:<18} "
                  f"예측 ({r['predicted'][0]:6.3f},{r['predicted'][1]:6.3f}) "
                  f"실제 ({r['truth'][0]:6.3f},{r['truth'][1]:6.3f}) "
                  f"오차 {r['error']:.3f} m")
        if inner:
            print(f"    안쪽 점 평균 {np.mean([r['error'] for r in inner]):.3f} m "
                  f"— 영역 안에서 기대할 수 있는 정확도")
        if outer:
            print(f"    꼭짓점 평균 {np.mean([r['error'] for r in outer]):.3f} m "
                  f"— 영역을 벗어났을 때 벌어지는 정도")
        print()
        print(f"[3] 측량 영역 {map_area:.2f} m^2, "
              f"화면의 {pixel_area / (width * height) * 100:.1f}% "
              f"({pixel_area:.0f} px^2)")
        print("    나머지 화면에서 잡힌 검출은 외삽이라 좌표를 믿을 수 없다.")
        print()
        print("[4] 1픽셀이 맵에서 몇 cm인가")
        for name, (u, v) in [
            ("화면 위(먼 쪽)", (width / 2, height * 0.25)),
            ("화면 중앙", (width / 2, height / 2)),
            ("화면 아래(가까운 쪽)", (width / 2, height * 0.75)),
        ]:
            dx, dy = pixel_sensitivity(matrix, u, v)
            print(f"    {name:<22} 가로 {dx * 100:.1f} cm / 세로 {dy * 100:.1f} cm")
        print()

        worst = max(r["error"] for r in loo)
        basis = np.mean([r["error"] for r in inner]) if inner else worst
        if basis <= LOO_PASS:
            print(f"판정: 영역 안이면 쓸 만하다 (안쪽 오차 {basis:.3f} m)")
        elif basis <= LOO_WARN:
            print(f"판정: 애매하다 — 점을 더 찍어야 한다 (안쪽 오차 {basis:.3f} m)")
        else:
            print(f"판정: 부족하다 — 재측량이 필요하다 (안쪽 오차 {basis:.3f} m)")
        print(f"      영역을 벗어나면 최대 {worst:.3f} m 까지 틀어진다.")
        print()

    return {
        "cam": cam, "points": points, "matrix": matrix, "area": area,
        "residuals": residuals, "loo": loo, "inner": inner, "outer": outer,
        "polygon": polygon, "pixel_ratio": pixel_area / (width * height),
        "map_area": map_area, "width": width, "height": height,
    }


def load_map():
    """map.pgm 을 배경으로 깔기 위해 읽는다. 없으면 배경 없이 그린다."""
    if not os.path.exists(MAP_YAML):
        return None
    meta = yaml.safe_load(open(MAP_YAML, encoding="utf-8"))
    image_path = os.path.join(os.path.dirname(MAP_YAML), meta["image"])
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    h, w = image.shape
    return image, [ox, ox + w * res, oy, oy + h * res]


def plot(data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon

    # 한글 라벨이 깨지지 않게 CJK 폰트를 지정한다.
    matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
    matplotlib.rcParams["axes.unicode_minus"] = False

    cam = data["cam"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    view_path = os.path.join(SURVEY_DIR, f"cam{cam}", "camera_view.jpg")
    view = cv2.imread(view_path)
    if view is not None:
        ax1.imshow(cv2.cvtColor(view, cv2.COLOR_BGR2RGB))
    ax1.add_patch(MplPolygon(
        data["polygon"], closed=True, fill=True,
        facecolor="tab:green", alpha=0.18, edgecolor="tab:green", lw=2.5,
    ))
    for point in data["points"]:
        u, v = point["pixel"]
        ax1.plot(u, v, "o", color="yellow", ms=9, mec="black", mew=1.2)
        ax1.annotate(point["label"], (u, v), xytext=(8, 8),
                     textcoords="offset points", color="yellow", fontsize=11,
                     fontweight="bold",
                     path_effects=None)
    ax1.set_xlim(0, data["width"])
    ax1.set_ylim(data["height"], 0)
    ax1.set_title(
        f"cam{cam} 카메라 화면 — 측량 영역은 화면의 "
        f"{data['pixel_ratio'] * 100:.1f}%\n이 밖에서 잡힌 검출은 외삽이다",
        fontsize=12)
    ax1.set_xlabel("픽셀 u")
    ax1.set_ylabel("픽셀 v")

    background = load_map()
    if background is not None:
        image, extent = background
        ax2.imshow(image, cmap="gray", extent=extent, origin="lower",
                   alpha=0.55, vmin=0, vmax=255)
    ax2.add_patch(MplPolygon(
        data["area"], closed=True, fill=True,
        facecolor="tab:green", alpha=0.20, edgecolor="tab:green", lw=2.5,
        label=f"측량 영역 {data['map_area']:.2f} m²",
    ))
    for result in data["loo"]:
        gx, gy = result["truth"]
        px, py = result["predicted"]
        color = "tab:red" if result["is_corner"] else "tab:blue"
        ax2.annotate("", xy=(px, py), xytext=(gx, gy),
                     arrowprops=dict(arrowstyle="->", color=color, lw=1.8))
        ax2.plot(gx, gy, "o", color="black", ms=7)
        ax2.annotate(f"{result['label']} {result['error']:.2f}m",
                     (gx, gy), xytext=(6, -14), textcoords="offset points",
                     fontsize=10, color=color, fontweight="bold")
    inner_mean = (np.mean([r["error"] for r in data["inner"]])
                  if data["inner"] else float("nan"))
    outer_mean = (np.mean([r["error"] for r in data["outer"]])
                  if data["outer"] else float("nan"))
    ax2.set_title(
        f"cam{cam} leave-one-out — 화살표는 그 점을 빼고 맞힌 예측\n"
        f"안쪽 점 평균 {inner_mean:.3f} m / 꼭짓점 평균 {outer_mean:.3f} m",
        fontsize=12)
    ax2.set_xlabel("map x (m)")
    ax2.set_ylabel("map y (m)")
    ax2.legend(loc="best", fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.set_aspect("equal")
    margin = 0.8
    xs = [p[0] for p in data["area"]]
    ys = [p[1] for p in data["area"]]
    ax2.set_xlim(min(xs) - margin, max(xs) + margin)
    ax2.set_ylim(min(ys) - margin, max(ys) + margin)

    fig.tight_layout()
    os.makedirs(IMAGE_DIR, exist_ok=True)
    out = os.path.join(IMAGE_DIR, f"homography_cam{cam}_verify.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"그림 저장: {out}")
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam", choices=("1", "2"),
                        help="지정하지 않으면 둘 다 검증한다")
    parser.add_argument("--no-plot", action="store_true",
                        help="숫자만 출력하고 그림은 만들지 않는다")
    args = parser.parse_args()

    cams = [args.cam] if args.cam else ["1", "2"]
    worst = 0.0
    for cam in cams:
        data = report(cam)
        if not args.no_plot:
            plot(data)
            print()
        basis = ([r["error"] for r in data["inner"]]
                 or [r["error"] for r in data["loo"]])
        worst = max(worst, float(np.mean(basis)))
    return 0 if worst <= LOO_WARN else 1


if __name__ == "__main__":
    sys.exit(main())
