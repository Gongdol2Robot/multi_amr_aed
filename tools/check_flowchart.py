#!/usr/bin/env python3
"""그린 그림에 겹침이 없는지 기계로 확인한다.

눈으로 보면 불투명한 상자에 가려 선이 안 보인다. 그래도 도형은 겹쳐 있고,
drawio 로 열어 상자를 옮기는 순간 드러난다. 사람이 볼 때 문제없어야 하므로
좌표로 직접 따진다.

  1. 상자끼리 겹치나 (품고 있는 것은 뺀다)
  2. 화살표가 상자를 뚫고 지나가나 (품은 상자 안의 화살표는 뺀다)
  3. 화살표끼리 같은 자리를 지나나

사용:
  python3 tools/check_flowchart.py docs/system_flow.drawio
"""
import sys
import xml.etree.ElementTree as ET


def rect(cell):
    g = cell.find("mxGeometry")
    return (float(g.get("x")), float(g.get("y")),
            float(g.get("width")), float(g.get("height")))


def overlaps(a, b) -> bool:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return not (ax + aw <= bx or bx + bw <= ax
                or ay + ah <= by or by + bh <= ay)


def contains(outer, inner) -> bool:
    ox, oy, ow, oh = outer
    ix, iy, iw, ih = inner
    return (ox <= ix and oy <= iy
            and ox + ow >= ix + iw and oy + oh >= iy + ih)


def crosses(x1, y1, x2, y2, box) -> bool:
    """선이 상자 안쪽을 지나가나. 테두리에 닿는 것은 지나가는 것이 아니다."""
    bx, by, bw, bh = box
    if x1 == x2:
        return (bx < x1 < bx + bw
                and not (max(y1, y2) <= by or min(y1, y2) >= by + bh))
    if y1 == y2:
        return (by < y1 < by + bh
                and not (max(x1, x2) <= bx or min(x1, x2) >= bx + bw))
    return False


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "docs/system_flow.drawio"
    model = ET.parse(path).getroot().find(".//mxGraphModel")
    cells = model.findall(".//mxCell")
    boxes = [(c.get("id"), rect(c)) for c in cells if c.get("vertex") == "1"]

    segments = []
    for cell in cells:
        if cell.get("edge") != "1":
            continue
        # 레인 안내선(생명선)은 지나가라고 있는 선이다. 검사에서 뺀다.
        if "dashPattern=3 7" in (cell.get("style") or ""):
            continue
        g = cell.find("mxGeometry")
        source = g.find("mxPoint[@as='sourcePoint']")
        target = g.find("mxPoint[@as='targetPoint']")
        segments.append((
            cell.get("id"),
            float(source.get("x")), float(source.get("y")),
            float(target.get("x")), float(target.get("y")),
        ))

    print(f"{path}")
    print(f"  상자 {len(boxes)}개 · 화살표 {len(segments)}개")

    problems = 0

    # 1. 상자끼리
    hits = []
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            if not overlaps(a[1], b[1]):
                continue
            if contains(a[1], b[1]) or contains(b[1], a[1]):
                continue      # 기둥이 안쪽 상자를 품는 것은 정상이다
            hits.append(f"{a[0]}↔{b[0]}")
    problems += len(hits)
    print(f"  상자 겹침        : {', '.join(hits) if hits else '없음'}")

    # 2. 화살표가 상자를 뚫나
    hits = []
    for segment in segments:
        sid, x1, y1, x2, y2 = segment
        for bid, box in boxes:
            if not crosses(x1, y1, x2, y2, box):
                continue
            # 상자가 선을 통째로 품고 있으면 그 안쪽 화살표다
            if contains(box, (min(x1, x2), min(y1, y2),
                              abs(x2 - x1), abs(y2 - y1))):
                continue
            hits.append(f"{sid}→{bid}")
    problems += len(hits)
    print(f"  상자를 뚫는 화살표: {', '.join(hits) if hits else '없음'}")

    # 3. 화살표끼리
    hits = []
    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            a, b = segments[i], segments[j]
            same_x = a[1] == a[3] == b[1] == b[3]
            same_y = a[2] == a[4] == b[2] == b[4]
            if same_x and not (max(a[2], a[4]) <= min(b[2], b[4])
                               or max(b[2], b[4]) <= min(a[2], a[4])):
                hits.append(f"{a[0]}↔{b[0]}")
            elif same_y and not (max(a[1], a[3]) <= min(b[1], b[3])
                                 or max(b[1], b[3]) <= min(a[1], a[3])):
                hits.append(f"{a[0]}↔{b[0]}")
    problems += len(hits)
    print(f"  화살표 겹침      : {', '.join(hits) if hits else '없음'}")

    print("  결과: " + ("문제 없음" if problems == 0 else f"{problems}건"))
    return 0 if problems == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
