#!/usr/bin/env python3
"""main 기준 실제 배선을 큰 그림 한 장으로 그린다.

왜 손으로 안 그리고 이 스크립트로 만드나
----------------------------------------
화살표가 서로 겹치거나 글자 위를 지나가면 그림이 쓸모없어진다. 좌표를
눈대중으로 찍으면 반드시 그렇게 된다. 그래서 배치 규칙을 코드로 강제한다.

  1. 세로 흐름은 x 축 한 자리(SPINE_X)만 쓴다.
  2. 세로 화살표의 설명 상자는 화살표 오른쪽 빈 칸에만 놓는다.
     선과 상자가 x 로 갈라져 있어 서로 지나갈 수 없다.
  3. 관제로 가는 가로 화살표는 **저마다 다른 높이**를 쓴다.
     순수한 가로선이라 높이가 다르면 겹칠 수가 없다.
  4. 가로 화살표의 설명 상자는 두 기둥 사이 빈 칸(GUTTER)에만 놓는다.

정보는 범례를 찾아보지 않아도 되게 화살표마다 다 적는다.
토픽 이름 · 메시지 타입 · QoS · 주기를 한 자리에 둔다.

구현 상태는 테두리 모양과 글자 양쪽으로 적는다. 한쪽만 보고도 알 수 있어야
범례를 안 찾는다.

  실선 + [구현]  실제로 도는 코드
  점선 + [뼈대]  패키지와 노드는 있으나 콜백이 비었다(29줄 scaffold)
  빨간 + [끊김]  발행자가 없어 이 자리에서 흐름이 멈춘다

사용:
  python3 tools/make_flowchart.py            docs/system_flow.html 생성
"""
import html
import os
import sys

# ── 배치 상수 ────────────────────────────────────────────────────────────
W = 2040                     # 전체 폭
LEFT_X, LEFT_W = 48, 860     # ROS 기둥
GUT_X, GUT_W = 940, 520      # 가로 화살표 설명이 들어가는 빈 칸
RIGHT_X, RIGHT_W = 1500, 500  # 관제 기둥

SPINE_X = LEFT_X + 120       # 세로 화살표가 지나는 단 하나의 x
NOTE_X = SPINE_X + 60        # 세로 화살표 설명 상자의 왼쪽. 선과 겹치지 않는다
NOTE_W = LEFT_X + LEFT_W - NOTE_X

TOP = 150
GAP = 168                    # 박스 사이 간격. 설명 상자가 들어갈 높이
LINE = 21                    # 글줄 높이


def esc(text: str) -> str:
    return html.escape(text, quote=False)


class Canvas:
    def __init__(self) -> None:
        self.parts: list[str] = []
        self.bottom = 0

    def add(self, svg: str) -> None:
        self.parts.append(svg)

    def text(self, x, y, s, size=15, weight="400", fill="currentColor",
             anchor="start", opacity=1.0, mono=False):
        family = ' font-family="ui-monospace,monospace"' if mono else ""
        self.add(
            f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}"'
            f' fill="{fill}" text-anchor="{anchor}" opacity="{opacity}"'
            f'{family}>{esc(s)}</text>'
        )

    def box(self, x, y, w, h, style="solid", accent=None):
        dash = ' stroke-dasharray="7 5"' if style == "dashed" else ""
        stroke = accent or "currentColor"
        width = 2.5 if accent else 1.8
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6"'
            f' fill="currentColor" fill-opacity="0.04"'
            f' stroke="{stroke}" stroke-width="{width}"{dash}/>'
        )


def node(canvas: Canvas, y: int, title: str, status: str, lines: list[str],
         accent=None) -> int:
    """왼쪽 기둥의 상자 하나. 아래쪽 y 를 돌려준다."""
    height = 46 + LINE * len(lines) + 14
    style = "dashed" if status in ("뼈대", "끊김") else "solid"
    canvas.box(LEFT_X, y, LEFT_W, height, style, accent)
    canvas.text(LEFT_X + 20, y + 30, title, size=19, weight="700",
                fill=accent or "currentColor")

    tag = {"구현": "[구현]", "뼈대": "[뼈대 · 콜백 없음]",
           "끊김": "[끊김]", "하드웨어": "[하드웨어]"}[status]
    canvas.text(LEFT_X + LEFT_W - 20, y + 30, tag, size=14, weight="700",
                fill=accent or "currentColor", anchor="end", opacity=0.9)
    for i, line in enumerate(lines):
        canvas.text(LEFT_X + 20, y + 58 + LINE * i, line, size=14,
                    opacity=0.85, mono=line.startswith("/"))
    return y + height


def down(canvas: Canvas, y_from: int, lines: list[str], accent=None) -> int:
    """세로 화살표 하나 + 오른쪽 설명 상자. 다음 상자의 y 를 돌려준다."""
    y_to = y_from + GAP
    stroke = accent or "currentColor"
    canvas.add(
        f'<line x1="{SPINE_X}" y1="{y_from}" x2="{SPINE_X}" y2="{y_to - 6}"'
        f' stroke="{stroke}" stroke-width="2.5" marker-end="url(#a)"/>'
    )
    height = 16 + LINE * len(lines) + 10
    top = y_from + (GAP - height) // 2
    canvas.box(NOTE_X, top, NOTE_W, height, accent=accent)
    for i, line in enumerate(lines):
        bold = "700" if i == 0 else "400"
        canvas.text(NOTE_X + 16, top + 26 + LINE * i, line, size=14,
                    weight=bold, fill=stroke if i == 0 else "currentColor",
                    opacity=1.0 if i == 0 else 0.85,
                    mono=line.startswith("/") or line.startswith("aed_"))
    return y_to


def across(canvas: Canvas, y: int, lines: list[str], accent=None) -> None:
    """관제로 가는 가로 화살표. 높이 y 는 이 화살표만 쓴다."""
    stroke = accent or "currentColor"
    canvas.add(
        f'<line x1="{LEFT_X + LEFT_W}" y1="{y}" x2="{RIGHT_X - 6}" y2="{y}"'
        f' stroke="{stroke}" stroke-width="2.5" marker-end="url(#a)"/>'
    )
    height = 16 + LINE * len(lines) + 10
    top = y - height // 2
    canvas.box(GUT_X, top, GUT_W, height, accent=accent)
    for i, line in enumerate(lines):
        bold = "700" if i == 0 else "400"
        canvas.text(GUT_X + 16, top + 26 + LINE * i, line, size=14,
                    weight=bold, fill=stroke if i == 0 else "currentColor",
                    opacity=1.0 if i == 0 else 0.85,
                    mono=line.startswith("/"))


def build() -> str:
    c = Canvas()
    RED, DIM = "var(--red)", "var(--dim)"

    # ── 제목 ────────────────────────────────────────────────────────────
    c.text(LEFT_X, 54, "Multi-AMR AED — main 기준 실제 배선", size=27,
           weight="700")
    c.text(LEFT_X, 84,
           "토픽 이름·메시지 타입·QoS 를 화살표마다 적었습니다. 범례를 "
           "따로 찾아볼 필요가 없습니다.", size=15, opacity=0.75)
    c.text(LEFT_X, 110,
           "테두리가 점선이면 아직 콜백이 빈 뼈대이고, 빨간 것은 발행자가 "
           "없어 흐름이 멈추는 자리입니다.", size=15, opacity=0.75)

    y = TOP

    # ── ① 고정 웹캠 ─────────────────────────────────────────────────────
    y = node(c, y, "① 고정 웹캠 ×2  ·  천장 USB 카메라", "하드웨어", [
        "camera_open (개방구역) · camera_alley (골목)",
        "640×480 JPEG. 같은 노트북 안에서 도는 노드가 직접 읽는다",
        "(direct_camera=true — 영상을 DDS 로 왕복시키지 않는다)",
    ])
    y = down(c, y, [
        "프로세스 안에서 직접 전달 · DDS 안 거침",
        "cv2.VideoCapture → numpy BGR uint8[480,640,3]",
        "15 Hz",
    ])

    # ── ② vision_detector ───────────────────────────────────────────────
    y_vision = y
    y = node(c, y, "② vision_detector ×2  ·  aed_vision", "구현", [
        "YOLO11n 파인튜닝(rescue_yolo11n.pt) — fallen_person · helper",
        "확정 규칙: 최근 10프레임 중 6프레임 이상 검출되면 CONFIRMED",
        "신뢰도 문턱 rescue_conf = 0.25",
        "혼잡도는 alley 카메라만 계산한다 (open 은 mode=open)",
        "호모그래피로 픽셀→map 변환은 아직 안 붙었다 (좌표가 0,0)",
    ])
    y = down(c, y, [
        "/{camera_id}/vision/emergency_event",
        "aed_interfaces/EmergencyEvent · RELIABLE depth 10",
        "상태가 바뀔 때만 (DETECTED→CONFIRMED→RESOLVED)",
    ])

    # ── ③ location_mapper (끊김) ────────────────────────────────────────
    y = node(c, y, "③ location_mapper  ·  카메라별 → 공용 토픽 중계",
             "뼈대", [
        "패키지와 노드는 있으나 콜백이 비어 있다 (29줄 scaffold)",
        "이것이 없어서 검출이 출동으로 이어지지 않는다",
    ], accent=RED)
    y = down(c, y, [
        "/aed/emergency_event  ← 발행하는 노드가 없다",
        "aed_interfaces/EmergencyEvent · RELIABLE depth 10",
        "mission_manager 는 이 토픽을 듣고 있으나 아무것도 안 온다",
    ], accent=RED)

    # ── ④ mission_manager ───────────────────────────────────────────────
    y_manager = y
    y = node(c, y, "④ mission_manager  ·  후보 순위와 배정", "구현", [
        "구독 /aed/emergency_event (10) · /aed/robot_state (20)",
        "        /aed/mission_status (20)",
        "가용 조건: availability=AVAILABLE ∧ network_ok ∧ nav2_ok ∧ path_valid",
        "순위: rank_candidates() — estimated_path_cost 오름차순",
        "재할당: 실패 로봇을 빼고 assignment_version 을 올려 다시 배정",
    ])
    y = down(c, y, [
        "/{robot_id}/mission_assignment",
        "aed_interfaces/MissionAssignment · RELIABLE depth 10",
        "목표 좌표는 이 메시지에만 실린다 (MissionStatus 에는 상태만)",
        "cancel_previous=true 면 받는 쪽이 기존 goal 을 취소한다",
    ])

    # ── ⑤ mission_executor ──────────────────────────────────────────────
    y = node(c, y, "⑤ mission_executor ×2  ·  robot_missions", "구현", [
        "배정을 Nav2 goal 로 옮긴다. goal_serial 로 늦게 온 결과를 버린다",
        "cancel_previous 면 기존 goal 을 cancel_goal_async() 로 취소",
        "발행 /aed/mission_status · RELIABLE depth 10",
    ])
    y = down(c, y, [
        "/{robot_id}/navigate_to_pose",
        "nav2_msgs/action/NavigateToPose  (Topic 아님 · Action)",
        "goal: PoseStamped(map) / feedback: 남은 거리 / result: 성공·실패",
    ])

    # ── ⑥ Nav2 + TurtleBot4 ─────────────────────────────────────────────
    y = node(c, y, "⑥ Nav2 + TurtleBot4 ×2  ·  주행", "하드웨어", [
        "bt_navigator · planner · controller · AMCL (공용 지도 maps/map.yaml)",
        "출력 /{robot_id}/cmd_vel · geometry_msgs/Twist · 최대 0.20 m/s",
        "센서 /scan · /odom · /battery_state · /dock_status  (BEST_EFFORT)",
        "카메라 /{robot_id}/oakd/rgb/image_raw/compressed (BEST_EFFORT)",
    ])
    y = down(c, y, [
        "센서값을 모아 RobotState 로 옮기는 일",
        "/odom → 위치·속도 · /battery_state → 배터리 · /dock_status → 도킹",
    ], accent=RED)

    # ── ⑦ robot_state_monitor (끊김) ────────────────────────────────────
    y_state = y
    y = node(c, y, "⑦ robot_state_monitor ×2  ·  로봇 상태 집계", "뼈대", [
        "패키지와 노드는 있으나 콜백이 비어 있다 (29줄 scaffold)",
        "이것이 없어서 관제의 로봇 카드가 빈칸으로 남는다",
        "시연에서는 tools/demo_publisher.py 가 bag 의 odom·battery 를 옮긴다",
    ], accent=RED)
    y_bottom_left = y

    # ── 관제로 가는 가로 화살표 (저마다 다른 높이) ──────────────────────
    across(c, y_vision + 96, [
        "/{camera_id}/vision/debug/compressed",
        "sensor_msgs/CompressedImage · BEST_EFFORT depth 1 · 15 Hz",
        "검출 상자가 그려진 그림. 최신 프레임만 의미가 있어 밀리면 버린다",
        "함께: /{camera_id}/vision/person_count · std_msgs/UInt32 · RELIABLE",
    ])
    across(c, y_manager + 108, [
        "/aed/mission_status",
        "aed_interfaces/MissionStatus · RELIABLE depth 20",
        "상태 14종. 한 번만 오므로 놓치면 그 기록이 영영 없다",
        "배정도 함께 구독한다 /{robot_id}/mission_assignment (목표 좌표)",
    ])
    across(c, y_state + 84, [
        "/aed/robot_state  ← 발행하는 노드가 없다",
        "aed_interfaces/RobotState · RELIABLE depth 20",
        "관제는 구독하고 있으나 아무것도 안 온다",
        "AED_HMI_STATE_RELIABILITY=best_effort 로 BEST_EFFORT 전환 가능",
    ], accent=RED)

    # ── 오른쪽 기둥: 관제 ───────────────────────────────────────────────
    ry = TOP
    rh = y_bottom_left - TOP
    c.box(RIGHT_X, ry, RIGHT_W, rh)
    c.text(RIGHT_X + 22, ry + 34, "aed_hmi  ·  관제", size=19, weight="700")
    c.text(RIGHT_X + RIGHT_W - 22, ry + 34, "[구현]", size=14, weight="700",
           anchor="end", opacity=0.9)

    inner = [
        ("aed_hmi_bridge (rclpy 스레드)", [
            "ros/bridge.py — rclpy 를 아는 유일한 곳",
            "구독이 안 붙으면 토픽 이름을 대어 로그에 적는다",
            "QoS 가 어긋나면 ROS 2 는 조용히 연결을 안 맺는다",
        ]),
        ("ros/converters.py", [
            "uint8 → 이름 문자열 · Time → UTC epoch 초",
            "여기서만 aed_interfaces 를 안다",
        ]),
        ("context.on_*  ← 갈림길", [
            "같은 값이 두 갈래로 간다",
            "Ⓐ 메모리(LiveState) · Ⓑ SQLite(Repository)",
        ]),
        ("SQLite  ·  var/aed_hmi.sqlite3", [
            "emergency_events · mission_assignments",
            "mission_events · robot_samples · eta_records",
            "상태 전이는 덧붙이기만 한다. 요약은 매번 되짚는다",
        ]),
        ("FastAPI  ·  :8000", [
            "WS /ws/live — 0.25초마다 스냅샷 (메모리에서)",
            "GET /api/missions · /api/stats/* (SQLite 에서)",
            "MJPEG /api/video/{stream_id}",
        ]),
        ("React + TypeScript  ·  :5173", [
            "4분할 영상 · 로봇 카드 · 경보 배너 · 출동 이력",
            "타입은 domain/models.py 와 1:1",
        ]),
    ]
    # 안쪽 상자를 기둥 높이에 고르게 편다. 붙여 쌓으면 아래가 텅 비어
    # 그림이 덜 그려진 것처럼 보인다.
    heights = [34 + LINE * len(lines) + 8 for _, lines in inner]
    room = rh - 58 - 24 - sum(heights)
    step = max(room // (len(inner) - 1), 26)

    iy = ry + 58
    for index, (title, lines) in enumerate(inner):
        height = heights[index]
        c.box(RIGHT_X + 20, iy, RIGHT_W - 40, height)
        c.text(RIGHT_X + 38, iy + 26, title, size=15, weight="700")
        for i, line in enumerate(lines):
            c.text(RIGHT_X + 38, iy + 48 + LINE * i, line, size=13,
                   opacity=0.82)
        iy += height
        if index < len(inner) - 1:
            c.add(
                f'<line x1="{RIGHT_X + RIGHT_W // 2}" y1="{iy}"'
                f' x2="{RIGHT_X + RIGHT_W // 2}" y2="{iy + step - 6}"'
                f' stroke="currentColor" stroke-width="2"'
                f' marker-end="url(#a)"/>'
            )
            iy += step

    # ── 아래: 정의는 했으나 아직 아무도 안 쓰는 것 ──────────────────────
    fy = y_bottom_left + 56
    notes = [
        ("정의만 해 두고 아직 쓰는 노드가 없는 인터페이스", [
            "action/DeliverAed        출동 지시. 지금은 MissionAssignment"
            " topic 이 그 자리를 대신한다",
            "srv/ReportEmergency      119·운영자 좌표 접수",
            "msg/CrowdLevel           혼잡도. 지금은 std_msgs/String 이 나간다",
            "msg/DetectionSummary     프레임당 검출 결과",
            "msg/SensorHealth         라이다 이상과 대체 주행 여부",
        ]),
        ("main 에 없는 것", [
            "multi_robot_emergency    ETA 예상·실측 비교. woduqAMR 브랜치에"
            " 있다",
            "/emergency/eta/result    관제는 이미 구독한다"
            " (std_msgs/String 안의 JSON · TRANSIENT_LOCAL depth 10)",
        ]),
    ]
    for title, lines in notes:
        height = 34 + LINE * len(lines) + 10
        c.box(LEFT_X, fy, W - LEFT_X * 2, height, style="dashed")
        c.text(LEFT_X + 20, fy + 26, title, size=16, weight="700",
               opacity=0.9)
        for i, line in enumerate(lines):
            c.text(LEFT_X + 20, fy + 50 + LINE * i, line, size=14,
                   opacity=0.8, mono=True)
        fy += height + 20

    height = fy + 30
    body = "\n".join(c.parts)
    return (
        f'<svg viewBox="0 0 {W} {height}" role="img"'
        f' aria-label="main 기준 실제 배선. 왼쪽은 ROS 흐름, 오른쪽은 관제.'
        f' 빨간 상자 둘은 발행자가 없어 흐름이 멈추는 자리다.">\n'
        f'<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5"'
        f' markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M0 0 L10 5 L0 10 z" fill="context-stroke"/>'
        f'</marker></defs>\n{body}\n</svg>'
    )


PAGE = """<meta charset="utf-8">
<title>Multi-AMR AED — main 기준 실제 배선</title>
<style>
  :root {{
    --bg:#fff; --fg:#16202b; --dim:#5b6b7d; --red:#c0392b; --line:#c9d3de;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{ --bg:#0f151c; --fg:#dce6f2; --dim:#8496ab; --red:#ff6b6b;
             --line:#2b3846; }}
  }}
  body {{ margin:0; padding:24px; background:var(--bg); color:var(--fg);
          font-family:"Noto Sans KR",-apple-system,"Segoe UI",sans-serif; }}
  svg {{ width:100%; height:auto; }}
  p.hint {{ color:var(--dim); font-size:14px; margin:0 0 16px; }}
</style>
<p class="hint">브라우저에서 Ctrl + 휠 로 확대하면 글자가 깨지지 않습니다
(그림이 SVG 라 배율을 올려도 선명합니다).</p>
{svg}
"""


def main() -> int:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(root, "docs", "system_flow.html")
    with open(out, "w", encoding="utf-8") as handle:
        handle.write(PAGE.format(svg=build()))
    print(f"저장: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
