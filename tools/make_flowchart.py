#!/usr/bin/env python3
"""시스템 아키텍처 + 기능 플로우를 큰 그림 한 장으로 그린다.

형태
----
세로 칸(레인) = 기계 한 대.  가로 띠 = 시나리오 단계.
기계 사이를 건너는 화살표에 **토픽 이름·메시지 타입·QoS·주기를 글로 바로
얹는다.** 상자에 담아 옆에 두지 않는다. 눈이 화살표를 떠날 일이 없어야 한다.

겹치지 않는 이유
----------------
  1. 가로 화살표는 저마다 자기 높이 하나만 쓴다. 높이가 다르면 겹칠 수 없다.
  2. 글은 그 화살표 바로 위에만 놓는다. 다음 화살표까지의 간격을 글 높이보다
     크게 잡아, 아래 화살표의 글이 위 화살표에 닿지 않게 한다.
  3. 레인 안에서 도는 화살표는 그 레인 폭 안에서만 움직인다.

좌표를 눈대중으로 찍으면 반드시 어긋나므로 간격을 코드가 계산하고,
결과는 tools/check_flowchart.py 가 좌표로 다시 확인한다.

내용은 main 에 있는 것만 적는다. 아직 콜백이 빈 뼈대는 그렇게 적는다.
그림이 실제보다 앞서 보이면 리뷰에서 바로 어긋난다.

사용:
  python3 tools/make_flowchart.py     docs/system_flow.{html,drawio} 생성
"""
import html
import os
import sys

# ── 기계(레인) ──────────────────────────────────────────────────────────
LANES = [
    ("웹캠 노트북 ×2", [
        "camera_open (개방구역) · camera_alley (골목)",
        "vision_detector — USB 카메라를 직접 읽는다",
        "YOLO11n rescue_yolo11n.pt (fallen_person · helper)",
    ], "구현"),
    ("중앙 관제 PC  192.168.107.122", [
        "Fast DDS Discovery Server :11811",
        "mission_manager · location_mapper(뼈대)",
        "aed_hmi — FastAPI :8000 + SQLite + aed_hmi_bridge",
    ], "일부"),
    ("로봇 제어 PC ×2", [
        "mission_executor · robot_state_monitor(뼈대)",
        "Nav2 — bt_navigator · planner · controller · AMCL",
        "turtlebot4_navigation nav2.launch.py namespace:=/robotN",
    ], "일부"),
    ("TurtleBot4 ×2", [
        "Create3 + Raspberry Pi",
        "RPLIDAR · OAK-D Pro · 배터리 · Dock",
        "AED 적재",
    ], "하드웨어"),
    ("운영자 브라우저", [
        "React + TypeScript :5173",
        "ROS 미설치",
    ], "구현"),
]

LANE_X = [60, 620, 1240, 1860, 2400]
LANE_W = [500, 560, 560, 480, 400]

W = 2860
HEAD_Y, HEAD_H = 120, 118
BODY_TOP = HEAD_Y + HEAD_H + 44

LINE = 20            # 글줄 높이
GAP_AFTER = 40       # 화살표 아래로 두는 여유
PHASE_PAD = 22


def esc(text: str) -> str:
    return html.escape(text, quote=False)


class Canvas:
    """SVG 조각과 배치 목록을 같이 남긴다. 좌표 계산은 한 번만 한다."""

    def __init__(self) -> None:
        self.parts: list[str] = []
        self.shapes: list[dict] = []
        self.lines: list[dict] = []
        self.bottom = 0

    def add(self, svg: str) -> None:
        self.parts.append(svg)

    def svg_text(self, x, y, s, size=15, weight="400", fill="currentColor",
                 anchor="start", opacity=1.0, mono=False):
        family = ' font-family="ui-monospace,monospace"' if mono else ""
        self.add(
            f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}"'
            f' fill="{fill}" text-anchor="{anchor}" opacity="{opacity}"'
            f'{family}>{esc(s)}</text>'
        )

    def svg_box(self, x, y, w, h, style="solid", accent=None, fill=0.04):
        dash = ' stroke-dasharray="8 5"' if style == "dashed" else ""
        stroke = accent or "currentColor"
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="7"'
            f' fill="currentColor" fill-opacity="{fill}"'
            f' stroke="{stroke}" stroke-width="2"{dash}/>'
        )

    # 아래 셋은 같은 배치를 drawio 로 다시 뽑기 위한 기록이다.
    def keep_box(self, x, y, w, h, title=None, tag=None, lines=(),
                 style="solid", accent=None, size=13, container=False):
        self.shapes.append({
            "kind": "box", "x": x, "y": y, "w": w, "h": h, "title": title,
            "tag": tag, "lines": list(lines), "style": style,
            "accent": accent, "size": size, "container": container,
        })

    def keep_label(self, x, y, w, h, lines, accent=None, size=13):
        """테두리도 채움도 없는 글. 화살표 위에 얹는 설명이다."""
        self.shapes.append({
            "kind": "text", "x": x, "y": y, "w": w, "h": h, "title": None,
            "tag": None, "lines": list(lines), "style": "none",
            "accent": accent, "size": size, "container": False,
        })

    def keep_line(self, x1, y1, x2, y2, accent=None, arrow=True,
                  dashed=False, guide=False) -> None:
        self.lines.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2,
                           "accent": accent, "arrow": arrow,
                           "dashed": dashed, "guide": guide})


def lane_mid(i: int) -> int:
    return LANE_X[i] + LANE_W[i] // 2


def label_height(lines) -> int:
    return LINE * len(lines) + 8


def arrow(c: Canvas, top: int, src: int, dst: int, lines, accent=None,
          dashed=False) -> int:
    """레인 src → dst 가로 화살표. 글은 선 바로 위에 얹는다.

    받는 top 은 이 화살표가 쓸 세로 공간의 **위끝**이다. 선 위치를 받으면
    글이 위로 자라 앞 단계를 침범하는데, 부르는 쪽이 그 높이를 미리 알 수
    없어 매번 어긋난다. 위끝을 받으면 그 계산을 여기서 한다.

    돌려주는 값은 다음 블록의 위끝이다.
    """
    stroke = accent or "currentColor"
    height = label_height(lines)
    y = top + height + 8
    # 레인 가장자리끼리 이으면 붙어 있는 레인 사이가 60px 밖에 안 돼,
    # 글이 화살표 밖으로 한참 삐져나간다. 중심에서 중심으로 긋는다.
    # 레인 몸통은 이 띠 안에서 비어 있으므로 지나가도 가리는 것이 없다.
    right = dst > src
    x1, x2 = lane_mid(src), lane_mid(dst)

    dash = ' stroke-dasharray="7 5"' if dashed else ""
    head_x = x2 - 7 if right else x2 + 7
    c.add(
        f'<line x1="{x1}" y1="{y}" x2="{head_x}" y2="{y}" stroke="{stroke}"'
        f' stroke-width="2.6" marker-end="url(#a)"{dash}/>'
    )
    c.keep_line(x1, y, x2, y, accent, dashed=dashed)

    text_x = min(x1, x2) + 16
    for i, line in enumerate(lines):
        c.svg_text(text_x, top + 16 + LINE * i, line, size=14,
                   weight="700" if i == 0 else "400",
                   fill=stroke if i == 0 else "currentColor",
                   opacity=1.0 if i == 0 else 0.82,
                   mono=line.startswith("/"))
    c.keep_label(text_x, top, abs(x2 - x1) - 32, height, lines, accent)

    return y + GAP_AFTER


def inside(c: Canvas, top: int, lane: int, lines, accent=None) -> int:
    """레인 안에서 도는 일. 짧은 세로 화살표와 그 오른쪽 글.

    arrow 와 같이 top 은 이 블록의 위끝이다.
    """
    stroke = accent or "currentColor"
    x = lane_mid(lane)
    height = label_height(lines)
    y = top + height + 8
    c.add(
        f'<line x1="{x}" y1="{top}" x2="{x}" y2="{y - 7}"'
        f' stroke="{stroke}" stroke-width="2.6" marker-end="url(#a)"/>'
    )
    c.keep_line(x, top, x, y, accent)
    for i, line in enumerate(lines):
        c.svg_text(x + 22, top + 16 + LINE * i, line, size=14,
                   weight="700" if i == 0 else "400",
                   fill=stroke if i == 0 else "currentColor",
                   opacity=1.0 if i == 0 else 0.82,
                   mono=line.startswith("/"))
    c.keep_label(x + 22, top, LANE_W[lane] // 2 + 200, height, lines, accent)
    return y + GAP_AFTER


def build():
    c = Canvas()
    RED = "var(--red)"

    c.svg_text(60, 56, "Multi-AMR AED — 기계 구성과 시나리오 흐름", size=30,
               weight="700")
    c.svg_text(60, 92,
               "세로 칸은 기계 한 대, 가로 띠는 시나리오 단계입니다. "
               "화살표 위의 글이 그 화살표로 오가는 것입니다 — "
               "토픽 이름 · 메시지 타입 · QoS · 주기.", size=16, opacity=0.75)

    # ── 레인 머리 ───────────────────────────────────────────────────────
    for i, (name, lines, status) in enumerate(LANES):
        c.svg_box(LANE_X[i], HEAD_Y, LANE_W[i], HEAD_H, fill=0.09)
        c.svg_text(LANE_X[i] + 18, HEAD_Y + 30, name, size=18, weight="700")
        tag = {"구현": "[구현]", "일부": "[일부 구현]",
               "하드웨어": "[하드웨어]"}[status]
        c.svg_text(LANE_X[i] + LANE_W[i] - 18, HEAD_Y + 30, tag, size=13,
                   weight="700", anchor="end", opacity=0.75)
        for j, line in enumerate(lines):
            c.svg_text(LANE_X[i] + 18, HEAD_Y + 58 + 21 * j, line, size=13,
                       opacity=0.8)
        c.keep_box(LANE_X[i], HEAD_Y, LANE_W[i], HEAD_H, title=name,
                   tag=tag, lines=lines, size=14)

    # ── 단계 띠 ─────────────────────────────────────────────────────────
    phases: list[list] = []
    y = BODY_TOP

    def phase(title: str, note: str) -> int:
        phases.append([title, note, y, None])
        return y + 56                     # 띠 제목 아래에서 시작한다

    def close(cursor: int) -> int:
        phases[-1][3] = cursor - GAP_AFTER + PHASE_PAD
        return phases[-1][3] + 30

    # P0 대기 ───────────────────────────────────────────────────────────
    y = phase("P0  대기", "로봇은 Dock 에서 AED 를 싣고 기다린다")
    y = arrow(c, y, 3, 2, [
        "/{robot_id}/odom  ·  /battery_state  ·  /dock_status  ·  /scan",
        "nav_msgs/Odometry · sensor_msgs/BatteryState · "
        "irobot_create_msgs/DockStatus · sensor_msgs/LaserScan",
        "BEST_EFFORT depth 10 — 센서는 최신값만 의미가 있어 밀리면 버린다",
    ])
    y = arrow(c, y, 2, 1, [
        "/aed/robot_state    ← 발행하는 노드가 없다",
        "aed_interfaces/RobotState · RELIABLE depth 20 · 2 Hz 예정",
        "robot_state_monitor 가 29줄 뼈대라 관제의 로봇 카드가 빈칸으로 남는다",
    ], accent=RED, dashed=True)
    y = arrow(c, y, 1, 4, [
        "WebSocket  ws://호스트:8000/ws/live",
        "SystemSnapshot JSON · 0.25초마다 · 로봇·사건·임무·영상 상태 한 장",
    ])
    y = close(y)

    # P1 감지 ───────────────────────────────────────────────────────────
    y = phase("P1  감지", "쓰러진 사람을 웹캠이 본다")
    y = inside(c, y, 0, [
        "USB 카메라 → vision_detector   같은 프로세스 · DDS 안 거침",
        "cv2.VideoCapture → numpy BGR uint8[480,640,3] · 15 Hz",
        "확정 규칙: 최근 10프레임 중 6프레임 이상 검출 · rescue_conf 0.25",
    ])
    y = arrow(c, y, 0, 1, [
        "/{camera_id}/vision/emergency_event",
        "aed_interfaces/EmergencyEvent · RELIABLE depth 10",
        "상태가 바뀔 때만 (DETECTED → CONFIRMED → RESOLVED)",
        "location.point 는 아직 0,0 — 호모그래피 변환이 안 붙었다",
    ])
    y = arrow(c, y, 0, 1, [
        "/{camera_id}/vision/debug/compressed   ·   /vision/person_count",
        "sensor_msgs/CompressedImage · BEST_EFFORT depth 1 · 15 Hz",
        "std_msgs/UInt32 · RELIABLE depth 10 — 화면 타일의 검출 표시",
    ])
    y = close(y)

    # P2 확정·배정 ──────────────────────────────────────────────────────
    y = phase("P2  확정 · 배정", "중앙이 후보를 세우고 한 대만 보낸다")
    y = inside(c, y, 1, [
        "location_mapper   ← 29줄 뼈대. 여기서 흐름이 끊긴다",
        "카메라별 토픽을 /aed/emergency_event 로 중계해야 한다",
        "mission_manager 는 그 토픽을 듣고 있으나 아무것도 안 온다",
    ], accent=RED)
    y = inside(c, y, 1, [
        "mission_manager._assign_next()",
        "가용: availability=AVAILABLE ∧ network_ok ∧ nav2_ok ∧ path_valid",
        "순위: rank_candidates() — estimated_path_cost 오름차순 · 1대만 배정",
    ])
    y = arrow(c, y, 1, 2, [
        "/{robot_id}/mission_assignment",
        "aed_interfaces/MissionAssignment · RELIABLE depth 10",
        "목표 좌표는 이 메시지에만 실린다 (MissionStatus 에는 상태만 있다)",
        "cancel_previous=true 면 받는 쪽이 기존 Nav2 goal 을 취소한다",
    ])
    y = arrow(c, y, 2, 1, [
        "/aed/mission_status    ASSIGNED → DISPATCHING",
        "aed_interfaces/MissionStatus · RELIABLE depth 10",
        "상태 14종. 한 번만 오므로 놓치면 그 기록이 영영 없다",
    ])
    y = close(y)

    # P3 이동 ───────────────────────────────────────────────────────────
    y = phase("P3  이동", "Nav2 가 목표까지 몬다")
    y = inside(c, y, 2, [
        "/{robot_id}/navigate_to_pose    Topic 아님 · Action",
        "nav2_msgs/action/NavigateToPose",
        "goal PoseStamped(map) / feedback 남은 거리 / result 성공·실패",
    ])
    y = arrow(c, y, 2, 3, [
        "/{robot_id}/cmd_vel",
        "geometry_msgs/Twist · 최대 선속도 0.20 m/s (nav2_aed.yaml)",
    ])
    y = arrow(c, y, 3, 1, [
        "/{robot_id}/oakd/rgb/image_raw/compressed",
        "sensor_msgs/CompressedImage · BEST_EFFORT depth 1",
        "로봇 시점 영상. 로봇 쪽 검출 노드는 아직 없어 원본만 나온다",
    ])
    y = arrow(c, y, 2, 1, [
        "/aed/mission_status    EN_ROUTE",
        "aed_interfaces/MissionStatus · RELIABLE depth 10",
    ])
    y = arrow(c, y, 1, 4, [
        "WebSocket 0.25초   +   MJPEG /api/video/{stream_id}",
        "경보 배너 · 도착 예상(ETA) · 로봇 카드 · 4분할 영상",
        "ETA = 남은 거리 ÷ 실측 속도. 못 믿을 때는 순항 0.20 m/s 로 갈음",
    ])
    y = close(y)

    # P4 재할당 ─────────────────────────────────────────────────────────
    y = phase("P4  장애 · 재할당", "가던 로봇이 못 가면 다른 한 대가 간다")
    y = arrow(c, y, 2, 1, [
        "/aed/mission_status    BLOCKED | NETWORK_LOST | NAVIGATION_ERROR",
        "aed_interfaces/MissionStatus · reason 에 사유 문자열이 실린다",
    ], accent=RED)
    y = arrow(c, y, 1, 2, [
        "/{robot_id}/mission_assignment    assignment_version = 2",
        "aed_interfaces/MissionAssignment · cancel_previous=true",
        "실패한 로봇을 후보에서 빼고 차순위에게 다시 보낸다",
    ], accent=RED)
    y = close(y)

    # P5 도착·종료 ──────────────────────────────────────────────────────
    y = phase("P5  도착 · 종료", "AED 전달이 끝나면 기록이 남는다")
    y = arrow(c, y, 2, 1, [
        "/aed/mission_status    ARRIVED → COMPLETED",
        "aed_interfaces/MissionStatus · RELIABLE depth 10",
    ])
    y = inside(c, y, 1, [
        "context.on_*  —  같은 값이 두 갈래로 간다",
        "Ⓐ 메모리(LiveState) → WebSocket 0.25초      지금 상태",
        "Ⓑ SQLite(Repository) → REST 5초             일어난 일",
    ])
    y = arrow(c, y, 1, 4, [
        "GET /api/missions · /api/stats/response-time · /stats/eta-accuracy",
        "출동 이력과 통계. 임무 요약은 저장하지 않고 상태 전이에서 되짚는다",
    ])
    y = close(y)

    # 레인 세로 안내선. 어느 칸의 일인지 눈으로 따라갈 수 있게 한다.
    # 가로 화살표가 이 선을 지나는 것은 정상이다(생명선).
    guides = []
    for i in range(len(LANES)):
        gx = lane_mid(i)
        guides.append(
            f'<line x1="{gx}" y1="{HEAD_Y + HEAD_H}" x2="{gx}" y2="{y}"'
            f' stroke="currentColor" stroke-width="1.5"'
            f' stroke-dasharray="3 7" stroke-opacity="0.35"/>'
        )
        c.keep_line(gx, HEAD_Y + HEAD_H, gx, y, arrow=False, dashed=True,
                    guide=True)

    # 단계 띠는 배경이므로 뒤에 만들어 앞에 붙인다.
    band = []
    for title, note, top, bottom in phases:
        band.append(
            f'<rect x="40" y="{top - 12}" width="{W - 80}"'
            f' height="{bottom - top + 12}" rx="10"'
            f' fill="currentColor" fill-opacity="0.03"'
            f' stroke="currentColor" stroke-opacity="0.28"'
            f' stroke-width="1.5"/>'
        )
        c.keep_box(40, top - 12, W - 80, bottom - top + 12, title=title,
                   tag=None, lines=[note], style="dashed", size=15,
                   container=True)
    for title, note, top, bottom in phases:
        band.append(
            f'<text x="62" y="{top + 20}" font-size="21" font-weight="700"'
            f' fill="currentColor">{esc(title)}</text>'
        )
        band.append(
            f'<text x="272" y="{top + 20}" font-size="15"'
            f' fill="currentColor" opacity="0.7">{esc(note)}</text>'
        )
    c.parts = band + guides + c.parts

    height = y + 30
    c.bottom = height
    body = "\n".join(c.parts)
    return c, (
        f'<svg viewBox="0 0 {W} {height}" role="img"'
        f' aria-label="기계 다섯 종을 세로 칸으로, 시나리오 여섯 단계를'
        f' 가로 띠로 놓은 그림. 화살표 위의 글이 그 화살표로 오가는 토픽과'
        f' 메시지 타입, QoS 다.">\n'
        f'<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5"'
        f' markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
        f'<path d="M0 0 L10 5 L0 10 z" fill="context-stroke"/>'
        f'</marker></defs>\n{body}\n</svg>'
    )


# ── 두 번째 그림: Nav2 안에서 goal 이 어떻게 굴러가나 ────────────────────
#
# 강사님이 여기를 자세히 본다. 노드 이름·액션 이름·메시지 타입·플러그인
# 클래스·파라미터 값을 코드에서 읽은 그대로 적는다. 값은
# src/aed_bringup/config/nav2_aed.yaml 에서 왔다.

NAV_W = 2860
NAV_COL_X = [60, 780, 1500, 2220]
NAV_COL_W = [660, 660, 660, 580]


def build_nav2():
    c = Canvas()
    RED = "var(--red)"

    c.svg_text(60, 56,
               "Nav2 상세 — goal 하나가 바퀴까지 가는 길", size=30,
               weight="700")
    c.svg_text(60, 92,
               "노드 이름 · 액션/토픽 이름 · 메시지 타입 · 플러그인 클래스 · "
               "파라미터 값을 코드에서 읽은 그대로 적었습니다 "
               "(src/aed_bringup/config/nav2_aed.yaml).", size=16,
               opacity=0.75)

    steps = [
        ("① 목표 접수", "bt_navigator", [
            "액션 서버  /{ns}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            "goal: geometry_msgs/PoseStamped (frame_id=map)",
            "동작 트리 XML: navigate_to_pose_w_replanning_and_recovery.xml",
            "global_frame=map · robot_base_frame=base_link",
            "보내는 쪽은 우리 mission_executor (robot_missions)",
        ], None),
        ("② 경로 계산", "planner_server", [
            "액션 서버  /{ns}/compute_path_to_pose",
            "nav2_msgs/action/ComputePathToPose  →  결과 nav_msgs/Path",
            "플러그인 GridBased = nav2_navfn_planner/NavfnPlanner",
            "tolerance 0.5 m · expected_planner_frequency 20.0 Hz",
            "동작 트리가 1 Hz 로 다시 계획한다(replanning)",
            "실패하면 BT 가 복구 가지로 넘어간다",
        ], None),
        ("③ 전역 지도", "global_costmap", [
            "레이어 static_layer · obstacle_layer · inflation_layer",
            "nav2_costmap_2d::StaticLayer  ← /{ns}/map (nav_msgs/OccupancyGrid)",
            "ObstacleLayer  ← /{ns}/scan (sensor_msgs/LaserScan)",
            "  obstacle_max_range 2.5 m · raytrace_max_range 3.0 m",
            "InflationLayer  inflation_radius 0.22 · cost_scaling_factor 8.0",
            "resolution 0.06 m · robot_radius 0.2 m · update 1.0 Hz",
        ], None),
        ("④ 경로 추종", "controller_server", [
            "액션 서버  /{ns}/follow_path",
            "nav2_msgs/action/FollowPath  ← 위 ②의 nav_msgs/Path",
            "플러그인 FollowPath = dwb_core::DWBLocalPlanner",
            "controller_frequency 20.0 Hz · max_vel_x 0.2 m/s",
            "진행 확인 SimpleProgressChecker",
            "  20초 안에 0.25 m 를 못 가면 실패로 본다",
            "도착 판정 SimpleGoalChecker",
            "  xy_goal_tolerance 0.15 m · yaw_goal_tolerance 0.25 rad",
        ], None),
        ("⑤ 지역 지도", "local_costmap", [
            "global_frame=odom · rolling_window=true · 3 m × 3 m",
            "레이어 static_layer · voxel_layer · inflation_layer",
            "nav2_costmap_2d::VoxelLayer  ← /{ns}/scan",
            "update 5.0 Hz · publish 2.0 Hz · resolution 0.06 m",
            "지역 지도는 map 이 아니라 odom 을 쓴다. 위치추정이 잠깐 튀어도",
            "장애물 회피가 흔들리지 않게 하기 위해서다",
        ], None),
        ("⑥ 속도 다듬기", "velocity_smoother", [
            "입력  /{ns}/cmd_vel_nav   geometry_msgs/Twist",
            "출력  /{ns}/cmd_vel       geometry_msgs/Twist",
            "급가감속을 깎아 낸다. Create3 가 이 값을 그대로 받는다",
        ], None),
        ("⑦ 위치추정", "amcl", [
            "입력  /{ns}/scan (LaserScan) · /{ns}/map (OccupancyGrid)",
            "       odom→base_link TF (Create3 가 낸다)",
            "출력  map→odom TF · /{ns}/amcl_pose",
            "       geometry_msgs/PoseWithCovarianceStamped",
            "초기 위치는 dock_poses.yaml 실측값을 /initialpose 로 넣는다",
            "  robot1 (-0.576, 0.137, 5.3°) · robot2 (-0.047, 0.049, 186.4°)",
            "설정은 turtlebot4_navigation 기본값을 쓴다",
        ], None),
        ("⑧ 복구 행동", "behavior_server", [
            "global_frame=odom · robot_base_frame=base_link",
            "spin  nav2_behaviors/Spin        제자리 회전으로 시야 확보",
            "backup  nav2_behaviors/BackUp    뒤로 물러난다",
            "drive_on_heading · wait · assisted_teleop",
            "BT 가 경로 계산이나 추종에 실패했을 때 부른다",
            "그래도 안 되면 NavigateToPose result 가 실패로 확정된다",
        ], RED),
    ]

    # 두 칸씩 네 줄로 놓는다. 화살표는 칸 사이 가로선 하나씩만 쓴다.
    y = 140
    positions = {}
    row_height = 0
    for index, (num, node_name, lines, accent) in enumerate(steps):
        col = index % 2
        if col == 0 and index:
            y += row_height + 60
            row_height = 0
        x = NAV_COL_X[col * 2]
        w = NAV_COL_W[col * 2]
        height = 74 + LINE * len(lines)
        row_height = max(row_height, height)

        c.svg_box(x, y, w, height, accent=accent)
        c.svg_text(x + 20, y + 32, num, size=19, weight="700",
                   fill=accent or "currentColor")
        c.svg_text(x + 20, y + 58, node_name, size=17, weight="700",
                   fill=accent or "currentColor", mono=True)
        for i, line in enumerate(lines):
            c.svg_text(x + 20, y + 84 + LINE * i, line, size=14, opacity=0.85,
                       mono=line.strip().startswith("/"))
        c.keep_box(x, y, w, height, title=f"{num}   {node_name}",
                   lines=lines, accent=accent, size=14)
        positions[index] = (x, y, w, height)

    bottom = y + row_height + 60

    # 흐름 화살표. 같은 줄 안에서는 가로, 줄이 바뀌면 아래로 내려간다.
    order = [(0, 1), (2, 3), (4, 5), (6, 7)]
    for left, right in order:
        if left in positions and right in positions:
            lx, ly, lw, lh = positions[left]
            rx, ry, rw, rh = positions[right]
            mid = ly + min(lh, rh) // 2
            c.add(
                f'<line x1="{lx + lw}" y1="{mid}" x2="{rx - 7}" y2="{mid}"'
                f' stroke="currentColor" stroke-width="2.6"'
                f' marker-end="url(#a)"/>'
            )
            c.keep_line(lx + lw, mid, rx, mid)

    c.svg_text(60, bottom - 20,
               "②③ 는 한 쌍이고 ④⑤ 도 한 쌍이다. planner 는 global_costmap 을,"
               " controller 는 local_costmap 을 보고 판단한다.",
               size=15, opacity=0.75)

    c.bottom = bottom
    body = "\n".join(c.parts)
    return c, (
        f'<svg viewBox="0 0 {NAV_W} {bottom}" role="img"'
        f' aria-label="Nav2 안에서 goal 이 경로 계산과 추종을 거쳐 cmd_vel'
        f' 로 나가기까지의 노드와 액션, 플러그인.">\n'
        f'<defs><marker id="b" viewBox="0 0 10 10" refX="9" refY="5"'
        f' markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
        f'<path d="M0 0 L10 5 L0 10 z" fill="context-stroke"/>'
        f'</marker></defs>\n{body}\n</svg>'.replace("url(#a)", "url(#b)")
    )


# ── drawio 로 뽑기 ──────────────────────────────────────────────────────
DRAWIO_PAGE = (
    '<diagram name="{name}">'
    '<mxGraphModel dx="1400" dy="900" grid="1" gridSize="10" guides="1"'
    ' tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1"'
    ' pageWidth="{w}" pageHeight="{h}" math="0" shadow="0">'
    '<root><mxCell id="0"/><mxCell id="1" parent="0"/>'
)
DRAWIO_PAGE_TAIL = "</root></mxGraphModel></diagram>"


def _xml(text: str) -> str:
    return (text.replace("&", "&amp;").replace("<", "&lt;")
                .replace(">", "&gt;").replace('"', "&quot;"))


def _value(title, tag, lines) -> str:
    rows = []
    if title:
        head = f"&lt;b&gt;{_xml(title)}&lt;/b&gt;"
        if tag:
            head += f"　{_xml(tag)}"
        rows.append(head)
    for index, line in enumerate(lines):
        if title is None and index == 0:
            rows.append(f"&lt;b&gt;{_xml(line)}&lt;/b&gt;")
        else:
            rows.append(_xml(line))
    return "&lt;br&gt;".join(rows)


def _page(canvas: Canvas, name: str, prefix: str) -> str:
    cells = []
    for index, shape in enumerate(canvas.shapes):
        colour = "#C0392B" if shape["accent"] else "#333333"
        if shape["kind"] == "text":
            # 테두리도 채움도 없는 글. 화살표 위에 얹는 설명이다.
            style = (
                f"text;html=1;align=left;verticalAlign=top;"
                f"fillColor=none;strokeColor=none;spacing=0;"
                f"fontSize={shape['size']};fontColor={colour};"
            )
        else:
            dashed = 1 if shape["style"] == "dashed" else 0
            fill = "none" if shape["container"] else "#FFFFFF"
            style = (
                f"rounded=1;arcSize=6;whiteSpace=wrap;html=1;align=left;"
                f"verticalAlign=top;spacing=10;spacingLeft=6;spacingTop=4;"
                f"fillColor={fill};strokeColor={colour};strokeWidth=2;"
                f"dashed={dashed};dashPattern=8 5;"
                f"fontSize={shape['size']};fontColor={colour};"
            )
        cells.append(
            f'<mxCell id="{prefix}s{index}"'
            f' value="{_value(shape["title"], shape["tag"], shape["lines"])}"'
            f' style="{style}" vertex="1" parent="1">'
            f'<mxGeometry x="{shape["x"]}" y="{shape["y"]}"'
            f' width="{shape["w"]}" height="{shape["h"]}" as="geometry"/>'
            f"</mxCell>"
        )

    for index, line in enumerate(canvas.lines):
        colour = "#C0392B" if line["accent"] else "#333333"
        end = "block" if line.get("arrow", True) else "none"
        dashed = 1 if line.get("dashed") else 0
        if line.get("guide"):
            # 레인 안내선. 검사기가 이 무늬로 알아본다.
            style = (
                f"endArrow=none;html=1;rounded=0;strokeColor=#9AA7B4;"
                f"strokeWidth=1.5;edgeStyle=none;dashed=1;dashPattern=3 7;"
            )
        else:
            style = (
                f"endArrow={end};endFill=1;endSize=6;html=1;rounded=0;"
                f"strokeColor={colour};strokeWidth=2.6;edgeStyle=none;"
                f"dashed={dashed};dashPattern=8 5;"
            )
        cells.append(
            f'<mxCell id="e{index}" style="{style}" edge="1" parent="1">'
            f'<mxGeometry relative="1" as="geometry">'
            f'<mxPoint x="{line["x1"]}" y="{line["y1"]}" as="sourcePoint"/>'
            f'<mxPoint x="{line["x2"]}" y="{line["y2"]}" as="targetPoint"/>'
            f"</mxGeometry></mxCell>"
        )

    head = DRAWIO_PAGE.format(name=name, w=W, h=canvas.bottom)
    return head + "".join(cells) + DRAWIO_PAGE_TAIL


def render_drawio(*pages) -> str:
    names = ["기계 구성과 시나리오 흐름", "Nav2 상세"]
    body = "".join(
        _page(canvas, names[i], f"p{i}")
        for i, canvas in enumerate(pages)
    )
    return "<mxfile host=\"app.diagrams.net\">" + body + "</mxfile>"


PAGE = """<meta charset="utf-8">
<title>Multi-AMR AED — 기계 구성과 시나리오 흐름</title>
<style>
  :root {{
    --bg:#fff; --fg:#16202b; --dim:#5b6b7d; --red:#c0392b;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{ --bg:#0f151c; --fg:#dce6f2; --dim:#8496ab; --red:#ff6b6b; }}
  }}
  body {{ margin:0; padding:20px; background:var(--bg); color:var(--fg);
          font-family:"Noto Sans KR",-apple-system,"Segoe UI",sans-serif; }}
  svg {{ width:100%; height:auto; }}
  p.hint {{ color:var(--dim); font-size:14px; margin:0 0 14px; }}
</style>
<p class="hint">Ctrl + 휠 로 확대해도 글자가 깨지지 않습니다.
고쳐 쓰시려면 docs/system_flow.drawio 를 draw.io 로 여세요.</p>
{svg}
"""


def main() -> int:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    canvas, svg = build()
    nav_canvas, nav_svg = build_nav2()

    html_path = os.path.join(root, "docs", "system_flow.html")
    with open(html_path, "w", encoding="utf-8") as handle:
        handle.write(PAGE.format(svg=svg + "\n<hr>\n" + nav_svg))

    drawio_path = os.path.join(root, "docs", "system_flow.drawio")
    with open(drawio_path, "w", encoding="utf-8") as handle:
        handle.write(render_drawio(canvas, nav_canvas))

    print(f"저장: {html_path}")
    print(f"저장: {drawio_path}")
    print(f"      1쪽 {W} x {canvas.bottom} · 도형 {len(canvas.shapes)} · "
          f"화살표 {len(canvas.lines)}")
    print(f"      2쪽 {NAV_W} x {nav_canvas.bottom} · "
          f"도형 {len(nav_canvas.shapes)} · 화살표 {len(nav_canvas.lines)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
