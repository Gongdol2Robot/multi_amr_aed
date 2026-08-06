#!/usr/bin/env python3
"""조원이 그린 설계도를 통째로 키우고, 화살표에 자료형을 박아 넣는다.

원본은 11800 x 3300 에 글자가 8~9pt 다. 화면에 맞추면 글자가 뭉개지고,
읽으려고 확대하면 전체가 안 보인다.

여기서는 배치를 하나도 안 건드린다. 좌표와 글자 크기를 같은 배율로 키우기만
하므로 서로의 위치 관계가 그대로다 — 새로 겹칠 일이 없다.

그 위에 두 가지를 더 한다.

  1. 화살표 라벨에 **통신 방식과 노드 이름**을 박는다. 자료형은 원본에
     이미 적혀 있는 것이 많으므로(예: "[ROS2] /aed/heartbeat Heartbeat"),
     그 자리에 없을 때만 넣는다. 같은 말을 두 번 적으면 라벨만 길어진다.
     원본에 없는 것은 Topic 인지 Service 인지 Action 인지, 그리고 누가
     발행하고 누가 구독하는지다. 눈이 화살표를 떠나지 않게 거기 적는다.
  2. main 에 실제로 도는 코드인지 표시한다. 29줄 뼈대뿐인 패키지는 그렇게
     적는다. 그림이 실제보다 앞서 보이면 리뷰에서 바로 어긋난다.

사용:
  python3 tools/enlarge_flowchart.py
  python3 tools/enlarge_flowchart.py --scale 4
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

SRC = "docs/multi_amr_aed_flowchart_actual_topics.drawio"
DST = "docs/system_flow_big.drawio"

DEFAULT_FONT = 12       # drawio 가 fontSize 없을 때 쓰는 값

# 인터페이스 이름 → (짧은 자료형 이름, 덧붙일 줄들)
#   짧은 이름이 원본 라벨에 이미 있으면 자료형 줄은 건너뛴다.
# 노드 이름은 코드에서 읽은 그대로다.
#   mission_manager/manager_node.py · robot_missions/mission_executor.py
#   aed_vision/vision_detector.py · aed_hmi/backend/ros/bridge.py
CONTRACT = [
    ("/aed/robot_state", "RobotState",
     "Topic · aed_interfaces/RobotState",
     "발행 robot_state_monitor  →  구독 mission_manager · aed_hmi_bridge"),
    ("/aed/mission_status", "MissionStatus",
     "Topic · aed_interfaces/MissionStatus",
     "발행 mission_manager · mission_executor  →  "
     "구독 mission_manager · aed_hmi_bridge"),
    ("/aed/emergency_event", "EmergencyEvent",
     "Topic · aed_interfaces/EmergencyEvent",
     "발행 location_mapper(예정)  →  구독 mission_manager · aed_hmi_bridge"),
    ("/robotX/mission_assignment", "MissionAssignment",
     "Topic · aed_interfaces/MissionAssignment  (목표 좌표가 실린다)",
     "발행 mission_manager  →  구독 mission_executor · aed_hmi_bridge"),
    ("/aed/heartbeat", "Heartbeat",
     "Topic · aed_interfaces/Heartbeat",
     "발행 amr_recovery  →  구독 amr_recovery (양쪽)"),
    ("/vision/status", "DetectionSummary",
     "Topic · aed_interfaces/DetectionSummary",
     "발행 vision_detector  →  구독 emergency_location_mapper"),
    ("image_raw/compressed", "CompressedImage",
     "Topic · sensor_msgs/CompressedImage",
     "발행 webcam_publisher · OAK-D 드라이버  →  "
     "구독 vision_detector · aed_hmi_bridge"),
    ("/scan", "LaserScan",
     "Topic · sensor_msgs/LaserScan",
     "발행 RPLIDAR 드라이버  →  구독 Nav2 costmap · amcl"),
    ("/odom", "Odometry",
     "Topic · nav_msgs/Odometry",
     "발행 Create3  →  구독 Nav2 controller_server · amcl"),
    ("/battery_state", "BatteryState",
     "Topic · sensor_msgs/BatteryState",
     "발행 Create3  →  구독 robot_state_monitor"),
    ("cmd_vel", "Twist",
     "Topic · geometry_msgs/Twist",
     "발행 Nav2 velocity_smoother  →  구독 Create3"),
    ("Nav2 Goal", "NavigateToPose",
     "Action · nav2_msgs/action/NavigateToPose",
     "클라이언트 mission_executor  →  서버 Nav2 bt_navigator"),
    ("WebSocket", "SystemSnapshot",
     "WebSocket · SystemSnapshot JSON",
     "발행 aed_hmi FastAPI  →  구독 브라우저(React)"),
    ("Undock", "irobot_create_msgs/action/Undock",
     "Action · irobot_create_msgs/action/Undock",
     "클라이언트 mission_executor  →  서버 Create3"),
    ("Dock", "irobot_create_msgs/action/Dock",
     "Action · irobot_create_msgs/action/Dock",
     "클라이언트 mission_executor  →  서버 Create3"),
    ("Twist(0)", "geometry_msgs/Twist",
     "Topic · geometry_msgs/Twist (전 축 0 · 즉시 정지)",
     "발행 sensor_recovery  →  구독 Create3"),
    ("ReportEmergency", "aed_interfaces/ReportEmergency",
     "Service · aed_interfaces/ReportEmergency  (정의만 · 쓰는 노드 없음)",
     "클라이언트 aed_hmi  →  서버 location_mapper(예정)"),
    ("[SQL]", "SQLite",
     "SQLite · repository.py 안에서만 쓴다",
     "쓰기 aed_hmi_bridge 스레드  →  읽기 FastAPI 스레드"),
]

# main 기준 구현 상태. 29줄 뼈대는 콜백이 비어 있다.
STATUS = [
    ("vision_detector", "구현"),
    ("aed_vision", "구현"),
    ("mission_manager", "구현"),
    ("mission_executor", "구현"),
    ("aed_hmi", "구현"),
    ("webcam_publisher", "구현"),
    ("emergency_location_mapper", "뼈대"),
    ("location_mapper", "뼈대"),
    ("robot_state_monitor", "뼈대"),
    ("amr_recovery", "뼈대"),
    ("sensor_recovery", "뼈대"),
    ("helper_mission", "뼈대"),
    ("event_logger", "뼈대"),
    ("multi_robot_emergency", "main 에 없음"),
]

SHORT = {"구현": "구현", "뼈대": "뼈대", "main 에 없음": "없음"}

TAG = {
    "구현": " 〔main: 구현〕",
    "뼈대": " 〔main: 29줄 뼈대 · 콜백 없음〕",
    "main 에 없음": " 〔main 에 없음 · woduqAMR 브랜치〕",
}


def scale_geometry(model: ET.Element, factor: float) -> None:
    """좌표와 크기를 같은 배율로 키운다. 배치 관계는 그대로다."""
    for tag in ("mxGeometry", "mxPoint"):
        for node in model.iter(tag):
            for key in ("x", "y", "width", "height"):
                value = node.get(key)
                if value is None:
                    continue
                node.set(key, str(round(float(value) * factor)))


def scale_fonts(model: ET.Element, factor: float) -> int:
    """글자 크기를 키운다. 없으면 drawio 기본값에서 시작한다."""
    changed = 0
    for cell in model.iter("mxCell"):
        style = cell.get("style")
        if style is None:
            continue
        if "fontSize=" in style:
            style = re.sub(
                r"fontSize=(\d+(?:\.\d+)?)",
                lambda m: f"fontSize={round(float(m.group(1)) * factor)}",
                style,
            )
        else:
            style = style.rstrip(";") + f";fontSize={round(DEFAULT_FONT * factor)};"
        cell.set("style", style)
        changed += 1
    return changed


def enrich_edges(model: ET.Element) -> int:
    """화살표 라벨에 메시지 타입과 QoS 를 덧붙인다."""
    changed = 0
    for cell in model.iter("mxCell"):
        if cell.get("edge") != "1":
            continue
        value = cell.get("value")
        if not value:
            continue
        plain = re.sub(r"<[^>]+>", " ", value)
        extras = []
        for key, short, type_note, role_note in CONTRACT:
            if key not in plain:
                continue
            # 자료형이 원본 라벨에 이미 있으면 다시 적지 않는다.
            if short not in plain and type_note not in value:
                extras.append(type_note)
            elif short in plain and role_note not in value:
                # 자료형은 있으나 통신 방식이 없다. 방식만 앞에 붙인다.
                kind = type_note.split(" · ")[0]
                if kind not in plain:
                    extras.append(f"{kind} · {short}")
            if role_note not in value:
                extras.append(role_note)
        if not extras:
            continue
        # 한 줄씩 아래에 덧붙인다. 화살표 라벨이라 상자가 없어 넘칠 걱정이 없다.
        cell.set("value", value + "<br>" + "<br>".join(extras))
        changed += 1
    return changed


def mark_status(model: ET.Element) -> int:
    """노드 라벨에 main 기준 구현 상태를 붙인다."""
    changed = 0
    for cell in model.iter("mxCell"):
        if cell.get("vertex") != "1":
            continue
        value = cell.get("value")
        if not value:
            continue
        plain = re.sub(r"<[^>]+>", " ", value)
        if "〔main" in value:
            continue
        # 한 상자가 여러 패키지를 적어 두는 경우가 많다. 첫 하나만 보고
        # 상태를 붙이면 "mission_executor · robot_state_monitor · amr_recovery"
        # 가 통째로 구현된 것처럼 보인다. 나온 것을 전부 적는다.
        found = [(name, status) for name, status in STATUS if name in plain]
        if not found:
            continue
        # 긴 이름이 짧은 이름을 품는 경우를 정리한다
        # (emergency_location_mapper 안에 location_mapper 가 들어 있다).
        names = [n for n, _ in found]
        found = [(n, s) for n, s in found
                 if not any(n != other and n in other for other in names)]

        if len({status for _, status in found}) == 1:
            cell.set("value", value + "<br>" + TAG[found[0][1]])
        else:
            detail = " · ".join(f"{n}={SHORT[s]}" for n, s in found)
            cell.set("value", value + "<br> 〔main: " + detail + "〕")
        changed += 1
    return changed


def drop_legend(model: ET.Element) -> int:
    """범례 상자를 뺀다. 정보를 화살표에 넣었으므로 찾아볼 일이 없다."""
    root = model.find("root")
    dropped = 0
    for cell in list(root):
        value = cell.get("value") or ""
        plain = re.sub(r"<[^>]+>", " ", value)
        if "통신 범례" in plain:
            root.remove(cell)
            dropped += 1
    return dropped


def main() -> int:
    parser = argparse.ArgumentParser(description="설계도를 통째로 키운다")
    parser.add_argument("--scale", type=float, default=3.0)
    parser.add_argument("--src", default=SRC)
    parser.add_argument("--out", default=DST)
    args = parser.parse_args()

    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    src = os.path.join(root_dir, args.src)
    out = os.path.join(root_dir, args.out)
    if not os.path.exists(src):
        print(f"실패: {src} 가 없습니다")
        return 1

    tree = ET.parse(src)
    model = tree.getroot().find(".//mxGraphModel")

    before = (model.get("pageWidth"), model.get("pageHeight"))
    scale_geometry(model, args.scale)
    for key in ("pageWidth", "pageHeight"):
        if model.get(key):
            model.set(key, str(round(float(model.get(key)) * args.scale)))

    fonts = scale_fonts(model, args.scale)
    edges = enrich_edges(model)
    nodes = mark_status(model)
    legend = drop_legend(model)

    tree.write(out, encoding="utf-8", xml_declaration=False)

    print(f"원본 {before[0]} x {before[1]}  →  "
          f"{model.get('pageWidth')} x {model.get('pageHeight')}  "
          f"({args.scale}배)")
    print(f"  글자 키운 셀   {fonts}개")
    print(f"  자료형 박은 화살표 {edges}개")
    print(f"  상태 표시한 노드  {nodes}개")
    print(f"  뺀 범례        {legend}개")
    print(f"저장: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
