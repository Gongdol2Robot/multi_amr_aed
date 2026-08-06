#!/usr/bin/env python3
"""rosbag 을 틀어 놓고 관제 화면이 실제 ROS 로 도는 것을 보여주는 발행기.

왜 이것이 필요한가
------------------
`--mock` 은 백엔드 안에서 도메인 객체를 바로 만들어 넣는다. 화면은 돌지만
`ros/bridge.py` 와 `ros/converters.py` 를 **건너뛴다**. 그래서 목업만으로는
"메시지 타입과 QoS 가 맞는가, 구독이 실제로 붙는가" 를 하나도 못 보인다.

이 도구는 반대로 간다. 진짜 ROS 토픽에 진짜 `aed_interfaces` 메시지를
발행하고, 관제는 평소대로 구독한다. 로봇이 없어도 구독 경로 전체가 돈다.

무엇이 진짜이고 무엇이 지어낸 값인가
-----------------------------------
섞어 쓰면 리뷰에서 서로 믿을 수 없게 되므로 여기서 분명히 한다.

* **녹화된 실제 값** — 위치, 방위, 속도, 배터리, 도킹 여부.
  `rosbag` 의 `/{robot}/odom`, `/battery_state`, `/dock_status` 를 읽어
  `RobotState` 로 옮긴다. 이 변환은 `robot_state_monitor` 가 할 일을
  그대로 하는 것이라, 그 노드가 채워지면 이 도구는 버린다.
* **지어낸 값** — 출동 시나리오(신고·배정·이동·재할당·도착).
  그 흐름을 담은 bag 이 없다. 대신 실제 상태 전이 순서와 같은 값을
  같은 메시지 타입으로 낸다.

영상은 bag 이 그대로 발행하므로 관제가 받는 그림도 실제 녹화본이다.

사용:
  # 터미널 1 — 로봇 주행 bag (odom/battery/dock/영상)
  ros2 bag play bags/robot1_map_0806_1846 --loop
  # 터미널 2 — 웹캠 검출 bag
  ros2 bag play bags/camera_open_0806_1900 --loop
  # 터미널 3
  python3 tools/demo_publisher.py
  # 터미널 4
  cd src/aed_hmi && python3 -m backend.main

  tools/demo.sh 가 위 넷을 한 번에 띄운다.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from aed_interfaces.msg import (
    EmergencyEvent,
    MissionAssignment,
    MissionStatus,
    RobotState,
)
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

# 관제(`backend/ros/topics.py`)의 state_qos 와 같아야 한다. 다르면 연결이
# 아예 안 맺어지고 경고도 없다.
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# bag 의 센서 토픽은 BEST_EFFORT 로 녹화돼 있다. 구독을 RELIABLE 로 두면
# 붙지 않는다.
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

ROBOT_IDS = ("robot1", "robot2")
STATE_HZ = 10.0

# 웹캠이 보는 구역의 중심. homography_cam1/cam2.yaml 의 측량 영역에서 왔다.
SCENE_TARGETS = ((1.99, 2.30), (-2.44, 1.84))

# 세 번에 한 번은 막아 재할당 화면을 보여준다. 무작위로 두면 시연 중에
# "다음 건에서 재할당이 납니다" 라고 미리 짚을 수 없다.
REASSIGN_EVERY = 3


def yaw_from_quaternion(q) -> float:
    return math.degrees(
        math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    )


class DemoPublisher(Node):
    """bag 의 로봇 상태를 RobotState 로 옮기고, 출동 시나리오를 얹는다."""

    def __init__(self, cycle_pause: float) -> None:
        super().__init__("aed_demo_publisher")
        self._cycle_pause = cycle_pause

        self.robot_state_pub = self.create_publisher(
            RobotState, "/aed/robot_state", STATE_QOS
        )
        self.mission_pub = self.create_publisher(
            MissionStatus, "/aed/mission_status", STATE_QOS
        )
        self.event_pub = self.create_publisher(
            EmergencyEvent, "/aed/emergency_event", STATE_QOS
        )
        # 목표 좌표는 이 메시지에만 실린다. MissionStatus 에는 상태만 있어서,
        # 이걸 안 내면 관제의 목표 좌표와 도착 예상이 빈 채로 남는다.
        self.assignment_pubs = {
            robot_id: self.create_publisher(
                MissionAssignment, f"/{robot_id}/mission_assignment", STATE_QOS
            )
            for robot_id in ROBOT_IDS
        }

        # bag 에서 온 마지막 값. 로봇마다 따로 담는다.
        self._latest: dict[str, dict] = {
            robot_id: {
                "pose": None, "speed": 0.0, "battery": 100.0,
                "docked": True, "stamp": 0.0,
            }
            for robot_id in ROBOT_IDS
        }
        self._lock = threading.Lock()

        for robot_id in ROBOT_IDS:
            self.create_subscription(
                Odometry, f"/{robot_id}/odom",
                lambda m, r=robot_id: self._on_odom(r, m), SENSOR_QOS,
            )
            self.create_subscription(
                BatteryState, f"/{robot_id}/battery_state",
                lambda m, r=robot_id: self._on_battery(r, m), SENSOR_QOS,
            )
            self.create_subscription(
                DockStatus, f"/{robot_id}/dock_status",
                lambda m, r=robot_id: self._on_dock(r, m), SENSOR_QOS,
            )

        # 임무 상태는 시나리오 스레드가 정하고, 상태 발행은 타이머가 한다.
        # 둘을 한 곳에서 하면 시나리오가 sleep 하는 동안 로봇 상태가 끊긴다.
        self._mission: dict[str, dict] = {
            robot_id: {"availability": RobotState.AVAILABLE,
                       "role": RobotState.ROLE_NONE,
                       "mission_id": ""}
            for robot_id in ROBOT_IDS
        }
        self.create_timer(1.0 / STATE_HZ, self._publish_states)

        self._sequence = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run_scenarios, name="scenario", daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            "발행 시작 — /aed/robot_state, /aed/mission_status, "
            "/aed/emergency_event"
        )

    # ------------------------------------------------------------------
    # bag 에서 오는 실제 값
    # ------------------------------------------------------------------

    def _on_odom(self, robot_id: str, message: Odometry) -> None:
        with self._lock:
            entry = self._latest[robot_id]
            entry["pose"] = message.pose.pose
            # 뒤로 갈 때도 화면에는 속력으로 보여야 하므로 절댓값을 쓴다.
            entry["speed"] = abs(float(message.twist.twist.linear.x))
            entry["stamp"] = time.time()

    def _on_battery(self, robot_id: str, message: BatteryState) -> None:
        with self._lock:
            # Create3 는 0~1 로 낸다. 화면은 백분율을 쓴다.
            self._latest[robot_id]["battery"] = float(message.percentage) * 100.0

    def _on_dock(self, robot_id: str, message: DockStatus) -> None:
        with self._lock:
            self._latest[robot_id]["docked"] = bool(message.is_docked)

    # ------------------------------------------------------------------
    # RobotState 발행
    # ------------------------------------------------------------------

    def _publish_states(self) -> None:
        now = self.get_clock().now().to_msg()
        with self._lock:
            snapshot = {r: dict(v) for r, v in self._latest.items()}
            mission = {r: dict(v) for r, v in self._mission.items()}

        for robot_id in ROBOT_IDS:
            entry = snapshot[robot_id]
            if entry["pose"] is None:
                # bag 이 아직 안 돌았다. 없는 값을 0,0 으로 내면 화면에
                # 로봇이 원점에 있는 것처럼 보이므로 아무것도 안 낸다.
                continue

            message = RobotState()
            message.robot_id = robot_id
            message.stamp = now
            message.last_heartbeat = now

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "map"
            pose.pose = entry["pose"]
            message.pose = pose

            message.speed_mps = float(entry["speed"])
            message.battery_percentage = float(entry["battery"])
            message.is_docked = bool(entry["docked"])

            state = mission[robot_id]
            message.availability = state["availability"]
            message.role = state["role"]
            message.mission_id = state["mission_id"]

            # 아래 넷은 bag 에 없다. 정상값으로 둔다. 이 값을 실제로 채우는
            # 것은 robot_state_monitor 와 sensor_recovery 의 몫이다.
            message.network_ok = True
            message.localization_ok = True
            message.nav2_ok = True
            message.emergency_stop = False
            message.path_valid = True
            message.estimated_path_cost = self._path_cost(robot_id, entry)
            message.detail = ""

            self.robot_state_pub.publish(message)

    def _path_cost(self, robot_id: str, entry: dict) -> float:
        """목표까지 남은 거리. 화면의 도착 예상이 이 값을 쓴다.

        Nav2 가 내는 경로 비용을 대신하는 값이라 직선거리다. 실제 경로는
        이보다 길기 때문에 관제는 여기에 우회 계수를 곱한다.
        """
        state = self._mission[robot_id]
        target = state.get("target")
        if target is None or entry["pose"] is None:
            return -1.0
        position = entry["pose"].position
        return float(math.hypot(target[0] - position.x, target[1] - position.y))

    # ------------------------------------------------------------------
    # 출동 시나리오
    # ------------------------------------------------------------------

    def _run_scenarios(self) -> None:
        # bag 이 첫 odom 을 낼 때까지 기다린다. 위치를 모르는 채로 출동을
        # 시작하면 화면의 거리와 예상 시간이 뜻 없는 값이 된다.
        while not self._stop.is_set():
            with self._lock:
                ready = all(
                    self._latest[r]["pose"] is not None for r in ROBOT_IDS
                )
            if ready:
                break
            time.sleep(0.5)
        else:
            return

        self.get_logger().info("bag 위치 수신 확인 — 시나리오 시작")
        while not self._stop.is_set():
            try:
                self._one_scenario()
            except Exception as error:      # 시나리오가 죽어도 상태는 계속
                self.get_logger().warning(f"시나리오 실패: {error}")
                time.sleep(1.0)

    def _one_scenario(self) -> None:
        self._sequence += 1
        event_id = f"evt-{self._sequence:04d}"
        mission_id = f"{event_id}-aed"
        target = SCENE_TARGETS[self._sequence % len(SCENE_TARGETS)]

        # ① 웹캠이 본다. 한 프레임으로 확정하지 않고 연속 검출이 쌓인다.
        camera = "camera_open" if target[0] > 0 else "camera_alley"
        detected_at = self.get_clock().now().to_msg()
        for count in range(1, 4):
            self._publish_event(
                event_id, detected_at, target, camera,
                EmergencyEvent.DETECTED, 0.55 + 0.12 * count, count,
            )
            if self._sleep(0.8):
                return
        self._publish_event(
            event_id, detected_at, target, camera,
            EmergencyEvent.CONFIRMED, 0.91, 4,
        )

        # ② 가까운 로봇을 고른다. mission_manager 의 규칙과 같다.
        chosen = self._nearest(target)
        other = next(r for r in ROBOT_IDS if r != chosen)

        self._set_mission(chosen, RobotState.BUSY,
                          RobotState.ROLE_AED_DELIVERY, mission_id, target)
        self._publish_assignment(mission_id, event_id, chosen, target, 1)
        self._publish_mission(mission_id, event_id, chosen,
                              MissionStatus.ASSIGNED)
        self._publish_mission(mission_id, event_id, chosen,
                              MissionStatus.DISPATCHING)
        self._publish_mission(mission_id, event_id, chosen,
                              MissionStatus.EN_ROUTE)

        blocked = self._sequence % REASSIGN_EVERY == 0
        if self._sleep(6.0 if blocked else 14.0):
            return

        if blocked:
            # ③ 경로 장애 → 다른 로봇으로 재할당
            self._publish_mission(mission_id, event_id, chosen,
                                  MissionStatus.BLOCKED, "경로가 반복 실패")
            self._set_mission(chosen, RobotState.AVAILABLE,
                              RobotState.ROLE_NONE, "")
            chosen = other
            self._set_mission(chosen, RobotState.BUSY,
                              RobotState.ROLE_AED_DELIVERY, mission_id, target)
            self._publish_assignment(mission_id, event_id, chosen, target, 2)
            self._publish_mission(mission_id, event_id, chosen,
                                  MissionStatus.EN_ROUTE, version=2)
            if self._sleep(14.0):
                return

        # ④ 도착
        self._publish_mission(mission_id, event_id, chosen,
                              MissionStatus.ARRIVED)
        if self._sleep(4.0):
            return

        # ⑤ 복귀와 종료
        self._set_mission(chosen, RobotState.BUSY,
                          RobotState.ROLE_RETURN, mission_id)
        if self._sleep(6.0):
            return
        self._publish_mission(mission_id, event_id, chosen,
                              MissionStatus.COMPLETED)
        self._set_mission(chosen, RobotState.AVAILABLE,
                          RobotState.ROLE_NONE, "")
        self._publish_event(
            event_id, detected_at, target, camera,
            EmergencyEvent.RESOLVED, 0.91, 4,
        )
        self._sleep(self._cycle_pause)

    # ------------------------------------------------------------------

    def _nearest(self, target) -> str:
        with self._lock:
            best, best_distance = ROBOT_IDS[0], float("inf")
            for robot_id in ROBOT_IDS:
                pose = self._latest[robot_id]["pose"]
                if pose is None:
                    continue
                distance = math.hypot(target[0] - pose.position.x,
                                      target[1] - pose.position.y)
                if distance < best_distance:
                    best, best_distance = robot_id, distance
            return best

    def _set_mission(self, robot_id, availability, role, mission_id,
                     target=None) -> None:
        with self._lock:
            self._mission[robot_id] = {
                "availability": availability, "role": role,
                "mission_id": mission_id, "target": target,
            }

    def _publish_event(self, event_id, detected_at, target, camera,
                       status, confidence, consecutive) -> None:
        message = EmergencyEvent()
        message.event_id = event_id
        message.detected_at = detected_at
        message.location.header.stamp = self.get_clock().now().to_msg()
        message.location.header.frame_id = "map"
        message.location.point.x = float(target[0])
        message.location.point.y = float(target[1])
        message.confidence = float(confidence)
        message.consecutive_detections = int(consecutive)
        message.status = status
        message.source_id = camera
        message.camera_id = camera
        message.zone_id = "zone-a"
        self.event_pub.publish(message)

    def _publish_assignment(self, mission_id, event_id, robot_id, target,
                            version: int) -> None:
        message = MissionAssignment()
        message.mission_id = mission_id
        message.event_id = event_id
        message.robot_id = robot_id
        message.role = RobotState.ROLE_AED_DELIVERY
        message.target.header.stamp = self.get_clock().now().to_msg()
        message.target.header.frame_id = "map"
        message.target.pose.position.x = float(target[0])
        message.target.pose.position.y = float(target[1])
        message.target.pose.orientation.w = 1.0
        message.assigned_at = self.get_clock().now().to_msg()
        message.assignment_version = int(version)
        message.cancel_previous = version > 1
        self.assignment_pubs[robot_id].publish(message)

    def _publish_mission(self, mission_id, event_id, robot_id, status,
                         reason: str = "", version: int = 1) -> None:
        message = MissionStatus()
        message.mission_id = mission_id
        message.event_id = event_id
        message.robot_id = robot_id
        message.assignment_version = version
        message.status = status
        message.stamp = self.get_clock().now().to_msg()
        message.reason = reason
        self.mission_pub.publish(message)
        self.get_logger().info(
            f"{mission_id} {robot_id} status={status}"
            + (f" ({reason})" if reason else "")
        )

    def _sleep(self, seconds: float) -> bool:
        """중간에 멈출 수 있는 대기. 멈추라는 뜻이면 True."""
        return self._stop.wait(seconds)

    def shutdown(self) -> None:
        self._stop.set()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="rosbag 과 함께 관제 화면을 실제 ROS 로 돌린다"
    )
    parser.add_argument(
        "--pause", type=float, default=4.0,
        help="한 시나리오가 끝나고 다음까지 쉬는 초",
    )
    args = parser.parse_args()

    rclpy.init()
    node = DemoPublisher(args.pause)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
