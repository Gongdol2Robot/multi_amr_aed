# 인터페이스 사양

이 문서가 통신 규약의 기준입니다. 노드를 만들기 전에 여기서 이름·자료형·
통신 방식을 확정하고, 코드는 이 표를 따릅니다.

이 값들이 관제 DB 의 어느 칸에 어떤 형으로 들어가는지, 그리고 지금 무엇이
구현돼 있고 무엇이 사양뿐인지는 [db_interfaces.md](db_interfaces.md) 에
있습니다.

## 통신 방식을 고르는 기준

셋을 섞어 쓰면 나중에 바꾸기 어려우므로 기준을 먼저 정합니다.

| 방식 | 쓰는 경우 | 안 쓰는 경우 |
|---|---|---|
| **Topic** | 계속 흐르는 상태, 받는 쪽이 여럿, 놓쳐도 다음 값이 옴 | 응답이 필요할 때 |
| **Service** | 한 번 묻고 바로 답, 즉시 끝남 | 오래 걸리는 일 |
| **Action** | 오래 걸리고, 진행 상황을 보고, 도중에 취소할 수 있어야 함 | 즉시 끝나는 일 |

**핵심 판단**: 출동 지시는 **Action** 입니다. 수십 초가 걸리고, 진행 상황을
계속 봐야 하고, 재할당 때 취소해야 합니다. 이 셋은 Action 의 정의 그대로입니다.
현재 코드는 Topic(`MissionAssignment`) + Topic(`MissionStatus`) 조합으로
Action 을 손으로 흉내내고 있어, 취소·중복 방지를 `assignment_version` 으로
직접 관리합니다. Action 으로 바꾸면 그 코드가 사라집니다.

---

## 시나리오와 인터페이스 대응

```
① 쓰러진 사람 발생
      ↓  웹캠이 봄 (목각인형)              → topic  EmergencyEvent
      ↓  또는 119/신고자가 좌표를 줌        → service ReportEmergency
② 그 지점의 혼잡도 판단                     → topic  CrowdLevel
      ↓  낮음 → 가까운 1대                  → action DeliverAed  (1대)
      ↓  높음 → 2대 모두                    → action DeliverAed  (2대)
③ 가던 로봇이 못 감                         → action 결과 = 실패
      ↓  나머지 1대 출발                    → action DeliverAed  (재할당)
④ 주행 중 라이다 고장                       → topic  SensorHealth
      ↓  cmd_vel 로 직접 주행               → topic  /robotN/cmd_vel
⑤ 현장 도착, AED 전달                       → action 결과 = 성공
      ↓  helper(빨간 RC카) 확인             → topic  DetectionSummary
```

---

## 1. 검출 — `aed_vision`

고정 웹캠은 **쓰러진 사람만** 봅니다. AMR 의 OAK-D 는 **쓰러진 사람 + helper**
를 봅니다. 그래서 검출 결과 메시지에 대상 종류가 들어가야 합니다.

| 이름 | 방식 | 타입 | 발행 | 구독 |
|---|---|---|---|---|
| `/{camera_id}/vision/emergency_event` | topic | `aed_interfaces/EmergencyEvent` | vision_detector | mission_manager, aed_hmi |
| `/{camera_id}/vision/detections` | topic | `aed_interfaces/DetectionSummary` | vision_detector | aed_hmi |
| `/{camera_id}/vision/crowd_level` | topic | `aed_interfaces/CrowdLevel` | vision_detector | mission_manager, aed_hmi |
| `/{camera_id}/vision/debug/compressed` | topic | `sensor_msgs/CompressedImage` | vision_detector | aed_hmi |
| `/{camera_id}/vision/heartbeat` | topic | `aed_interfaces/Heartbeat` | vision_detector | amr_recovery |

`camera_id` 는 `camera_open`, `camera_alley`, `robot1`, `robot2` 입니다.
로봇의 OAK-D 검출도 같은 규약을 쓰면 화면과 관제가 구분 없이 처리합니다.

**바뀌어야 할 것**: 현재 `crowd_level` 은 `std_msgs/String`, `person_count`
는 `std_msgs/UInt32` 입니다. 문자열로 상태를 주고받으면 오타가 실행 시점에야
드러나고, 판단 근거(사람 수, 기준값)가 따라오지 않습니다. 아래 `CrowdLevel`
로 바꿉니다.

## 2. 신고 접수 — 외부 입력

119 나 신고자가 주는 좌표는 **service** 로 받습니다. 접수됐는지 즉시 답을
줘야 하고(Topic 은 받았는지 알 수 없음), 오래 걸리는 일이 아닙니다.

| 이름 | 방식 | 타입 |
|---|---|---|
| `/aed/report_emergency` | **service** | `aed_interfaces/ReportEmergency` |

## 3. 배정과 출동 — `mission_manager` ↔ `robot_missions`

| 이름 | 방식 | 타입 | 서버 | 클라이언트 |
|---|---|---|---|---|
| `/{robot_id}/deliver_aed` | **action** | `aed_interfaces/DeliverAed` | robot_missions | mission_manager |
| `/aed/mission_status` | topic | `aed_interfaces/MissionStatus` | mission_manager | aed_hmi, event_logger |
| `/aed/robot_state` | topic | `aed_interfaces/RobotState` | robot_state_monitor | mission_manager, aed_hmi |

Action 을 쓰면 이렇게 정리됩니다.

- 재할당 = 기존 goal 을 `cancel_goal` 하고 다른 로봇에 새 goal
- 진행 상황 = feedback 으로 남은 거리·예상 시간이 계속 옴
- 결과 = 성공/실패가 result 로 한 번 확정됨

`MissionStatus` topic 은 남깁니다. Action 은 요청한 쪽(mission_manager)만
결과를 보지만, 화면과 로그는 모든 임무의 상태 변화를 봐야 하기 때문입니다.

## 4. 센서 이상과 대체 주행 — `sensor_recovery`

| 이름 | 방식 | 타입 | 발행 | 구독 |
|---|---|---|---|---|
| `/{robot_id}/sensor_health` | topic | `aed_interfaces/SensorHealth` | sensor_recovery | mission_manager, aed_hmi |
| `/{robot_id}/cmd_vel` | topic | `geometry_msgs/Twist` | sensor_recovery(대체 주행) | Create3 |

라이다가 죽으면 Nav2 는 못 씁니다. 그때 `sensor_recovery` 가 `cmd_vel` 로
직접 몹니다. Nav2 도 같은 토픽에 쓰므로 **둘이 동시에 쓰면 안 됩니다.**
`SensorHealth.degraded` 가 참인 동안에는 mission_manager 가 Nav2 goal 을
내지 않는 것으로 약속합니다.

## 5. 시간 기록 — `event_logger`, `aed_hmi`

DB 에 남길 시각은 전부 **이미 있는 메시지에서** 얻습니다. 새 통신이 필요
없습니다.

| 값 | 출처 |
|---|---|
| 신고 접수 시각 | `EmergencyEvent.detected_at` |
| 출동 시작 시각 | `DeliverAed` goal 수락 시점 = `MissionStatus.DISPATCHING` |
| 도착 시각 | `DeliverAed` result 수신 시점 = `MissionStatus.ARRIVED` |
| 예상 도착 시간 | `DeliverAed` feedback 의 `eta_seconds` |
| 속도 | `RobotState.speed_mps` (신설) |
| 현재 위치 | `RobotState.pose` |

**바뀌어야 할 것**: `RobotState` 에 속도가 없습니다. 지금은 aed_hmi 가
연속된 pose 로 직접 계산하는데, 같은 계산을 여러 곳에서 하게 되므로
`robot_state_monitor` 가 실어 보내는 편이 맞습니다.

---

## 신설·변경이 필요한 인터페이스

### 신설 `msg/CrowdLevel.msg`
혼잡도를 문자열이 아니라 등급 + 근거로 보냅니다.

### 신설 `msg/DetectionSummary.msg`
프레임당 검출 결과. 웹캠은 쓰러진 사람만, AMR 은 helper 까지 채웁니다.

### 신설 `msg/SensorHealth.msg`
라이다 상태와 대체 주행 여부.

### 신설 `srv/ReportEmergency.srv`
119/신고자 좌표 접수.

### 신설 `action/DeliverAed.action`
출동 지시. `MissionAssignment.msg` 를 대체합니다.

### 변경 `msg/RobotState.msg`
`float32 speed_mps` 추가.

### 변경 `msg/EmergencyEvent.msg`
`uint8 crowd_level` 추가. 어느 혼잡도에서 판단된 이벤트인지 남겨야 사후에
"왜 2대가 갔나"를 되짚을 수 있습니다.
