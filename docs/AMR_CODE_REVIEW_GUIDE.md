# AMR Code Review Guide

## 0. 리뷰 목표

PPT 없이 **설계도 -> 실제 코드 -> 테스트/검증 결과** 순서로 설명한다.
발표 본문은 15~18분 정도로 끝내고 나머지를 Q&A에 남긴다.

코드를 line-by-line으로 읽지 않는다. VS Code에서 `[CODE REVIEW]`를 검색해 아래 핵심 위치만 순서대로 연다.

---

## 1. 먼저 보여줄 디렉토리 구조

```text
src/multi_robot_emergency/
├── config/crowd_zones.yaml
├── launch/central_dispatch.launch.py
├── multi_robot_emergency/
│   ├── mission_manager.py   # 중앙 판단/배정/재배정
│   ├── assignment.py        # ETA/dispatch/standoff 순수 정책
│   └── crowd.py             # crowd 상태 시간 필터
└── test/
    ├── test_assignment.py
    └── test_crowd.py

src/robot_missions/
├── robot_missions/mission_executor.py  # robot별 NavigateToPose 실행기
└── test/test_progress.py
```

`central_dispatch.launch.py` 기준 핵심 런타임 노드는 **3개**다.

1. 중앙 `emergency_mission_manager` 1개
2. `robot1_mission_executor` 1개
3. `robot2_mission_executor` 1개

같은 MissionExecutor 클래스를 robot_id/topic/action namespace만 다르게 두 번 실행한다.

---

## 2. 설계도 설명 순서 (약 4~5분)

### 2-1. 응급 목표 입력

설명 문장:

> 응급 위치는 중앙 `Emergency Mission Manager`가 받고, 이 시점에는 바로 로봇을 움직이지 않습니다. 먼저 robot1과 robot2 각각의 실제 Nav2 global path를 요청해서 ETA를 비교합니다.

입력 예:
- `/emergency/request`
- Vision `EmergencyEvent`

### 2-2. 후보 경로 계산

Nav2 API:
- `nav2_msgs/action/ComputePathToPose`
- `/robot1/compute_path_to_pose`
- `/robot2/compute_path_to_pose`

설명 문장:

> 중앙은 두 로봇의 `ComputePathToPose` Action Client입니다. Goal은 환자 쪽 목표 위치이고, Planner Server가 Action Result로 `nav_msgs/Path`를 반환합니다. 이 경로는 주행 명령이 아니라 로봇 선택을 위한 비교 데이터입니다.

### 2-3. ETA + crowd penalty

기본 ETA:

```text
base ETA = path_distance / linear_speed
         + total_turn / angular_speed
         + slowdown_count * slowdown_penalty
```

현재 주요 기본값:
- linear speed: `0.20 m/s`
- angular speed: `0.70 rad/s`
- 큰 회전 threshold: `45 deg`
- slowdown penalty: `4.0 s`

Crowd 추가 지연:

```text
crowd_delay = d_zone * (1 / v_stage - 1 / v_normal)
final ETA   = base ETA + crowd_delay
```

설명 문장:

> base ETA에 정상 속도 주행 시간이 이미 들어 있으므로 crowd 구간 전체 시간을 다시 더하지 않고, 정상 속도 대비 추가 지연만 더합니다.

Crowd 단계:
- CLEAR
- BUSY
- CROWDED
- BLOCKED

BLOCKED 설명:

> BLOCKED는 단순히 아주 느리게 통과시키는 단계가 아니라 keepout mask를 Nav2 costmap에 반영하고 다시 경로를 요청합니다. 그래도 반환 경로가 crowd polygon을 교차하면 해당 후보를 제외합니다.

### 2-4. 선택 / dual dispatch / 재배정

설명 문장:

> final ETA가 가장 작은 로봇이 기본 1순위입니다. 단, 두 후보가 모두 유효하고 가장 빠른 ETA도 목표 30초의 85%인 25.5초 이상이면 deadline risk로 보고 두 대를 동시 출동시킵니다.

주행 중에는 live ETA를 다시 계산한다.

```text
교체 조건:
standby ETA + 2 s <= current ETA
AND
standby ETA <= current ETA * 0.85
```

설명 문장:

> 초기 ETA는 출발 시점 정보이므로 주행 중 crowd 변화나 경로 변화가 생길 수 있습니다. 그래서 약 3초마다 current와 standby의 남은 경로를 다시 계산하고, 의미 있게 빨라질 때만 교체합니다.

---

## 3. 코드 설명 순서 (약 8~10분)

VS Code에서 `[CODE REVIEW]` 검색을 사용한다.

### 3-1. `mission_manager.py` — 중앙 전체 flow

#### A. `_on_request`

보여줄 내용:
- 새 emergency cycle 초기화
- 기존 mission 상태 clear
- crowd snapshot 확보
- BLOCKED면 keepout 적용 후 planner 호출

설명:

> 이 함수가 한 건의 응급 이벤트 시작점입니다. 기존 요청 상태를 전부 초기화해서 이전 mission 데이터가 다음 사건에 섞이지 않도록 합니다.

#### B. `_make_standoff_target`

설명:

> 환자 좌표를 그대로 Nav2 goal로 사용하면 로봇 중심이 환자 위치까지 들어갑니다. 그래서 환자 반경 0.15m 지점에 정지 목표를 만들고 로봇 yaw는 환자를 향하게 했습니다.

#### C. `_calculate_and_assign` -> `_on_plan_response` -> `_on_plan_result`

여기가 **Nav2 API 설명의 핵심**이다.

강조할 코드:

```python
goal = ComputePathToPose.Goal()
...
future = client.send_goal_async(goal)
```

그리고:

```python
handle.get_result_async()
```

최종:

```python
path = wrapped_result.result.path
```

설명:

> Action은 Goal 전송과 Result 수신이 분리되어 있습니다. 설계도에서 Planner로 가는 화살표가 Action Goal이고, 이 `result.path`가 반대로 돌아오는 `nav_msgs/Path`입니다.

그 다음 `_on_plan_result`에서:
1. path distance
2. 회전량 / slowdown
3. crowd polygon 내부 거리
4. crowd delay
5. final ETA
를 계산한다.

#### D. `_finish_planning`

설명:

> 여기서 두 ETA를 정렬하고 `dispatch_candidates()` 정책으로 single/dual dispatch를 결정합니다. ROS callback 안에 정책식을 섞지 않고 `assignment.py`에 순수 함수로 뺀 이유는 테스트하기 쉽게 하기 위해서입니다.

#### E. `_publish_assignment` + `_on_mission_status`

인터페이스:
- 중앙 PUB: `/robotX/mission_assignment`
- robot PUB / 중앙 SUB: `/aed/mission_status`

강조:
- `event_id`
- `robot_id`
- `assignment_version`

설명:

> DDS에서는 과거 메시지가 늦게 도착하거나 재배정 직전 상태가 뒤늦게 들어올 수 있어서 version을 함께 비교합니다. 현재 event와 현재 assignment_version이 일치하는 상태만 중앙 판단에 사용합니다.

#### F. `_handle_navigation_failure`

실패 상태:
- CANCELED
- BLOCKED
- NETWORK_LOST
- NAVIGATION_ERROR

설명:

> 실패 종류별로 재배정 코드를 따로 만들지 않고 한 함수로 모았습니다. 실패 로봇을 excluded set에 넣은 다음 기존 ETA ranking에서 다음 후보를 찾고 새 MissionAssignment를 발행합니다.

#### G. `_monitor_live_replan` / `_finish_live_replan`

설명:

> 주행 중에도 동일한 `ComputePathToPose`를 다시 호출합니다. 단순히 0.1초 빠르다고 계속 교체하지 않도록 2초 절대 이득과 85% 상대 조건을 동시에 사용했습니다.

#### H. `_monitor_dual_robot_proximity`

설명:

> dual dispatch는 도착 가능성을 높이지만 두 로봇이 같은 환자 위치로 모이면 충돌 위험이 있습니다. 일정 거리 이내가 일정 시간 유지되면 환자에게 더 먼 로봇을 복귀시킵니다.

---

### 3-2. `assignment.py` — 알고리즘만 빠르게

전부 읽지 말고 아래 5개만 보여준다.

1. `patient_standoff()`
2. `dispatch_candidates()`
3. `should_switch_for_live_eta()`
4. `crowd_delay_seconds()`
5. `path_motion_cost()`

핵심 설명:

> 이 파일은 ROS Node가 아니라 순수 함수 모음입니다. 센서나 Topic이 없어도 입력값만으로 테스트할 수 있게 중앙 callback에서 알고리즘을 분리했습니다.

`path_motion_cost()`에서 path simplification을 쓰는 이유:

> Grid planner의 픽셀/격자 수준 지터를 모두 실제 회전으로 계산하면 ETA가 과대평가됩니다. 그래서 Ramer-Douglas-Peucker 방식으로 작은 경로 흔들림을 제거한 뒤 의미 있는 회전량만 계산합니다.

---

### 3-3. `crowd.py` — 1분 이내

설명:

> 사람 수를 AMR에서 다시 thresholding하지 않습니다. Detection이 정한 최종 crowd stage를 받아 시간 안정화만 합니다.

핵심:
- 위험 상승: confirm 후 반영
- 위험 하강: 더 긴 hold 후 반영
- timeout: UNKNOWN
- person_count: 진단용

왜 하강 hold가 긴가:

> 군중이 잠깐 가려졌다고 즉시 CLEAR로 복귀하면 경로가 계속 흔들릴 수 있어서 보수적으로 하강을 늦췄습니다.

---

### 3-4. `robot_missions/mission_executor.py` — 실제 주행

#### `_on_assignment`

> 중앙에서 받은 MissionAssignment의 robot_id/version을 확인하고 최신 배정만 실행합니다.

#### `_send_goal`

Nav2 API:
- `nav2_msgs/action/NavigateToPose`

설명:

> `ComputePathToPose`는 중앙의 비교용 API이고, 실제 로봇을 움직이는 API는 여기의 `NavigateToPose`입니다.

#### `_goal_response` / `_navigation_done`

상태 흐름:

```text
MissionAssignment
-> DISPATCHING
-> NavigateToPose Goal accepted
-> EN_ROUTE
-> Result success: ARRIVED
-> error: NAVIGATION_ERROR
```

#### `_on_feedback` / `_check_progress`

설명:

> 처음에는 거리 감소만 보면 blocked를 판단할 수 있지만, 로봇이 제자리에서 방향을 맞추는 구간도 정상 동작입니다. 그래서 translation, rotation, distance_remaining 중 하나라도 진전이 있으면 progress로 보고, 모두 일정 시간 멈췄을 때만 BLOCKED를 중앙에 보냅니다.

---

## 4. 테스트 설명 (약 3~4분)

### 단위 테스트 매칭

`test_assignment.py`
- patient standoff 위치/yaw
- dual dispatch 경계값
- live ETA 교체 조건
- crowd delay 계산
- path distance/회전/slowdown ETA
- path simplification
- dual robot proximity

`test_crowd.py`
- 처음/timeout -> UNKNOWN
- stage 상승 confirm
- stage 하강 hold
- person_count가 stage 판단에 영향 없음
- numeric `0..3` 입력 변환

`test_progress.py`
- 제자리 회전도 progress
- 작은 pose noise는 progress 아님
- 실제 translation은 progress
- angle wrap 처리

### 기능 테스트 영상/로그에서 보여주면 좋은 3개 장면

1. **정상 배정**
   - 응급 request
   - 두 ComputePathToPose 결과
   - robot1/2 predicted ETA
   - selected/dispatched robot
   - MissionAssignment 발행

2. **Crowd 변화**
   - crowd level 변화
   - crowd delay 또는 BLOCKED keepout 반영
   - candidate ETA/경로가 달라지는 로그

3. **실패 재배정**
   - 선택 로봇에서 BLOCKED/NAVIGATION_ERROR
   - 중앙 `REASSIGNING`
   - 다음 로봇 MissionAssignment 발행

가능하면 영상 한 화면에 RViz + terminal log를 같이 보이게 한다.

### 현재 자동 테스트 실행 상태

이 작업을 수행한 Web Codex 실행 환경에는 `pytest` 명령이 설치되어 있지 않아 여기서는 자동 테스트를 재실행하지 못했다.
실제 리뷰 PC의 ROS2 workspace에서 환경을 source한 뒤 아래 테스트를 직접 실행하고 결과 캡처를 준비한다.

```bash
pytest src/multi_robot_emergency/test/test_assignment.py \
       src/multi_robot_emergency/test/test_crowd.py \
       src/robot_missions/test/test_progress.py -q
```

---

## 5. 30분 시간 배분 추천

- 0~2분: AMR 목적 + 디렉토리/노드 3개
- 2~6분: 설계도 flow
- 6~15분: mission_manager 핵심 코드
- 15~18분: assignment/crowd/mission_executor 핵심
- 18~21분: 테스트 결과/영상
- 21~30분: Q&A

발표를 20분 이상 끌지 않는 것이 좋다. 조교가 코드에서 질문할 시간을 남긴다.

---

## 6. 예상 질문과 답변 포인트

### Q. 왜 Euclidean distance가 아니라 ComputePathToPose를 사용했나?

> 실제 주행 가능 경로에는 벽/장애물/global costmap이 반영되어야 해서 직선거리는 실제 도착시간과 차이가 큽니다. 그래서 각 robot의 Nav2 Planner가 계산한 실제 path를 기준으로 비교했습니다.

### Q. ComputePathToPose와 NavigateToPose 차이는?

> ComputePathToPose는 경로만 계산해서 중앙 ETA 비교에 사용하고 로봇을 움직이지 않습니다. NavigateToPose는 선택된 로봇의 실제 주행 Action입니다.

### Q. ETA를 거리만으로 안 한 이유는?

> 같은 거리라도 큰 회전이 많으면 TurtleBot4가 감속/회전을 해야 해서 시간이 더 걸립니다. 그래서 path distance, total yaw change, 큰 코너 slowdown을 함께 사용했습니다.

### Q. crowd penalty를 왜 저 식으로 계산했나?

> base ETA에 정상속도 시간이 이미 포함돼 있어 crowd 구간의 전체 시간을 더하면 중복 계산됩니다. 따라서 crowd 속도로 달릴 때 증가하는 시간만 차이로 더합니다.

### Q. BLOCKED에서 왜 ETA를 매우 크게 주지 않고 후보를 제외하나?

> BLOCKED의 의미는 해당 공간을 통과하지 않는 것입니다. 그래서 먼저 keepout으로 Nav2에 재계획시키고, 반환 경로가 여전히 zone을 교차하면 유효 경로가 아니라고 판단합니다.

### Q. 왜 25.5초부터 두 대를 보내나?

> 목표 도착시간 30초에 trigger ratio 0.85를 둬 25.5초 이상이면 여유가 15% 미만이라고 판단합니다. 이때 둘 다 유효하면 deadline miss 위험을 낮추기 위해 dual dispatch합니다.

### Q. 왜 live ETA 교체 조건이 두 개인가?

> 하나만 쓰면 ETA noise 때문에 선택이 자주 바뀔 수 있습니다. 최소 2초 이득이라는 절대 기준과 85% 이하라는 상대 기준을 모두 만족할 때만 바꿉니다.

### Q. assignment_version이 왜 필요한가?

> 재배정 시 이전 로봇의 상태 메시지가 늦게 도착할 수 있습니다. event_id와 version을 같이 확인해 현재 배정의 상태만 유효하게 처리합니다.

### Q. 로봇이 막혔다는 건 어떻게 판단하나?

> NavigateToPose feedback에서 translation, rotation, distance_remaining을 모두 봅니다. 이 세 가지가 모두 일정 시간 진전이 없을 때만 BLOCKED로 판단합니다. 제자리 회전은 정상 progress입니다.

### Q. 왜 crowd state를 AMR에서 person_count로 다시 만들지 않나?

> Detection과 AMR의 책임을 분리하기 위해서입니다. Detection이 최종 stage를 결정하고 AMR은 그 stage를 경로/ETA 정책에 사용하는 역할만 합니다.

---

## 7. 발표 때 피할 것

- `mission_manager.py`를 위에서부터 줄줄 읽지 말 것
- parameter를 전부 설명하지 말 것
- ROS2 Topic 이름만 나열하고 끝내지 말 것
- 테스트 코드 자체를 자세히 리뷰하지 말 것
- LiDAR fallback 등 다른 담당자의 세부 구현을 본인이 구현한 것처럼 설명하지 말 것

항상 **문제 -> 선택한 설계 -> 핵심 코드 -> 검증 방법** 순서로 설명한다.
