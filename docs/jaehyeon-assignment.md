# 프로젝트 컨텍스트 및 박재현 담당 영역

## 프로젝트명

다중 AMR 기반 실내 안전 순찰 및 긴급 AED 운반 시스템

## 프로젝트 개요

두 대의 TurtleBot4를 이용하여 평상시에는 실내 시설을 순찰하고, 응급상황 발생 시 적합한 로봇을 선정하여 AED를 현장까지 운반하는 시스템이다.

시스템은 다음 기능으로 구성된다.

* 다중 AMR 순찰
* 고정 웹캠 기반 응급상황 감지
* 응급상황 발생 위치 전달
* 출동 로봇 선정
* AED 운반
* 현장 길안내
* LiDAR 및 로봇 상태 감시
* 장애 발생 시 대체 로봇 재할당
* HMI 기반 로봇 상태 관제

---

## 평상시 운영

두 대의 TurtleBot4는 각각 지정된 Waypoint를 따라 실내를 순찰한다.

순찰 중에는 다음 정보가 관리된다.

* 현재 로봇 위치
* 현재 순찰 Waypoint
* 다음 이동 Waypoint
* Nav2 주행 상태
* LiDAR 상태
* 위치 추정 상태
* 로봇 통신 상태
* 배터리 상태
* 현재 임무 상태

응급상황이 발생하면 진행 중인 순찰 임무를 중단하고, 이후 복귀를 위해 현재 순찰 지점과 진행 상태를 저장한다.

---

## 응급상황 처리

고정 웹캠과 비전 시스템에서 응급상황을 감지하면 응급상황 발생 위치가 Mission Manager로 전달된다.

Mission Manager는 두 AMR 각각에 대해 응급상황 위치까지의 이동 조건을 비교한다.

출동 로봇 선정에는 다음 정보가 반영된다.

* Nav2가 생성한 이동 경로
* 경로 길이
* 예상 이동시간
* Costmap의 장애물 비용
* 고정 웹캠 기반 사람 혼잡도
* LiDAR 정상 여부
* 위치 추정 정상 여부
* Nav2 주행 가능 여부
* 로봇 통신 상태
* 배터리 상태

초기 계획의 단순 최단거리 방식은 사용하지 않는다.

각 로봇의 Nav2 경로와 Costmap 장애물 비용, 사람 혼잡도를 반영하여 보정 ETA를 계산한다.

```text
보정 ETA =
Nav2 기본 예상 이동시간
+ Costmap 장애물 비용
+ 사람 혼잡도 비용
+ 로봇 상태 패널티
```

보정 ETA가 가장 짧고 정상적으로 주행 가능한 로봇을 `PRIMARY`로 선정한다.

다른 로봇은 `STANDBY` 상태로 대기한다.

---

## 이동 중 재평가

PRIMARY 로봇이 AED를 운반하는 동안에도 두 로봇의 상태와 예상 도착시간을 계속 확인한다.

사람 혼잡도 증가, 통로 장애물 발생, 경로 변경 등으로 PRIMARY의 예상 도착시간이 크게 증가할 수 있다.

단순한 순간적 ETA 역전으로 출동 로봇을 바로 변경하지 않고 다음과 같은 조건을 고려한다.

* 두 로봇의 ETA 차이
* ETA 차이가 유지된 시간
* 재할당으로 인한 추가 지연
* STANDBY 로봇의 현재 위치와 상태
* 기존 PRIMARY의 이동 진행도
* 재할당 횟수 제한

PRIMARY 로봇에서 LiDAR 장애, 통신 장애, 위치 추정 장애 또는 Nav2 주행 불가 상태가 발생하면 해당 로봇은 출동 임무를 지속할 수 없는 상태로 처리된다.

이 경우 STANDBY 로봇의 상태를 확인한 뒤 긴급 임무를 재할당한다.

---

## 긴급 임무 종료

AED 운반 또는 현장 대응이 완료되면 로봇은 기존 순찰 임무로 복귀한다.

복귀 방식은 다음과 같이 구성될 수 있다.

* 긴급상황 발생 전 마지막 순찰 지점부터 재개
* 다음 Waypoint부터 순찰 재개
* 지정된 대기 위치로 복귀한 뒤 순찰 재시작

긴급 임무 수행 전 저장된 순찰 상태를 기준으로 복귀 지점을 결정한다.

---

# 박재현 담당 영역

박재현의 담당 영역은 다음과 같다.

## 1. Waypoint 기반 순찰

* TurtleBot4의 평상시 순찰 흐름
* 순찰 Waypoint 목록 및 순서 관리
* Nav2를 이용한 Waypoint 이동
* 현재 순찰 지점 관리
* 순찰 진행 상태 관리
* 긴급상황 발생 시 순찰 상태 저장
* 긴급 임무 전환 시 기존 Nav2 이동 중단
* 긴급 임무 종료 후 순찰 상태 복원
* 마지막 지점 또는 다음 Waypoint부터 순찰 재개

## 2. LiDAR 상태 감시

각 로봇의 `/scan` 토픽 수신 상태를 독립적으로 감시한다.

LiDAR 상태 판단에는 다음 정보가 포함된다.

* 마지막 `/scan` 수신 시각
* 현재 `/scan` 수신 주기
* 설정 시간 이상 메시지가 수신되지 않았는지 여부
* LiDAR 정상 상태
* LiDAR 장애 상태
* 장애 이후 복구 상태

LiDAR 메시지가 일정 시간 이상 수신되지 않으면 해당 로봇은 LiDAR 장애 상태로 분류된다.

LiDAR 장애 상태의 로봇은 정상적인 장애물 인식이 불가능하므로 주행 가능 상태에서 제외된다.

## 3. LiDAR 장애 발생 시 시스템 상태

LiDAR 장애 발생 시 로봇 상태는 다음과 같은 흐름을 가진다.

```text
정상 주행
→ /scan 수신 중단
→ LiDAR Timeout
→ 주행 불가 상태
→ 현재 이동 중단
→ Mission Manager에 장애 상태 전달
→ PRIMARY 선정 후보 제외
→ 필요 시 STANDBY 로봇으로 임무 재할당
```

예상 상태 정보는 다음과 같다.

```text
robot_id
lidar_alive
navigation_available
fault_code
last_scan_time
robot_state
```

LiDAR 장애 상태의 예시는 다음과 같다.

```text
lidar_alive = false
navigation_available = false
fault_code = LIDAR_TIMEOUT
robot_state = LIDAR_FAULT
```

## 4. LiDAR 복구 상태

LiDAR 토픽이 다시 한 번 수신됐다는 이유만으로 즉시 정상 상태로 전환하지 않는다.

복구 과정은 다음과 같다.

```text
LiDAR 메시지 재수신
→ 일정 횟수 또는 일정 시간 연속 수신
→ LiDAR 수신 주기 정상 여부 확인
→ Nav2 상태 확인
→ 위치 추정 상태 확인
→ RECOVERING 상태
→ 정상 주행 가능 상태 복귀
```

LiDAR 복구 이후에는 Mission Manager가 해당 로봇을 다시 출동 후보 또는 순찰 가능 로봇으로 사용할 수 있다.

---

## 로봇 상태 체계

프로젝트에서 사용할 수 있는 주요 로봇 상태는 다음과 같다.

```text
IDLE
PATROL
EMERGENCY_PRIMARY
STANDBY
LIDAR_FAULT
LOCALIZATION_FAULT
COMMUNICATION_FAULT
NAVIGATION_FAULT
RECOVERING
MISSION_COMPLETE
```

박재현 담당 영역은 이 중 다음 상태와 직접적으로 관련된다.

* `PATROL`
* `EMERGENCY_PRIMARY`
* `STANDBY`
* `LIDAR_FAULT`
* `NAVIGATION_FAULT`
* `RECOVERING`

---

## 담당 영역의 시스템 연동 위치

```text
Waypoint 순찰 노드
        │
        ├─ 현재 순찰 위치 및 진행 상태
        ├─ 순찰 중단 및 재개 정보
        │
LiDAR Watchdog
        │
        ├─ /robot1/scan 감시
        ├─ /robot2/scan 감시
        ├─ LiDAR 정상·장애·복구 상태
        │
        ▼
Robot Status
        │
        ▼
Mission Manager
        │
        ├─ 출동 후보 판단
        ├─ PRIMARY 선정
        ├─ STANDBY 관리
        └─ 장애 발생 시 재할당
```

---

## 담당 테스트 범위

박재현 담당 기능과 관련된 주요 테스트 항목은 다음과 같다.

* 두 로봇의 Waypoint 순찰
* 순찰 Waypoint 순차 이동
* 순찰 중 응급상황 발생
* 순찰 상태 저장
* 기존 Nav2 목표 중단
* 긴급 임무 종료 후 순찰 재개
* robot1의 `/scan` 중단
* robot2의 `/scan` 중단
* 두 로봇의 LiDAR 상태 독립 감시
* 순찰 중 LiDAR 장애 발생
* AED 출동 중 LiDAR 장애 발생
* 일시적인 LiDAR 메시지 지연
* 지속적인 LiDAR 메시지 중단
* LiDAR 재수신 후 복구 상태 확인
* LiDAR 장애 로봇의 PRIMARY 후보 제외
* PRIMARY LiDAR 장애 시 STANDBY 재할당
* 복구된 로봇의 순찰 또는 대기 상태 복귀

---

## 팀 내 역할 구분

* 이준우: 프로젝트 관리, 일정, 요구사항, 문서 및 발표
* 이현민·김지훈: 비전 감지 결과 처리, 응급 위치 전달, Mission Manager, 출동 로봇 선정 및 통합
* 박재현: Waypoint 순찰, 순찰 상태 저장·중단·재개, LiDAR 장애 감시, 주행 불가 상태 전달
* 김재엽: AED 운반 기능
* 김민성: 현장 길안내 기능
* 김영기: 청소 관련 기능
* HMI 담당: 로봇 상태와 임무 진행 상황을 표시하는 관제 화면

박재현 담당 영역의 핵심 구성은 다음 세 부분이다.

```text
Waypoint 순찰
LiDAR Watchdog
Mission Manager 상태 연동
```
