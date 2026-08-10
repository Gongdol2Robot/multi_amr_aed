# aed_interfaces

멀티 AMR 시스템에서 공통으로 사용하는 ROS 2 인터페이스입니다.

- `EmergencyEvent`: 감지 시각, 위치, 좌표 신뢰 상태, 검출 근거와 이벤트 상태
- `RobotState`: 로봇 위치, 가용성, 배터리, 현재 역할과 이벤트별 경로비용
- `MissionAssignment`: 단일 활성 로봇의 AED 임무와 재할당 버전
- `MissionStatus`: 출동·도착·장애·취소·복구 대기 상태
- `Heartbeat`: Mission Manager와 로봇 사이의 양방향 생존 신호
- `CrowdLevel`: 카메라 ROI의 0~3 혼잡 등급과 통행 가능 여부
- `DetectionSummary`: 비전 프레임별 사람·환자·조력자 검출 요약
- `HelperPresence`: 위치와 거리까지 확보된 구조화된 조력자 관측
