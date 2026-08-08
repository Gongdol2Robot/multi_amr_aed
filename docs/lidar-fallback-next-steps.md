# LiDAR fallback 컨트롤러 다음 단계 (계획만, 미구현)

`sensor_recovery/fallback_path_follower.py`(구현 완료, 실험 단계)에 이어서
논의된 개선 방향을 정리한다. 아래는 전부 설계만 되어 있고 코드는 없다.

## 목표 재확인

**정밀 도달이 목적이 아니다.** LiDAR가 없는 상황에서도 목표 방향으로
"어느 정도" 계속 주행할 수 있으면 충분하다 — 오도메트리 드리프트나 depth
판정 오차가 있어도, 완전히 멈추거나 엉뚱한 방향으로 폭주하지만 않으면
성공으로 본다. 이 전제가 아래 두 항목의 설계 기준이다: 애매하면 "그래도
대충 맞는 방향으로 진행" 쪽을 우선하고, 아주 정밀한 판정 로직에 과투자하지
않는다.

## 1. 깊이 안전정지에서 "이미 아는 지도상 장애물" 제외

**문제**: 지금 `min_depth_in_roi`는 전방에 뭐가 가깝기만 하면 무조건
정지시킨다. 그런데 지도(map)에 이미 있는 벽·구조물 옆을 지나가는 경로도
많아서, 실제로는 안전한데 불필요하게 자주 멈출 수 있다.

**설계**: 깊이로 감지한 가까운 지점이 **정적 지도에 이미 점유(occupied)로
표시된 곳**이면 무시하고, **지도에 없는데 갑자기 가까운 것**만 새로운
장애물로 보고 정지한다.

판정 흐름:
```
전방 ROI의 가까운 픽셀들
  → 픽셀을 3D로 역투영 (카메라 프레임, camera_info의 fx/fy/cx/cy 사용)
  → base_link 프레임으로 변환 (camera→base_link는 고정 TF, LiDAR 없어도 항상 유효)
  → map 프레임으로 변환 (fallback이 이미 오도메트리로 추적 중인
    integrate_odom_delta 결과의 현재 pose 사용)
  → 그 map 좌표가 /robotN/map(OccupancyGrid)에서 이미 occupied인지 조회
  → 전부 "이미 아는 점유 지점"이면 무시하고 계속 주행
  → 하나라도 "지도엔 없는데 가까움"이면 정지
```

필요한 입력 추가: `<ns>/map`(OccupancyGrid, map_server가 이미 발행 중이라
구독만 하면 됨), `<ns>/oakd/stereo/camera_info` 또는 depth와 정렬된
camera_info, camera→base_link 정적 TF(`tf2_ros.Buffer`).

`path_follow_control.py`에 추가할 순수 함수 후보: 픽셀→3D 역투영,
map 좌표→grid cell 인덱스 변환 및 점유 조회, 이 둘을 합친 판정 함수.
기존 `day3/3_3_b_depth_to_3d.py`의 역투영 패턴을 참고할 수 있다.

**한계**: 이 판정은 fallback이 오도메트리로 "추정"하는 현재 위치에
의존한다. LiDAR 없이 오도메트리만 쓰는 거라 시간이 지날수록 드리프트가
쌓여서 오판(아는 벽인데 모르는 걸로, 또는 그 반대)이 늘 수 있다. 애매하면
정지 쪽으로 두는 fail-safe 원칙은 유지한다 — 위 "목표 재확인" 기준대로,
가끔 불필요하게 멈추는 건 괜찮지만 모르는 장애물에 부딪히는 건 안 된다.

## 2. 카메라 원본(raw) 대신 압축/저해상도 스트림 사용

**문제**: `<ns>/oakd/stereo/image_raw`를 그대로 구독하면 프레임 용량이 커서
discovery server 환경(이미 여러 번 느린 걸 확인함)에서 프레임 드랍이
심할 것으로 예상됨.

**대안 (실제 존재 확인된 토픽, robot1 기준)**:
- `<ns>/oakd/stereo/image_raw/compressedDepth` — depth 값을 보존하는 압축
  포맷. `image_transport`의 `compressed_depth_image_transport` 플러그인으로
  구독해야 depth 값이 정상 복원됨 (일반 `cv_bridge.imgmsg_to_cv2`로 바로
  못 읽음 — `image_transport.Subscriber`에 transport hint로
  `compressedDepth` 지정하거나, 해당 패키지의 디코더를 직접 써야 함).
- `<ns>/oakd/rgb/preview/image_raw` — RGB는 저해상도 preview 스트림이 이미
  있음 (지금 fallback 로직은 RGB를 안 쓰지만, 나중에 RGB 기반 판단이
  추가되면 이걸 쓰면 됨).
- depth 쪽에 preview 급 저해상도 스트림이 없다면, `depthai_ros_driver`
  파라미터(카메라 config yaml, `tools/`나 로봇 쪽 `turtlebot4_bringup/config/
  oakd_pro.yaml` 등)에서 해상도/FPS를 낮추는 것도 대안 — 구현 시점에
  실제 파라미터 이름 확인 필요.

**적용 위치**: `fallback_path_follower.py`의 depth 구독을
`stereo/image_raw` → `stereo/image_raw/compressedDepth` + 적절한 디코더로
교체. 새로 추가하는 map-occupancy 체크(1번 항목)도 이 압축 스트림을
전제로 설계한다.

## 우선순위

카메라(robot1 OAK-D)는 사용자가 직접 수리 예정. 수리 완료 후:
1. 압축 depth 스트림으로 구독 방식 교체 (프레임 드랍 문제 먼저 해결)
2. 지도 기반 안전정지 개선 (1번 항목)
3. 실제 로봇으로 재검증
