# 작업 로그 (박재현 / jaehyeon 브랜치)

수정할 때마다 아래에 이어서 기록한다. 최신 항목이 아래로 쌓인다.

## 저장소 설정

- `multi_amr_aed`를 pull 받아 `multi_amr_aed_main`으로 이름 변경
- `multi_amr_aed_jaehyeon` 폴더 복사, `jaehyeon` 브랜치 생성 후 origin에 push

## 프로젝트 문서화

- `docs/jaehyeon-assignment.md`: 프로젝트 개요와 박재현 담당 영역(Waypoint 순찰,
  LiDAR Watchdog, Mission Manager 상태 연동) 정리

## LiDAR Watchdog 구현 (`sensor_recovery` 패키지)

- `sensor_recovery/lidar_state_machine.py`: ROS 비의존 순수 상태 머신
  (`STARTING→ALIVE→FAULT→RECOVERING`). `mission_manager/role_assignment.py`
  패턴을 따라 ROS와 분리해 pytest로 단독 검증 가능하게 구성.
- `sensor_recovery/lidar_watchdog_node.py`: 로봇별(`robot_names` 파라미터)
  `/scan` 독립 감시, `lidar_alive`(Bool)/`lidar_state`(String) 발행.
  `handle_lidar_fault`/`handle_lidar_recovery`를 확장 포인트로 분리해둠
  (Nav2 취소, Mission Manager 연동은 아직 미구현).
- `config/lidar_watchdog.yaml`, `launch/lidar_watchdog.launch.py` 추가.
- `package.xml`/`setup.py`에 `std_msgs` 의존성, 콘솔 스크립트, launch/config
  설치 규칙 추가.
- `test/test_lidar_state_machine.py`: 상태 전이 pytest 11개, 모두 통과.

### 상태 토픽 QoS 문제 발견 및 수정

- 처음엔 상태가 바뀔 때만 발행 → 늦게 구독하면(`ros2 topic echo`) 아무것도
  안 보이는 문제 발견.
- QoS를 `TRANSIENT_LOCAL`(latched)로 바꿨지만, 기본 volatile 구독자(`ros2
  topic echo` 등)는 latched 값 자체를 못 받는다는 것도 실측으로 확인
  (durability는 구독자도 요청해야 실제로 전달됨).
- 실용적 해결로 `status_publish_period_sec`(기본 1초) 타이머를 추가해 전이가
  없어도 현재 상태를 주기적으로 재발행. 로그는 여전히 전이 시점에만 출력.

### 검증

- pytest 11개, flake8 clean.
- 이 개발 환경은 Bash 호출 간 ROS2 프로세스 디스커버리가 안 되는 사실을
  확인 → 같은 프로세스 안에서 실제 노드를 그대로 구동하는 통합 스크립트로
  전이 전체(독립성, 복구, 복구 중 재끊김, 시작 유예시간 등) 검증.

## 실제 로봇 LiDAR 제어 조사 및 도구

- 실제 discovery server(192.168.107.101/.102)가 이 네트워크에서 접속 가능한
  것 확인. robot1(amcl/map_server 떠 있음, 사용 중일 가능성), robot2(idle)
  확인.
- `/robotN/stop_motor`, `/robotN/start_motor`(`std_srvs/srv/Empty`,
  `rplidar_ros` 표준 서비스) 존재 확인.
- **실측으로 발견한 문제**: 이 로봇 펌웨어는 `stop_motor`를 호출해도 모터
  회전만 멈추고 드라이버는 계속 `/scan`을 발행함(멈춘 지점의 고정값을 계속
  내보냄) → watchdog이 계속 `ALIVE`로 판정, 실제 fault 재현 안 됨.
- SSH로 `rplidar_composition` 프로세스 확인(읽기 전용): `turtlebot4_bringup/
  launch/rplidar.launch.py`에 `respawn` 설정이 없어 kill하면 자동으로 안
  살아남. systemd 서비스(`turtlebot4.service`)에도 `Restart=` 없어 다른
  노드(base, oakd, joy 등)에는 영향 없음을 확인.
- `tools/lidar_toggle.sh`에 `status`/`stop`/`start`(모터 제어, ROS 서비스만
  사용) + `scan-off`/`scan-on`(SSH로 드라이버 프로세스 kill/재실행, 진짜
  `/scan` 끊김 재현) 추가. 도킹 상태 확인 + 실행 전 확인 프롬프트 내장.
- `~/.bashrc`의 `ssh-robot` alias를 함수로 바꿔 `ssh-robot 1`→.101,
  `ssh-robot 2`→.102로 분기되도록 수정.
- `.env.robots`(gitignore됨) + `.env.robots.example`(템플릿, 커밋됨)로 SSH
  비밀번호를 저장소 밖에서 관리.

## scan-on 실사용 중 발견한 버그 (robot1에서 실측, 2026-08-06)

`scan-off 1`로 robot1 LiDAR를 정상적으로 끊는 데는 성공했으나, 초기 버전의
`scan-on`으로 되살리자 프로세스는 떴는데 `/robot1/scan`이 전혀 발행되지
않는 문제 발생. 원인 2가지, 둘 다 고침:

1. SSH로 띄운 명령이 `/opt/ros/humble/setup.bash`만 source하고 로봇 고유의
   discovery 환경(`/etc/turtlebot4/setup.bash`: `ROS_DOMAIN_ID=1`,
   `ROS_DISCOVERY_SERVER=";127.0.0.1:11811;"`)은 안 불러옴 → 새 프로세스가
   격리된 기본 도메인으로 붙어서, RPLidar SDK는 하드웨어 연결에 성공했지만
   ROS 그래프에는 노드도 퍼블리셔도 안 잡힘. `scan-on`이 로봇 고유의
   `/etc/turtlebot4/setup.bash`를 source하도록 수정.
2. `ros2 run rplidar_ros rplidar_composition ...`으로 띄우면 `ros2 run`
   래퍼(부모)와 실제 바이너리(자식)가 별도 PID로 뜬다. `find_lidar_pid`가
   부모 PID를 찾아서 kill하면 자식이 시리얼 포트(`/dev/RPLIDAR`)를 문 채로
   안 죽고 남아 다음 재시작을 막음. 원래 launch가 쓰는 바이너리
   (`/opt/ros/humble/lib/rplidar_ros/rplidar_composition`)를 `ros2 run` 없이
   직접 실행하도록 수정, `find_lidar_pid`도 이 경로로 매칭하도록 변경.
3. `scan-on`이 프로세스 존재만 확인하고 끝내던 것도 위 문제를 못 잡는
   원인이었음 → 재실행 후 실제로 `$ROBOT_NS/scan` 데이터가 오는지
   `ros2 topic echo --once`로 검증하는 단계 추가.

이 과정에서 robot1 LiDAR가 일시적으로 완전히 죽었던 상태(격리된 도메인의
좀비 프로세스가 시리얼 포트 점유)를 발견해 kill -9로 정리하고, 고친
스크립트로 재실행해 정상화함(watchdog에서 ALIVE 확인됨).

## .env.robots를 git 추적 대상으로 전환

프라이빗 저장소이고 팀원 전체가 로봇 SSH 비밀번호를 이미 알고 있다는
피드백에 따라, `.env.robots`를 더 이상 gitignore하지 않고 저장소에서 함께
관리한다. `.env.robots.example` 템플릿은 삭제하고,
`tools/lidar_toggle.sh`/`sensor_recovery/README.md`도 현재 관리 방식에 맞췄다.

## RobotState LiDAR 연동 제안 문서 작성 (코드 변경 없음)

`docs/robotstate-lidar-proposal.md` 작성. `RobotState.msg`에 `bool
lidar_ok` 필드 추가(기존 `*_ok` 필드 스타일 그대로), `robot_state_monitor`가
`lidar_alive`/`lidar_state`를 구독해 반영, `mission_manager._is_available()`에
`and state.lidar_ok` 추가하는 3단계 파이프라인 제안. `aed_interfaces`/
`mission_manager`는 각각 이현민/김재엽 담당이라 실제 구현은 보류 —
팀 협의 후 메시지 타입 확정되면 한 번에 구현 예정.

## LiDAR 장애 → Nav2 정지/재개 fallback 컨트롤러 구현 (실험 단계)

계획 승인 후 구현. `docs/robotstate-lidar-proposal.md`(RobotState 연동)는
보류하고 이 작업을 먼저 진행.

- `sensor_recovery/path_follow_control.py` 신설 — ROS 비의존 순수 함수:
  `advance_waypoint`(pure-pursuit 목표점 선택), `compute_cmd_vel`(비례
  제어), `integrate_odom_delta`(마지막 AMCL pose 기준 오도메트리 적분,
  로컬 프레임 변환으로 정확히 처리), `min_depth_in_roi`(깊이 이미지 안전
  거리), `pose_error`(오차 계산). 단위테스트 17개 전부 통과.
- `sensor_recovery/fallback_path_follower.py` 신설 — `lidar_state` 구독,
  FAULT 시 `navigate_to_pose` 활성 goal 전체 취소(빈 goal_id로 표준 방식
  사용, 자기가 안 보낸 goal도 끌 수 있음) + `/plan`·`/amcl_pose` 스냅샷,
  fallback 중엔 오도메트리 적분 위치로 스냅샷 경로를 따라가며 `/cmd_vel`
  직접 발행하되 전방 깊이(OAK-D stereo)가 가까우면 무조건 정지. ALIVE
  복귀 시 정지 + FAULT 시점 대비 AMCL pose 오차(m/deg) 로그 + 스냅샷
  경로의 마지막 지점으로 새 Nav2 goal 재전송.
- `package.xml`에 `action_msgs`/`cv_bridge`/`geometry_msgs`/`nav2_msgs`/
  `nav_msgs` 의존성 추가, `setup.py`에 `lidar_fallback_controller` 콘솔
  스크립트 추가.
- 실제 robot1에서 `/robotN/plan`(nav_msgs/Path, Nav2가 계속 갱신), `/robotN/
  odom`, `/robotN/amcl_pose`(PoseWithCovarianceStamped), `/robotN/oakd/
  stereo/image_raw` 토픽 존재 확인(read-only). cmd_vel 중재 장치(twist_mux
  등)는 이 저장소에 없어서, 별도 mux 없이 "Nav2 goal 취소 → 우리가 cmd_vel
  직접 발행" 순차 전환 방식으로 설계.
- 검증: 단위테스트 통과, 실제 노드를 그대로 띄우고 가짜 센서 데이터를
  publish하는 단일 프로세스 통합 테스트로 FAULT→주행 시작→장애물 안전정지
  →ALIVE→정지+오차로그(0.158m으로 정확히 계산됨 확인)+재개시도까지 전체
  흐름 확인. 실제 로봇 주행 테스트는 아직 미실행(사람이 지켜보는 가운데
  진행 예정, 실제 LiDAR는 안 끄고 `lidar_state`에 수동으로 FAULT/ALIVE를
  publish해서 트리거하는 방식).
- 설계 결정: 위치 추정은 RGB-D가 아니라 오도메트리 적분(마지막 AMCL
  기준)으로 하고, 깊이 카메라는 전방 안전정지 판단에만 쓰기로 사용자와
  합의(순수 visual localization은 이번 실험 범위 밖이라 판단).

## preflight.py 호스트 자동 계산 버그 수정

`nav 1`을 실행했더니 "로봇 네트워크 — 192.168.107.102"(robot2 IP)가 찍히는
걸 발견. 원인: `tools/preflight.py`의 `--host` 기본값이 `192.168.107.102`로
하드코딩되어 있었고, `aliases.sh`의 `nav()`/`loc()`는 `--namespace`만 넘기고
`--host`는 안 넘겨서 항상 이 기본값(robot2 IP)으로 ping했음 — `--namespace
robot1`을 줘도 네트워크 체크만 robot2를 보고 있었던 것. ping이라 로봇2에
직접 피해는 없지만, robot1 네트워크가 실제로 끊겨도 통과로 나오거나 그
반대로 나올 수 있는 잘못된 진단이었음.

`preflight.py`: `--host` 기본값을 `None`으로 바꾸고, 명시적으로 안 주면
`--namespace`의 로봇 번호로 `192.168.107.10N`을 자동 계산하도록 수정
(`robot1`→`.101`, `robot2`→`.102`, 확인됨). `aliases.sh`의 `pf()`도 같은
문제(`192.168.0.2`라는 임의 기본값)가 있어 host 인자를 안 주면 preflight.py의
자동 계산에 맡기도록 수정. robot1로 재확인 완료 — `192.168.107.101`
정상 ping.

## initpose 로봇별 좌표 버그 + 맵/RViz 통합 launch

`initpose 1`이 로봇2 도킹 위치(원점 근처, x=-0.0465 y=0.049)와 거의 같은
`(0,0,0)`을 고정으로 보내고 있어서 robot1(x=-0.576 y=0.1372)에 쓰면
지도·코스트맵이 안 맞는 문제 발견. main 브랜치에 이미 이 문제를 고친
`tools/initpose.py`(로봇별 실측 도킹 좌표를 `dock_poses.yaml`에서 읽어
발행/기록)와 `src/aed_bringup/config/dock_poses.yaml`이 올라와 있었는데
jaehyeon 브랜치엔 아직 병합 전이라 없었음. 두 파일을 jaehyeon으로 복사하고,
`aliases.sh`의 `initpose()`를 main과 동일하게 `python3 tools/initpose.py`를
호출하도록 수정(하드코딩된 `(0,0,0)` 발행 제거).

맵/RViz를 매번 터미널 두 개(`loc`+`rv`)로 따로 띄워야 하는 불편함 해결을
위해 `src/aed_bringup/launch/localization_view.launch.py` 신설 —
`turtlebot4_navigation/localization.launch.py` + `turtlebot4_viz/
view_robot.launch.py`를 하나로 묶음. `CMakeLists.txt`에 `launch` 디렉터리
설치 규칙 추가, `package.xml`에 `launch`/`launch_ros`/`turtlebot4_viz`
의존성 추가. `aliases.sh`에 `locview()` 별칭 추가. `--show-args`로 구조적
검증 완료(실제 로봇 없이 인자 파싱만 확인, 두 서브 launch의 인자가 정상
합쳐짐).

**참고**: jaehyeon 브랜치는 아직 main의 최신 커밋과 병합 전이라, 위 두
파일 외에도 `tools/align_maps.py`, `tools/fit_homography.py`,
`tools/pick_pixels.py`, `tools/explore.sh`, `vision_training/` 등 main에
새로 들어온 것들은 아직 안 가져왔음 — 필요해지면 그때 마저 가져오거나,
나중에 정식으로 `git merge main` 필요.

## woduqAMR 브랜치의 map_navigation 패키지를 jaehyeon으로 이식

`multi_amr_aed_woduqAMR` 폴더를 새로 만들어 `woduqAMR` 브랜치를 체크아웃해
확인해보니, `turtlebot4_map_navigation` 패키지에 localization + Nav2 + RViz +
`dock_poses.yaml` 기반 자동 초기위치 설정까지 한 번에 처리하는
`map_navigation.launch.py`가 있었음. jaehyeon에서 바로 쓸 수 있도록
가져옴:

- `src/turtlebot4_map_navigation/` 패키지 통째로 복사 (launch +
  `localization_initializer`/`navigation_initializer` 노드).
- `maps/map1.pgm`, `map1.yaml`, `map2.pgm`, `map2.yaml` 복사(기존 `map.pgm`/
  `map.yaml`은 이미 있었음).
- `src/aed_bringup/config/explore_aed.yaml` 복사.
- `src/aed_bringup/CMakeLists.txt`에 최상위 `maps/`를
  `share/aed_bringup/maps`로 설치하는 규칙 추가 (`map_navigation.launch.py`가
  `get_package_share_directory('aed_bringup')/maps/map.yaml`을 기본값으로
  참조하기 때문에 필요) — 기존에 만든 `localization_view.launch.py` 설치
  규칙은 그대로 유지.
- `dock_poses.yaml`/`nav2_aed.yaml`은 두 브랜치가 이미 동일해서 별도
  작업 없음.
- 빌드 확인, import 확인, `--show-args`로 launch 구조 검증 완료.

실행: `ros2 launch turtlebot4_map_navigation map_navigation.launch.py
namespace:=robot1` — 이제 jaehyeon 저장소 안에서 바로 된다 (별도
`multi_amr_aed_woduqAMR` 폴더 갈 필요 없음).

`multi_robot_emergency`, `central_dispatch.launch.py`는 이번 요청
범위(맵+초기위치 launch)와 무관해서 안 가져옴 — 필요해지면 그때 추가.

## woduqAMR odom 보정 커밋 추가 반영

이전에 `turtlebot4_map_navigation`을 복사해온 뒤 woduqAMR 브랜치에 새 커밋
(`bd1cd09 Stabilize multi-robot Nav2 dispatch and localization`)이 더
올라왔는데, jaehyeon에는 그 시점 스냅샷만 있어서 반영이 안 되고 있었음
(정합률 39.9%였던 근본 원인 — 언도킹 후 로봇이 ~180도 돌아가는데 도킹
방향 5.3도로만 초기화하던 문제 — 을 고친 odom 기반 보정 로직이 빠져있었음).

`multi_amr_aed_woduqAMR`에서 `git pull`로 새 커밋 받아서 diff 확인 후
jaehyeon에 반영:
- `localization_initializer.py`(odom 기반 보정 로직 대폭 추가),
  `navigation_initializer.py`, `map_navigation.launch.py`,
  `turtlebot4_map_navigation/package.xml`
- `aed_bringup/config/nav2_aed.yaml`: `obstacle_min_range` 0.0→0.20
  (로봇 자기 몸통이 ~0.12m에서 잡혀서 도크 근처 costmap이 99로 뜨던 문제)
- `maps/map.pgm` 재생성분

빌드 확인, `odom-adjusted undocked pose` 로그 문구가 소스에 있는 것 확인.
`--symlink-install`로 빌드해서 앞으로 `multi_amr_aed_woduqAMR`에 커밋이
더 올라오면 다시 diff 확인 후 필요한 파일만 복사하는 방식 반복 필요
(정식 병합 전까지는 자동 동기화 안 됨).

## robot1 카메라 장애 발견 + fallback 다음 단계 계획 (코드 변경 없음)

robot1에서 fallback 컨트롤러 실제 테스트 중 로봇이 안 움직이는 증상 발견.
진단 결과 코드 문제가 아니라 **robot1의 OAK-D 카메라가 RGB/depth 둘 다
발행을 안 하고 있었음** (SSH로 직접 확인: `oakd_container` 프로세스는
떠 있지만 로컬에서도 토픽이 하나도 안 잡힘 — 카메라 파이프라인 자체가
죽은 상태). fallback의 depth-safety 로직은 "깊이 데이터 없으면 무조건
정지"라 설계대로 안전하게 동작한 것 — 로직 버그 아님. 재시작 시도했으나
사용자가 직접 고치기로 해서 보류. 이 시점에 로봇 2대 다 다른 팀원이
쓰기 시작해서 실기 테스트는 중단.

카메라 수리 이후를 위해 `docs/lidar-fallback-next-steps.md` 작성 (계획만,
미구현):
1. 깊이 안전정지에서 정적 지도(map)에 이미 점유로 표시된 지점(아는 벽/
   구조물)은 제외하고, 지도에 없는 새 장애물에만 반응하도록 개선 — 픽셀
   3D 역투영(camera_info) → base_link(고정 TF) → map(오도메트리 추정
   pose) → OccupancyGrid 점유 조회.
2. `oakd/stereo/image_raw` 원본 대신 `compressedDepth` 스트림 사용 —
   discovery server 환경에서 원본 depth는 프레임 드랍이 심할 것으로 예상.
3. 설계 원칙 재확인: 목표는 정밀 도달이 아니라 LiDAR 없이도 대략 목표
   방향으로 계속 주행하는 것 — 애매하면 안전(정지) 우선이되, 판정 로직에
   과도하게 정밀도를 투자하지 않는다.

## lidar_replacement_request 노드 신설 (cmd_vel 주행은 보류)

`lidar_fallback_controller`(오도메트리+깊이카메라로 직접 주행)는 당분간
보류하고, 더 단순한 대안을 새로 만듦: LiDAR FAULT 시 그냥 멈추고 "대체
로봇 필요" 신호만 발행, 자동으로 다른 로봇에 보내지는 않음(사람/이후
Mission Manager가 판단하도록 함 — 사용자 확인된 방향).

- `sensor_recovery/replacement_request.py` 신설. FAULT 시: `/cmd_vel` 0
  발행 + `navigate_to_pose` 활성 goal 취소(기존과 동일한 zero-goal-id
  방식) + `/plan` 마지막 지점을 목적지로 저장 + `replacement_needed`
  (Bool, latched) true 발행 + `pending_goal`(PoseStamped) 발행. ALIVE 시:
  `replacement_needed` false로 되돌리고 저장해둔 목적지로 Nav2 재개.
- `setup.py`에 `lidar_replacement_request` 콘솔 스크립트 추가. 기존
  의존성으로 충분해서 `package.xml` 변경 없음.
- 단일 프로세스 통합 테스트로 FAULT→정지+신호+목적지 발행, ALIVE→신호
  해제+재개시도 확인 완료.
- `lidar_fallback_controller`와는 동시에 같은 로봇에서 실행하면 안 됨
  (둘 다 FAULT 시 Nav2 goal 취소 + cmd_vel 발행을 시도해서 충돌) — README에
  명시.

## lidar_replacement_request: ALIVE 시 자동 재개 제거

사용자 지적: ALIVE 때 저장된 목적지로 자동 재출발하는 기존 동작은 Mission
Manager가 그 사이 다른 로봇으로 재할당했을 경우와 충돌할 수 있음(두 로봇이
같은 목적지로 동시 출발 위험). 동의하고 수정:

- `auto_resume_on_recovery` 파라미터 추가(기본 `false`). 기본값에서는
  ALIVE 시 `replacement_needed=false`만 발행하고 "Mission Manager/운영자
  명령 대기" 로그만 남김 — 재개 시도 자체를 안 함.
- `true`로 켜면 기존 동작(저장된 목적지로 자동 재개) 그대로 유지 — Mission
  Manager 없이 이 노드만 단독 테스트할 때 쓰는 용도로 남겨둠.
- FAULT 시 정지+goal취소+신호발행은 변경 없음(이건 항상 안전하게 수행해야
  하는 부분이라 게이팅 안 함).
- 단일 프로세스 통합 테스트를 두 모드(`auto_resume_on_recovery` false/true)
  각각 돌려서 둘 다 검증.

## lidar_fallback_controller 전면 보강 (상태 머신 + 실패 판정)

사용자의 상세 코드 리뷰를 그대로 반영해 `lidar_fallback_controller`를
"속도 명령 계산 가능" 단계에서 "성공/실패를 판정하고 실패 시 대체 로봇을
요청" 단계로 끌어올림. 리뷰에서 지적된 핵심 문제 3가지: (1) lookahead가
경로 처음부터 매번 재탐색해서 곡선/왕복 경로에서 과거 지점을 다시 고를
위험, (2) depth 안전정지가 단일 최소값이라 노이즈 한 픽셀에 취약하고
좌우 장애물을 못 봄, (3) "정지 상태"가 대기인지 실패인지 구분이 안 됨.

**`path_follow_control.py` 재작성**:
- `advance_waypoint`(매번 처음부터 스캔) 제거 → `find_closest_index`(이전
  인덱스보다 뒤로 안 감, monotonic) + `select_lookahead_target`(그 인덱스부터
  누적 경로거리로 lookahead 지점 선택)로 교체.
- `path_deviation_m` 추가 — 경로 전체에 대한 최단 수선 거리(횡방향 이탈).
- `compute_cmd_vel`에 하드 컷오프 추가 — heading error 60도 초과 시 직진
  성분 무조건 0 (기존 `cos()` 스케일링만으로는 애매하다는 지적 반영).
- `rate_limit` 추가 — 가속도 제한용.
- `is_stale`/`time_regressed` 추가 — odom/depth 타임아웃, 시간 역행 감지.
- `min_depth_in_roi`(단일 min값) 제거 → `evaluate_depth_safety`(ROI 유효
  픽셀 비율/근접 픽셀 비율 기준, `CLEAR`/`OBSTACLE`/`STALE`/
  `INSUFFICIENT_DATA` 반환) + `worst_depth_result`(여러 ROI 결과 중 가장
  심각한 것 선택)로 교체.
- 관련 단위테스트 전면 재작성(53개, 회전 오도메트리 합성·yaw wrap·
  monotonic lookahead·경로 이탈·depth 픽셀비율 등).

**`fallback_state_machine.py` 신설**: `IDLE/STARTING/ACTIVE/BLOCKED/
SUCCEEDED/FAILED` 상태와 `next_fallback_state`(순수 함수, 실패 조건 우선순위:
경로/앵커 없음 > odom stale > stuck > 경로 이탈 > blocked timeout 초과).
`SUCCEEDED`/`FAILED`는 terminal. 단위테스트 12개.

**`fallback_path_follower.py` 전면 재작성**:
- 로봇별 최근접 인덱스(`_closest_index`) 추적, 좌/중앙/우 depth ROI 분리
  후 계산된 회전 방향 쪽만 추가로 확인, stuck 판정(cmd_vel은 보냈는데
  `stuck_distance_m`도 안 움직임), 경로 이탈 계산, `fallback_state` 토픽
  발행(latched).
- `FAILED`가 되면 그 자리에서 직접 `replacement_needed=true` +
  `pending_goal` 발행 (별도 coordinator 없이 `lidar_replacement_request`와
  같은 토픽 계약 재사용 — 리뷰에서 제시한 두 옵션 중 "직접 발행"을 선택,
  간단함 우선).
- LiDAR ALIVE 시 즉시 재개하지 않고 새 `/amcl_pose` 수신(AMCL 재수렴,
  `reconvergence_timeout_sec` 타임아웃 있음)까지 대기 후, 이번 시도에서
  이미 대체 요청을 보냈으면(`replacement_dispatched`) 자동 재개 안 하고
  Mission Manager 대기, 아니면 기존처럼 자동 재개 — `lidar_replacement_request`의
  `auto_resume_on_recovery` 게이팅과 같은 원칙.
- 새 파라미터 다수 추가(odom/depth timeout, blocked/stuck timeout, 가속도
  제한, depth 픽셀비율 임계값 등) — README에 표로 정리.

**검증**: 단위테스트 64개(패키지 전체) 전부 통과. 단일 프로세스 통합
테스트 2종 작성: (1) FAULT→ACTIVE(가속도 램프업 확인)→장애물 지속으로
BLOCKED→FAILED→replacement 발행→ALIVE 시 재개 안 함(게이팅 확인),
(2) 목표 근접 상태에서 즉시 SUCCEEDED, 실패 이력 없는 ALIVE는 재개
시도함(액션 서버 없어 에러 로그로만 확인). 첫 통합 테스트 스크립트에
버그가 있었음(장애물 픽셀을 depth ROI가 보는 세로 중앙 밴드 밖에 찍어서
장애물이 전혀 감지 안 됐던 것 — 실제 노드 코드가 아니라 테스트 스크립트의
ROI 좌표 실수였고, 고쳐서 재확인함). 실제 로봇 테스트는 아직 안 함.

README의 lidar_watchdog "Extension points" 섹션이 실수로 lidar_fallback_controller
아래 붙어있던 것도 이번에 lidar_watchdog 섹션으로 옮겨서 바로잡음.

## mapnav 로그 저장 (Claude가 나중에 확인 가능하도록)

실제 주행 테스트 중 "경로가 이상하다"는 문제를 진단하려 했으나, `ros2
launch`의 `output='screen'` 노드 로그는 화면에만 나가고 파일엔 프로세스
시작/종료 이벤트만 남는다는 걸 확인(`~/.ros/log/*/launch.log`에 실제 WARN/
ERROR 내용이 없었음). woduqAMR 브랜치에 이미 있던 `mapnav()` 별칭(아직
jaehyeon에 안 가져왔던 것)을 가져오면서, 화면 출력을 `$AED_WS/logs/`에
`tee`로 같이 저장하도록 수정. `.gitignore`에 `logs/` 추가.

```bash
mapnav 1   # robot1: map_navigation.launch.py 실행 + 로그를
           # logs/mapnav_robot1_<타임스탬프>.log 로 저장
```

## nav2_aed.yaml: DWB 후진 오작동 수정 (woduqAMR 반영)

실기 테스트에서 로봇이 후진 필요한 상황에서 거의 못 가고 오작동하는 문제를
woduqAMR 팀원이 진단해서 커밋(`5b8752f`)으로 올림. 원인: DWB
`min_vel_x: -0.10`(후진 허용) + 고정 속도 샘플 개수 조합이 `vx=0`(제자리
회전) 샘플을 건너뛰어서, 뒤쪽 목표를 향할 때 무진행/오작동 상태가 됨.
수정: `min_vel_x: 0.0`으로 후진 자체를 비활성화, `vx=0`이 항상 샘플에
포함되도록 함 — 뒤쪽 목표는 이제 제자리 회전 후 전진으로 처리.

jaehyeon의 `src/aed_bringup/config/nav2_aed.yaml`에 그대로 반영(diff 한 줄
확인 후 복사). `--symlink-install`이라 재빌드 불필요.

같은 pull에 있던 `multi_robot_emergency`/`mission_executor.py`의 재시도·
진행상황감시 로직(ETA 계산, blocked timeout 등)은 이번 문제와 무관한
별개 작업(다른 담당자 영역)이라 가져오지 않음.

## lidar_fallback_controller: 스냅샷 경로 재사용 → Nav2 planner_server 재계획으로 전환

사용자 요청(전격 수정): 1) 실제 LiDAR를 꺼서 cmd_vel로만 주행, 2) 안전을
위해 최소 속도, 3) 예전 스냅샷 경로를 그대로 쓰지 말고 마지막 위치→목표
사이 최적 경로를 새로 생성. 추가로 FAULT 직후 cmd_vel 시작 전 1초 정지
요청.

**질문에 대한 답**: LiDAR 꺼진 채로 Nav2 goal만으로 계속 이동하는 건
불가능(권장 안 함) — AMCL이 `/scan` 없이는 오도메트리만으로 굴러가고,
TF가 stale해지면서 controller_server가 "Transform data too old"/
"Controller patience exceeded"를 뱉기 시작함(방금 전 실측 로그에서 LiDAR가
멀쩡해도 몇 초 끊긴 것만으로 봤던 바로 그 에러). 그래서 Nav2를 완전히
우회하는 지금 구조가 맞는 방향.

**구현**: 예전 방식(FAULT 순간의 `/plan`을 그대로 스냅샷 떠서 따라가기)을
제거하고, Nav2 `planner_server`의 `compute_path_to_pose` 액션을 재사용하도록
변경:
- `ComputePathToPose.use_start=true`로 **명시적 시작 pose**(FAULT 시점
  AMCL pose)를 넘겨서, AMCL의 TF가 죽어있어도 정적 지도 기준 재계획이
  되도록 함(전역 코스트맵은 주로 static_layer라 실시간 LiDAR 없이도 동작).
- 목적지는 여전히 FAULT 직전 `/plan`의 마지막 지점에서 추출(경로 전체가
  아니라 끝점만 사용).
- FAULT 직후 `pre_replan_delay_sec`(기본 1초) 동안 완전 정지 후에 재계획
  요청 시작 — 비동기 액션 응답을 기다리는 동안(`_replanning` 플래그)
  상태 머신 평가를 보류해서 "경로 없음"으로 성급하게 FAILED 처리되지
  않게 함. `replan_timeout_sec` 안에 응답 없으면 다음 틱에서 자연스럽게
  FAILED로 이어짐(has_plan=False 경로 재사용).
- 안전 속도 기본값 하향: `max_linear_speed` 0.1→0.05, `max_angular_speed`
  0.4→0.2, 가속도 제한도 비례해서 낮춤.
- 새 파라미터: `pre_replan_delay_sec`, `replan_timeout_sec`, `planner_id`
  (기본 `GridBased`, `nav2_aed.yaml`의 `planner_plugins`와 일치),
  `compute_path_action`.

**검증**: 가짜 `compute_path_to_pose` 액션 서버(직선 경로 반환)를 붙인
단일 프로세스 통합테스트로 1초 정지→`use_start=true`/`planner_id=GridBased`
요청→응답 경로로 `STARTING→ACTIVE`→저속(0.05) 주행 시작까지 전부 확인.
기존 64개 순수 로직 단위테스트는 이번 변경과 무관해서 전부 그대로 통과.
실제 `planner_server`에 대해 재계획이 실제로 성공하는지는 아직 실기
미검증(가짜 서버로만 확인).

## tools/lidar_toggle.sh: 주행 중 fault 재현을 위한 `--allow-undocked` 추가

`lidar_fallback_controller` 실기 테스트를 시작하려니 `scan-off`가
"도킹되어 있지 않아 LiDAR를 끄지 않습니다"로 막힘 — 원래 이 스크립트의
도킹 체크는 "정지 상태에서만 LiDAR를 끈다"는 안전장치인데,
`lidar_fallback_controller`를 검증하려면 정반대로 **주행 중에** LiDAR가
꺼져야 함(그게 이 기능이 다루는 시나리오 자체). 그래서 실수로 주행 중에
끄는 걸 막는 기본값은 유지하되, `--allow-undocked` 플래그를 명시적으로
줬을 때만 도킹 체크를 건너뛰도록(`check_docked`에서 실패 대신 warn 후
통과) 수정. `stop`/`scan-off` 둘 다 적용.

## lidar_fallback_controller: Nav2 planner_server 재계획 → 자체 A* 재계획으로 전환

실기 테스트 결과 보고: "주행 중 라이다를 껐을 때 오류가 많이 발생함" —
직전 라운드에서 만든 `compute_path_to_pose`(`use_start=true`) 재계획도
근본적으로 Nav2에 기대는 방식이라 문제였음. 원인 확인: `nav2_aed.yaml`의
global costmap에 `obstacle_layer`(observation_sources: scan)가 들어있어서
LiDAR가 죽으면 costmap 갱신이 끊기고, AMCL의 `map→odom` TF도 결국
stale해짐 — `use_start`로 AMCL 의존 하나는 피했어도 Nav2 스택 자체가
LiDAR 없이는 성치 않다는 게 진짜 문제였음.

사용자 확인 질문에 대한 답: (3) "cmd vel 제어할 때 경로계산도 가능한지"
→ 가능함, 정적 지도(`map_server`가 발행하는 latched `/map`)와 마지막 AMCL
pose만 있으면 Nav2/TF/costmap 없이도 우리 코드 안에서 직접 경로를 뽑을 수
있음. (4) "최적 경로보다 벽에 안 박게 우선할 수 있는지" → 가능함, 벽까지의
거리(clearance)를 비용에 반영하는 자체 A*로 자연스럽게 구현됨.

**구현**: `nav2_msgs/action/ComputePathToPose` 액션 클라이언트를 완전히
제거하고, 새 순수 모듈 `sensor_recovery/grid_path_planner.py` 추가:
- `OccupancyGridData`: ROS `nav_msgs/OccupancyGrid`의 ROS-free 미러.
- `compute_clearance_field`: 모든 셀→가장 가까운 점유 셀까지 거리를
  멀티소스 BFS로 계산(맵 수신 시 1회만 계산해서 캐싱, 재계획마다 재계산
  안 함).
- `plan_path`: 8방향 A*. `robot_radius_m + hard_margin_m` 이내로 벽에
  붙는 셀은 통행 자체를 금지(단, 시작 셀은 예외 — 고장 시점에 이미 벽
  근처였을 수 있어서 그 자리에서 못 움직이게 되는 걸 막음). `soft_clearance_m`
  이내인 통행 가능 셀엔 `wall_clearance_weight`로 비용 페널티를 줘서
  최단경로보다 여유 있는 통로를 우선 선택하게 함. `allow_unknown_cells`
  (기본 false)로 미탐사 영역 통행 여부 결정.
- `fallback_path_follower.py`는 `<ns>/map`(TRANSIENT_LOCAL)을 구독해서
  맵을 캐싱해두고, FAULT 시 1초 정지(`pre_replan_delay_sec`, 기존 그대로
  유지) 후 `_request_replan()`이 이 A*를 **동기 호출**로 바로 실행 —
  액션 왕복이 없어져서 `_replanning`/`replan_timeout_sec` 같은 비동기
  대기 상태 자체가 필요 없어짐, 관련 코드 삭제.
- 새 파라미터: `robot_radius_m`(0.20, `nav2_aed.yaml`과 일치),
  `hard_margin_m`(0.05), `soft_clearance_m`(0.4), `wall_clearance_weight`
  (2.0), `occupied_threshold`(50), `allow_unknown_cells`(false),
  `map_topic`(map). 제거된 파라미터: `compute_path_action`, `planner_id`,
  `replan_timeout_sec`.

**검증**: `grid_path_planner.py` 단위테스트 14개 신규(개활지 직선,
전체 벽 차단, 부분 벽 우회, 좁은 통로 하드 차단, 넓은 통로 선호, 시작
셀 인플레이션 예외, 미탐사 셀 기본 차단/허용, 경계 밖 시작·목적지,
시작=목적지, 목적지가 벽 위, 클리어런스 필드 값/해상도 스케일링,
사전계산된 클리어런스 재사용) — 전부 통과, 기존 64개 포함 총 78개 전부
통과. flake8 clean, colcon build 성공. 4x4 m 가짜 `/map`을 붙인 단일
프로세스 통합테스트로 1초 정지→(Nav2 없이) 즉시 자체 경로계산→
`STARTING→ACTIVE`→저속(0.05) 주행 시작까지 확인. 실제 로봇에서 진짜
`scan-off` 상태로 이 전체 흐름을 돌려본 적은 아직 없음(로봇1의
`turtlebot4.service`가 별개 문제로 oom-kill 되어 있어 실기 테스트 자체가
막혀 있던 상태).

## tools/lidar_toggle.sh: 주행 중 fault 재현을 위한 `--allow-undocked` 추가

`lidar_fallback_controller` 실기 테스트를 시작하려니 `scan-off`가
"도킹되어 있지 않아 LiDAR를 끄지 않습니다"로 막힘 — 원래 이 스크립트의
도킹 체크는 "정지 상태에서만 LiDAR를 끈다"는 안전장치인데,
`lidar_fallback_controller`를 검증하려면 정반대로 **주행 중에** LiDAR가
꺼져야 함(그게 이 기능이 다루는 시나리오 자체). 그래서 실수로 주행 중에
끄는 걸 막는 기본값은 유지하되, `--allow-undocked` 플래그를 명시적으로
줬을 때만 도킹 체크를 건너뛰도록(`check_docked`에서 실패 대신 warn 후
통과) 수정. `stop`/`scan-off` 둘 다 적용.

## 로봇1 turtlebot4.service oom-kill (환경 문제, 우리 코드와 무관)

`pf 1` preflight에서 `dock_status`/OAK-D/dock·undock 액션이 전부 실패로
나와서 SSH로 로봇1에 직접 들어가 확인함: `turtlebot4.service`(Create3
베이스+OAK-D를 띄우는 systemd 서비스)가 메모리 부족(`oom-kill`)으로
죽어 있었음(`systemctl status` 기준 실패 시각 확인됨). 살아있는 건
`rplidar_composition` 프로세스뿐이라 로봇이 실제로 못 움직이는 상태였음.
우리 `lidar_toggle.sh`/`sensor_recovery` 코드가 만든 문제가 아니라 로봇
온보드 컴퓨터 자체의 메모리 부족 — 사용자가 직접 로봇에서 확인/복구하기로
함. 이게 해결돼야 실기 테스트 재개 가능.

## lidar_watchdog: 네트워크 지터로 인한 오탐 방지 (scan_timeout_sec 1.0 → 5.0)

robot2로 실기 fallback 테스트 중 `scan-off`를 한 번도 안 눌렀는데도
`fallback_state`가 `BLOCKED`로 뜸 → 원인 추적해보니 `lidar_watchdog`
로그에 `FAULT`/`RECOVERING`이 몇 초 간격으로 계속 반복되고 있었음.
사용자 확인: "지금 핑이 불안정해서 1초 이상 라이다 수신이 안될 때도
있어" — 실제 LiDAR 하드웨어 문제가 아니라 디스커버리 서버 네트워크
지터로 `/scan`이 순간적으로 끊긴 걸 watchdog이 진짜 고장으로
오판정하고 있었던 것(`scan_timeout_sec` 기본값 1.0초가 이 환경 기준
너무 타이트함). `lidar_watchdog_node.py`의 `scan_timeout_sec` 기본값을
5.0초로 올림.

추가로 "라이다나 이미지 받을 때 best effort로 받게 해달라"는 요청도
확인함 — `lidar_watchdog_node.py`의 `/scan` 구독과
`fallback_path_follower.py`의 depth 이미지 구독 둘 다 이미
`qos_profile_sensor_data`(RELIABILITY=BEST_EFFORT)를 쓰고 있어서 코드
변경 없음, 확인만 하고 넘어감.

빌드/flake8/pytest(78개) 전부 정상.

## lidar_fallback_controller 경로 추종 안정화

첨부 작업 지시를 기준으로 현재 코드를 다시 분석했다. 기존에도 closest index의
단조 증가와 누적거리 lookahead가 있었지만, 제어 루프가 lookahead 함수의
`target_index`를 `_closest_index`에 덮어써 실제 진행 위치보다 앞을 통과한 것으로
기록하고 있었다. 또한 그 인덱스부터 경로 끝까지 closest를 검색해 U자형/평행
인접 경로의 먼 미래 구간으로 점프할 수 있었다.

- `path_follow_control.py`: `PathProgress`/`update_path_progress` 추가.
  `closest_index`와 `target_index`를 분리하고, 이전 closest 기준 전방 1.0m/
  후방 0.3m의 경로거리 창에서만 검색한다. 정상 진행은 단조 증가하고, 기존
  진행점에서 0.5m 이상 이탈했을 때만 제한된 후방 재획득을 허용한다. 이 세
  값과 60도 회전 우선 기준은 모두 ROS 파라미터로 노출했다. 도착 판정도
  `goal_reached` 순수 함수로 분리했다.
- `grid_path_planner.py`: 대각선 이동 때 두 직교 셀 통행 가능 여부를 확인해
  corner-cutting을 차단. A* 결과는 `simplify_path`에서 supercover
  line-of-sight로 단순화하되, shortcut이 만나는 모든 셀에 기존 occupancy/
  hard-clearance 조건을 그대로 적용해 벽 안전 여유를 보존한다.
- `fallback_path_follower.py`: 새 progress 상태 사용, 목표 도착 시 stale depth
  때문에 완료가 BLOCKED/FAILED로 바뀌지 않게 정지 상태에서 즉시 SUCCEEDED
  판정. `debug_enabled`와 1초 주기 debug 로그를 추가해 추정 pose, closest/
  target index, target, goal 거리, 횡오차, heading error, 최종 cmd_vel, depth,
  fallback state를 확인할 수 있다. RViz용 `fallback_debug/path`, `target`,
  `estimated_pose` 토픽도 추가했다.
- `config/lidar_fallback.yaml`: progress/lookahead/회전 기준과 안전·실패 조건을
  소스 수정 없이 실기 튜닝할 수 있도록 현재 기본값을 한곳에 정리했다.
- 테스트: 기존 78개 + 신규 17개 = 총 95개 통과. U자형 미래 구간 점프 방지,
  과거 점 재선택 방지, 제한적 재획득, goal 처리, path progress window,
  parameterized 회전 우선, 좌/우/근거리 제어, 직선 단순화, obstacle/hard-margin
  shortcut 차단, diagonal corner-cutting 등을 추가 검증. flake8 clean,
  `colcon build --symlink-install --packages-select sensor_recovery` 성공.

실제 robot1은 아직 복구 전이므로 실기 성공으로 간주하지 않는다. 실기에서는
debug 토픽/로그로 인덱스 진행과 횡오차를 보면서 search/reacquire/lookahead 값을
조정해야 한다.

## woduqAMR 최신 Nav2 작업 이식 + progress 판정 보강

`multi_amr_aed_woduqAMR`이 clean한 것을 확인하고 `git pull --ff-only origin
woduqAMR`로 `5b8752f → 389a615`까지 fast-forward했다. 최신 Nav2 관련 변경은
로봇 TF 준비 대기와 crowd keepout/RViz 연결이었다.

- `localization_initializer.py`의 `wait_for_robot_transforms()`를 jaehyeon에
  이식. 최신 실기 로그에서 Nav2 local costmap이 `base_link`/`odom` frame이
  아직 없는 상태로 활성화를 시작해 약 17초간 대기했던 문제를 막는다.
- crowd keepout 변경은 `multi_robot_emergency` 패키지와 전용 RViz를 launch
  시점에 무조건 요구하지만 해당 패키지가 jaehyeon에는 없어 그대로 복사하면
  기존 `mapnav`가 시작부터 깨진다. 일반 Nav2 막힘과도 별개이므로 이번에는
  강제 의존성을 가져오지 않았다. 이후 crowd 기능을 합칠 때 선택 옵션으로
  통합해야 한다.
- `logs/mapnav_robot2_20260807_111948.log` 분석: 첫 목표는 정상 도착했지만
  두 번째 목표는 `movement_time_allowance=20.0`과 정확히 일치하는 간격으로
  `Failed to make progress`가 네 번 반복됐다. 기존
  `SimpleProgressChecker`는 제자리 회전을 진행으로 보지 않는 구조라
  `nav2_controller::PoseProgressChecker`로 교체하고
  `required_movement_angle=0.35 rad`를 명시했다. 실제 정지/구동 실패를
  무한정 숨기지 않도록 20초 timeout과 0.25m 이동 기준은 유지했다.
- DWB `vtheta_samples`를 20→21로 변경. `-max_vel_theta~+max_vel_theta` 대칭
  표본에서 0 각속도 후보가 확실히 포함되도록 해 직선 경로의 불필요한 좌우
  조향 가능성을 줄였다.
- 설정 회귀 테스트 2개 추가(PoseProgressChecker/각도 기준, 홀수 각속도
  표본). `aed_bringup`/`turtlebot4_map_navigation` 빌드 성공,
  `map_navigation.launch.py --show-args` 성공, map_navigation 패키지 테스트
  4 passed/1 skipped.

실기에서 제자리 회전 뒤 실제 전진하는지 확인해야 최종 해결로 판단할 수 있다.
`Transform data too old`와 scan message-filter drop은 별도 네트워크/시간 동기
문제이며 progress checker 변경으로 해결되는 문제가 아니다.

### robot2 12:47 재시험 로그 분석

`logs/mapnav_robot2_20260807_124746.log`에서 새 `PoseProgressChecker`가 실제로
로드된 것을 확인했다. 첫 목표 `(2.77, 0.42)`와 마지막 목표 `(1.24, 0.34)`는
성공했으므로 설정 미적용이나 전체 cmd_vel 통신 장애는 아니다. 문제 목표는
`(2.66, 0.36) → (2.33, 2.38)`이고, 두 좌표 사이에는 지도상의 긴 가로벽이
있어 왼쪽 끝으로 우회해야 한다. 취소 직전 자세는 `(2.29, 0.69)`로 총 약
0.50 m 이동했지만, 각 20초 구간에 0.25 m를 채우지 못해 progress recovery가
세 번 반복됐다.

- `required_movement_radius`를 `0.25 → 0.10 m`로 낮춰 AMCL 수 cm 노이즈보다
  크면서 저속 벽 우회는 진행으로 인정한다. `movement_time_allowance=20 s`는
  유지해 실제 정지는 계속 검출한다.
- `nav_diagnostics`를 map navigation launch에 추가했다. 다음 실기부터 로그에
  `NAV_TRACE`로 DWB `cmd_vel_nav`, smoother 출력 `cmd_vel`, odom/AMCL 자세,
  1초 odom 이동량과 전역 경로 시작/끝/pose 수를 함께 남긴다. 이를 보고 정상
  명령인데 베이스가 못 움직이는지, DWB 자체가 0/극저속 명령을 고르는지 다음
  한 번의 재현으로 구분한다.

### robot2 12:59 NAV_TRACE 재시험

`logs/mapnav_robot2_20260807_125925.log`에서 원인이 확정됐다. 첫 목표는
`cmd_vel_nav≈0.19~0.20 m/s`가 smoother를 거쳐 그대로 전달되고 정상 성공했다.
문제 목표도 처음 4초는 전진했지만 `(2.32, 0.70)` 부근에서 DWB가 회전 명령을
`+0.7 → +1.0 → -0.5 → -0.7 → +0.4 → +0.2 rad/s`로 번갈아 선택한 뒤,
전역 경로가 125~126 pose로 계속 존재하는데도 `cmd_vel_nav=(0, 0)`을 반복했다.
`cmd_vel`도 같은 값이고 odom 이동량도 0이므로 베이스나 smoother가 명령을
잃은 것이 아니라 DWB가 정지 명령을 선택한 것이 직접 원인이다.

- `FollowPath`를 `RegulatedPurePursuitController`로 교체했다. 경로 각도 오차가
  0.60 rad 이상이면 0.60 rad/s로 제자리 정렬하고, 이후 0.25~0.55 m 범위의
  속도 비례 lookahead로 최대 0.20 m/s 추종한다.
- RPP의 collision detection과 1초 전방 충돌 예측은 활성화하고 후진은
  비활성화했다. 따라서 이번 변경은 장애물 회피를 끄는 방식이 아니다.
- `NAV_TRACE`는 그대로 유지해 RPP 출력과 실제 이동을 다음 실기에서 검증한다.

## 다음 할 일

- `lidar_fallback_controller` 실기 풀 체인 테스트: 로봇1 `turtlebot4.service`
  복구 확인 → `scan-off --allow-undocked`로 주행 중 진짜 LiDAR 차단 →
  FAULT 감지 → 자체 A* 재계획 → cmd_vel 저속 주행 → `scan-on` → 복귀까지
- 실제 맵 크기에서 자체 A*/클리어런스 계산 소요 시간 실측 (맵이 크면 순수
  Python 루프라 느려질 수 있음)
- `soft_clearance_m`/`wall_clearance_weight`/`robot_radius_m` 기본값이
  실제 복도 폭에 맞는지 실기로 조정
- `scan-off`/`scan-on` 실제 로봇 라이브 테스트 (아직 미실행, 문법/에러
  경로만 확인)
- `RobotState.msg`에 LiDAR 필드 추가 여부 `aed_interfaces`/`mission_manager`
  담당자와 협의
- Waypoint 순찰 패키지 (아직 착수 안 함)
- Nav2 정지, Mission Manager 연동 (`handle_lidar_fault`/`handle_lidar_recovery`
  구현)

## robot1 Nav2→cmd_vel fallback→Nav2 자동 전환 연결 (2026-08-07)

- watchdog 5초 FAULT와 fallback controller를 함께 실행하는
  `lidar_fallback.launch.py` 추가.
- 활성 Nav2 action status를 확인해 idle 중 오래된 `/plan`으로 출발하는
  동작을 차단했다.
- FAULT 순간 마지막 AMCL+이후 odom delta로 현재 pose를 만들고, 원래 goal과
  최신 Nav2 global path의 현재 위치 이후 구간을 저장한다.
- Nav2 cancel 응답 확인 전에는 0 명령만 유지하며 timeout이면 FAILED 처리한다.
- fallback 속도 입력을 물리 `cmd_vel` 직접 경쟁 발행에서 velocity smoother
  입력 `cmd_vel_nav`로 변경했다.
- 저장 경로 hard corner에 감속/정지/제자리 정렬 상태를 추가했다.
- robot1 실측에서 raw depth는 프레임이 없고 compressedDepth가 약 5~12Hz로
  들어오는 것을 확인해 `compressedDepth` 16UC1 PNG 복원 구독으로 변경했다.
  통합 launch에서 `(704, 704) uint16` depth 수신을 확인했다.
- LiDAR 연속 복구 확인 후 새 AMCL pose를 기다려 기존 goal을 Nav2로 재전송한다.
- 단위테스트 118개 통과, flake8 clean, 두 패키지 build 성공. 주행 중 실제
  LiDAR 차단을 포함한 풀 체인 실기 검증은 아직 남아 있다.

## 저장 Nav2 경로의 LiDAR-on cmd_vel 격리 시험 준비 (2026-08-07)

- grid clearance의 8방향 BFS가 대각선 거리를 과소평가하던 문제를 정확한
  유클리드 거리 변환으로 수정했다. 실제 0.28~0.32m 벽 여유가 0.20m로
  계산되던 구간이 바로잡혀 robot1 route 210구간 모두 0.25m 검사를 통과한다.
- route 시작/끝의 planner 방향 artifact 코너를 제외해 실제 hard corner
  index 45 하나만 정지·제자리 회전 대상으로 남겼다.
- route follower에 AMCL 시작 위치/yaw 검사와 compressedDepth 안전정지를
  추가하고 출력은 `cmd_vel_nav`를 통해 velocity smoother로 전달한다.
- `ROUTE_RESULT`에 goal 대비 최종 odom 추정/AMCL 위치·yaw 오차를 기록한다.
- `tools/test_cmd_vel_route.sh 1`은 dock/undock을 자동 실행하지 않고 follower
  사전검사·시작·결과 로그만 담당한다. 로봇 배치는 사용자가 직접 수행한다.
- 단위테스트 121개, flake8, bash 문법, sensor_recovery build 통과. 실제 로봇
  경로 주행은 사용자가 주변 안전을 확인한 뒤 실행하는 단계로 남겼다.
