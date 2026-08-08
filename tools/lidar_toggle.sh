#!/usr/bin/env bash
# TurtleBot4 RPLidar 제어 (lidar_watchdog 테스트용)
#
# 사용법:
#   tools/lidar_toggle.sh status   [robot_number]
#   tools/lidar_toggle.sh stop     [robot_number] [--yes] [--allow-undocked]
#   tools/lidar_toggle.sh start    [robot_number]           # 모터만 재시작
#   tools/lidar_toggle.sh scan-off [robot_number] [--yes] [--allow-undocked]
#   tools/lidar_toggle.sh scan-on  [robot_number]           # 드라이버 프로세스 재실행
#
# stop/start: rplidar_ros의 공식 서비스 /stop_motor, /start_motor(std_srvs/Empty).
#   모터만 멈추고 드라이버는 계속 떠 있어서, 이 로봇 펌웨어 기준으로는
#   모터가 멈춰도 /scan이 계속(고정된 값으로) 발행된다 — 실제 fault 재현이 안 됨.
# scan-off/scan-on: SSH로 rplidar_composition 프로세스 자체를 kill/재실행한다.
#   이 launch에는 respawn이 걸려있지 않아(turtlebot4_bringup/launch/rplidar.launch.py
#   확인함) kill하면 /scan이 완전히 끊기고 자동으로 안 살아난다. systemd 서비스
#   전체(turtlebot4.service)는 건드리지 않으므로 base/oakd/joy 등 다른 노드에는
#   영향 없다. SSH 접속 정보는 저장소의 .env.robots에서 읽는다.
#
# 공통 안전 장치:
# - stop/scan-off 전에 로봇이 도킹 상태(is_docked=true)인지 확인한다. 주행
#   중일 수 있으면 중단한다. lidar_fallback_controller를 주행 중에 테스트하려면
#   (원래 목적이 그것임) --allow-undocked로 이 체크를 의도적으로 건너뛴다 —
#   실수로 주행 중에 끄는 걸 막으려는 장치라 기본값은 여전히 거부이고, 명시적
#   플래그로만 우회 가능하다.
# - --yes 없이는 실행 전에 사용자 확인을 받는다. 같은 로봇을 다른 팀원이
#   쓰고 있을 수 있기 때문이다.

usage() {
  echo "사용: tools/lidar_toggle.sh {status|stop|start|scan-off|scan-on} [robot_number] [--yes] [--allow-undocked]" >&2
}

ACTION="${1:-}"
if [[ -z "$ACTION" ]]; then
  usage
  exit 2
fi
shift

ROBOT_NUMBER=1
YES=0
ALLOW_UNDOCKED=0
for arg in "$@"; do
  if [[ "$arg" == "--yes" ]]; then
    YES=1
  elif [[ "$arg" == "--allow-undocked" ]]; then
    ALLOW_UNDOCKED=1
  elif [[ "$arg" =~ ^[0-9]+$ ]]; then
    ROBOT_NUMBER="$arg"
  fi
done

if [[ ! "$ROBOT_NUMBER" =~ ^([0-9]|1[01])$ ]]; then
  echo "오류: 로봇 번호는 0~11이어야 합니다." >&2
  exit 2
fi

ROBOT_NS="/robot${ROBOT_NUMBER}"

source /opt/ros/humble/setup.bash
if [[ -f /etc/turtlebot4_discovery/setup.bash ]]; then
  source /etc/turtlebot4_discovery/setup.bash
else
  echo "오류: /etc/turtlebot4_discovery/setup.bash 가 없습니다. 실제 로봇 네트워크에 연결되어 있는지 확인하세요." >&2
  exit 2
fi
export ROS_SUPER_CLIENT=True

# ament setup 스크립트는 초기화되지 않은 변수를 내부에서 참조하므로
# 모든 source가 끝난 뒤에 nounset을 켠다.
set -u

GREEN='\033[1;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
RESET='\033[0m'
ok()   { echo -e "${GREEN}[OK]${RESET} $*"; }
warn() { echo -e "${YELLOW}[주의]${RESET} $*"; }
fail() { echo -e "${RED}[실패]${RESET} $*" >&2; }

STOP_SERVICE="$ROBOT_NS/stop_motor"
START_SERVICE="$ROBOT_NS/start_motor"

check_docked() {
  local msg
  if (( ALLOW_UNDOCKED == 1 )); then
    warn "$ROBOT_NS dock 상태 조회를 생략합니다. --allow-undocked 로 주행 중 LiDAR fault 시험을 명시적으로 허용했습니다."
    return 0
  fi
  msg="$(timeout 5 ros2 topic echo "$ROBOT_NS/dock_status" --once 2>&1)" || {
    fail "$ROBOT_NS/dock_status 를 읽을 수 없습니다. 로봇이 켜져 있는지 확인하세요."
    return 1
  }
  if [[ "$msg" != *"is_docked: true"* ]]; then
    fail "$ROBOT_NS 가 도킹되어 있지 않습니다. 주행 중일 수 있어 LiDAR를 끄지 않습니다. (의도적으로 주행 중 테스트하려면 --allow-undocked)"
    return 1
  fi
  ok "$ROBOT_NS 도킹 상태 확인됨 (정지 상태)"
}

# scan-off / scan-on 전용: robot_number -> discovery server 밖의 실제 로봇 IP.
robot_ip() {
  case "$1" in
    1) echo 192.168.107.101 ;;
    2) echo 192.168.107.102 ;;
    *) return 1 ;;
  esac
}

load_robot_env() {
  local env_file
  env_file="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.env.robots"
  if [[ ! -f "$env_file" ]]; then
    fail "$env_file 가 없습니다. ROBOT_SSH_USER / ROBOT_SSH_PASSWORD 를 채운 .env.robots 를 만드세요."
    return 1
  fi
  # shellcheck disable=SC1090
  source "$env_file"
  if [[ -z "${ROBOT_SSH_USER:-}" || -z "${ROBOT_SSH_PASSWORD:-}" ]]; then
    fail "$env_file 에 ROBOT_SSH_USER / ROBOT_SSH_PASSWORD 가 없습니다."
    return 1
  fi
  if ! command -v sshpass >/dev/null 2>&1; then
    fail "sshpass 가 설치되어 있지 않습니다: sudo apt install -y sshpass"
    return 1
  fi
}

ssh_cmd() {
  local ip="$1" remote_cmd="$2"
  # 원격 background 프로세스가 SSH의 stdin을 잡고 있거나 네트워크 응답이
  # 끊겨도 호출자가 무한히 기다리지 않도록 전체 실행 시간도 제한한다.
  timeout 15 sshpass -p "$ROBOT_SSH_PASSWORD" ssh \
    -o StrictHostKeyChecking=no -o ConnectTimeout=8 \
    "$ROBOT_SSH_USER@$ip" "$remote_cmd"
}

RPLIDAR_BIN="/opt/ros/humble/lib/rplidar_ros/rplidar_composition"

find_lidar_pids() {
  local ip="$1"
  # pgrep -f는 검색 문자열을 포함한 원격 `bash -c` 자체까지 잡을 수 있다.
  # ps의 executable 필드($2)가 실제 바이너리와 정확히 같은 행만 고른다.
  # PID 하나만 고르지 않고 모두 반환해서 중복 드라이버도 빠짐없이 종료한다.
  ssh_cmd "$ip" "ps -eo pid=,args= | awk -v bin='${RPLIDAR_BIN}' -v ns='__ns:=${ROBOT_NS}' '\$2 == bin && index(\$0, ns) { print \$1 }'" 2>/dev/null
}

case "$ACTION" in
  status)
    if timeout 5 ros2 service type "$STOP_SERVICE" >/dev/null 2>&1; then
      ok "$STOP_SERVICE / $START_SERVICE 서비스 확인됨"
    else
      fail "$ROBOT_NS 에서 stop_motor 서비스를 찾을 수 없습니다. rplidar_composition 이 떠 있는지 확인하세요."
      exit 1
    fi
    ;;
  stop)
    check_docked || exit 1
    if (( YES == 0 )); then
      read -r -p "$ROBOT_NS 의 LiDAR 모터를 정지합니다. 다른 팀원이 사용 중이 아닌지 확인했습니까? [y/N] " reply
      [[ "$reply" =~ ^[Yy]$ ]] || { echo "취소했습니다."; exit 1; }
    fi
    if timeout 5 ros2 service call "$STOP_SERVICE" std_srvs/srv/Empty "{}" >/dev/null 2>&1; then
      ok "$ROBOT_NS LiDAR 모터 정지 요청 전송"
      warn "watchdog 이 scan_timeout_sec 이후 FAULT 로 판정하는지 확인하세요."
    else
      fail "$STOP_SERVICE 호출 실패"
      exit 1
    fi
    ;;
  start)
    if timeout 5 ros2 service call "$START_SERVICE" std_srvs/srv/Empty "{}" >/dev/null 2>&1; then
      ok "$ROBOT_NS LiDAR 모터 재시작 요청 전송"
      warn "watchdog 이 RECOVERING 을 거쳐 ALIVE 로 복귀하는지 확인하세요."
    else
      fail "$START_SERVICE 호출 실패"
      exit 1
    fi
    ;;
  scan-off)
    check_docked || exit 1
    ip="$(robot_ip "$ROBOT_NUMBER")" || { fail "robot$ROBOT_NUMBER 의 IP를 모릅니다 (1, 2만 지원)"; exit 1; }
    load_robot_env || exit 1
    if (( YES == 0 )); then
      read -r -p "$ROBOT_NS 의 LiDAR 드라이버 프로세스를 SSH로 kill합니다 (모터 정지보다 강함, /scan 완전히 끊김). 다른 팀원이 사용 중이 아닌지 확인했습니까? [y/N] " reply
      [[ "$reply" =~ ^[Yy]$ ]] || { echo "취소했습니다."; exit 1; }
    fi
    pids="$(find_lidar_pids "$ip")"
    if [[ -z "$pids" ]]; then
      fail "$ROBOT_NS 에서 rplidar_composition 프로세스를 찾을 수 없습니다."
      exit 1
    fi
    while IFS= read -r pid; do
      if [[ ! "$pid" =~ ^[0-9]+$ ]]; then
        fail "잘못된 LiDAR PID가 감지됐습니다: $pid"
        exit 1
      fi
    done <<< "$pids"
    if ! ssh_cmd "$ip" "kill -- $(tr '\n' ' ' <<< "$pids")"; then
      fail "kill 실패"
      exit 1
    fi
    ok "$ROBOT_NS rplidar_composition 종료 요청 전송 (pid: $(tr '\n' ' ' <<< "$pids"))"

    # kill(TERM)이 실제로 반영됐는지 확인한다. 이전에는 이 확인이 없어
    # 프로세스가 살아 있어도 OFF 성공으로 표시될 수 있었다.
    stopped=0
    for _ in {1..10}; do
      sleep 0.3
      remaining="$(find_lidar_pids "$ip")"
      if [[ -z "$remaining" ]]; then
        stopped=1
        break
      fi
    done
    if (( stopped == 0 )); then
      fail "$ROBOT_NS LiDAR 프로세스가 종료되지 않았습니다 (남은 pid: $(tr '\n' ' ' <<< "$remaining"))."
      exit 1
    fi
    ok "$ROBOT_NS 원격 LiDAR 프로세스 0개 확인"

    # ROS 그래프의 publisher endpoint는 한동안 남을 수 있으므로 count가 아니라
    # 새 LaserScan 메시지가 실제로 오는지를 검사한다.
    if timeout 2 ros2 topic echo "$ROBOT_NS/scan" --once --qos-reliability best_effort >/dev/null 2>&1; then
      fail "$ROBOT_NS/scan 새 데이터가 아직 수신됩니다. 다른 LiDAR 발행자가 남아 있습니다."
      exit 1
    fi
    ok "$ROBOT_NS/scan 새 데이터 단절 확인"
    warn "watchdog 이 scan_timeout_sec 이후 FAULT 로 판정하는지 확인하세요."
    warn "되살리려면: tools/lidar_toggle.sh scan-on $ROBOT_NUMBER"
    ;;
  scan-on)
    ip="$(robot_ip "$ROBOT_NUMBER")" || { fail "robot$ROBOT_NUMBER 의 IP를 모릅니다 (1, 2만 지원)"; exit 1; }
    load_robot_env || exit 1
    existing="$(find_lidar_pids "$ip")"
    if [[ -n "$existing" ]]; then
      ok "$ROBOT_NS rplidar_composition 이미 실행 중 (pid $existing)"
      # auto_standby 상태에서는 프로세스가 살아 있어도 모터와 /scan이 멈출
      # 수 있다. 프로세스 존재만으로 성공 처리하지 말고 모터를 깨운 뒤 실제
      # LaserScan 수신까지 확인한다.
      if timeout 5 ros2 service call "$START_SERVICE" std_srvs/srv/Empty "{}" >/dev/null 2>&1; then
        ok "$ROBOT_NS LiDAR 모터 재시작 요청 전송"
      else
        warn "$START_SERVICE 호출을 확인하지 못했습니다. /scan 수신으로 최종 판정합니다."
      fi
      echo "  -> $ROBOT_NS/scan 실제 데이터 확인 중 (최대 12초)..."
      if timeout 12 ros2 topic echo "$ROBOT_NS/scan" sensor_msgs/msg/LaserScan \
          --once --qos-reliability best_effort >/dev/null 2>&1; then
        ok "$ROBOT_NS/scan 실제 데이터 수신 확인됨"
        warn "watchdog 이 FAULT -> RECOVERING -> ALIVE 로 복귀하는지 확인하세요."
        exit 0
      fi
      fail "LiDAR 프로세스는 실행 중이지만 $ROBOT_NS/scan 데이터가 안 옵니다."
      ssh_cmd "$ip" "tail -n 30 /tmp/rplidar_restart.log 2>/dev/null || true" || true
      exit 1
    fi
    # 로봇의 discovery/도메인 환경(/etc/turtlebot4/setup.bash)을 반드시 먼저
    # source해야 한다 — 안 하면 새 프로세스가 격리된 기본 도메인으로 붙어서
    # 하드웨어 연결은 성공해도 ROS 그래프에 안 잡힌다 (2026-08-06 robot1에서
    # 실측: 프로세스는 떠 있는데 /scan 퍼블리셔 count 0, 노드도 안 보임).
    # 그리고 원래 launch가 쓰는 바이너리를 직접 실행한다 — "ros2 run"으로
    # 띄우면 부모(ros2 run)와 실제 바이너리가 별도 PID로 떠서, 부모만 kill
    # 하면 자식이 시리얼 포트를 문 채로 안 죽고 남는다.
    # setsid -f와 stdin/stdout/stderr 분리로 드라이버를 SSH 세션에서 완전히
    # 떼어낸다. 이전 nohup 방식은 stdin이 SSH 채널에 남아 두 번째 Enter 뒤
    # 로봇에서는 LiDAR가 켜져도 로컬 터미널이 끝나지 않았다.
    launch_cmd="source /etc/turtlebot4/setup.bash && \
setsid -f ${RPLIDAR_BIN} --ros-args \
-r __node:=rplidar_composition -r __ns:=${ROBOT_NS} \
-p serial_port:=/dev/RPLIDAR -p serial_baudrate:=115200 \
-p frame_id:=rplidar_link -p inverted:=false \
-p angle_compensate:=true -p auto_standby:=true \
 </dev/null >/tmp/rplidar_restart.log 2>&1; sleep 2"
    if ! ssh_cmd "$ip" "$launch_cmd"; then
      fail "LiDAR 원격 재실행 명령이 실패하거나 15초를 초과했습니다."
      exit 1
    fi
    result="$(find_lidar_pids "$ip")"
    if [[ -z "$result" ]]; then
      fail "재실행 확인 실패. 로봇의 /tmp/rplidar_restart.log 를 확인하세요."
      ssh_cmd "$ip" "tail -n 30 /tmp/rplidar_restart.log 2>/dev/null || true" || true
      exit 1
    fi
    ok "$ROBOT_NS rplidar_composition 재실행됨 (pid $result)"
    echo "  -> ROS 그래프에 실제로 잡히는지 확인 중 (최대 12초)..."
    if timeout 12 ros2 topic echo "$ROBOT_NS/scan" sensor_msgs/msg/LaserScan \
        --once --qos-reliability best_effort >/dev/null 2>&1; then
      ok "$ROBOT_NS/scan 실제 데이터 수신 확인됨"
      warn "watchdog 이 FAULT -> RECOVERING -> ALIVE 로 복귀하는지 확인하세요."
    else
      fail "프로세스는 떠 있지만 $ROBOT_NS/scan 데이터가 안 옵니다. 로봇의 /tmp/rplidar_restart.log 와 ROS_DOMAIN_ID/ROS_DISCOVERY_SERVER 환경을 확인하세요."
      ssh_cmd "$ip" "tail -n 30 /tmp/rplidar_restart.log 2>/dev/null || true" || true
      exit 1
    fi
    ;;
  *)
    usage
    exit 2
    ;;
esac
