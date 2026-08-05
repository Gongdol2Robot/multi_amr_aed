#!/usr/bin/env bash
# TurtleBot4 Nav2 lifecycle 복구 도구
#
# 사용법:
#   tools/nav_recovery.sh        # robot2 복구
#   tools/nav_recovery.sh 3      # robot3 복구
#
# COMM이 끊긴 상태에서는 lifecycle을 변경하지 않는다. 통신이 살아 있으면
# 현재 상태를 읽어 active는 유지하고 inactive/unconfigured만 복구한다.

ROBOT_NUMBER="${1:-2}"
if [[ ! "$ROBOT_NUMBER" =~ ^([0-9]|1[01])$ ]]; then
  echo "오류: 로봇 번호는 0~11이어야 합니다." >&2
  exit 2
fi

ROBOT_NS="/robot${ROBOT_NUMBER}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${AED_WS:-$(dirname "$SCRIPT_DIR")}"

# Discovery server 환경에서는 lifecycle 서비스 응답이 수십 초 늦어질 수 있다.
# 필요하면 실행할 때 환경변수로 더 늘릴 수 있다.
#   NAV_RECOVERY_STATE_TIMEOUT=90 NAV_RECOVERY_TRANSITION_TIMEOUT=180 \
#     tools/nav_recovery.sh
STATE_TIMEOUT="${NAV_RECOVERY_STATE_TIMEOUT:-48}"
TRANSITION_TIMEOUT="${NAV_RECOVERY_TRANSITION_TIMEOUT:-80}"
COMM_TIMEOUT="${NAV_RECOVERY_COMM_TIMEOUT:-32}"
ACTION_TIMEOUT="${NAV_RECOVERY_ACTION_TIMEOUT:-40}"

source /opt/ros/humble/setup.bash

if [[ -f /etc/turtlebot4_discovery/setup.bash ]]; then
  source /etc/turtlebot4_discovery/setup.bash
fi
export ROS_SUPER_CLIENT=True

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  source "$WORKSPACE/install/setup.bash"
else
  echo "오류: $WORKSPACE/install/setup.bash 가 없습니다." >&2
  exit 2
fi

# ROS/ament setup 스크립트는 초기화되지 않은 환경변수를 내부에서 참조한다.
# setup을 모두 source한 뒤에 nounset을 켜야 AMENT_TRACE_SETUP_FILES 오류가 없다.
set -u

readonly STEPS=(
  controller_server
  smoother_server
  planner_server
  behavior_server
  bt_navigator
  waypoint_follower
  velocity_smoother
)

GREEN='\033[1;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
RESET='\033[0m'

ok()   { echo -e "${GREEN}[OK]${RESET} $*"; }
warn() { echo -e "${YELLOW}[주의]${RESET} $*"; }
fail() { echo -e "${RED}[실패]${RESET} $*" >&2; }

get_state() {
  local node="$1" output
  output="$(timeout "$STATE_TIMEOUT" ros2 lifecycle get "$node" 2>&1)" || return 1
  case "$output" in
    active*)       echo active ;;
    inactive*)     echo inactive ;;
    unconfigured*) echo unconfigured ;;
    finalized*)    echo finalized ;;
    *)             return 1 ;;
  esac
}

transition() {
  local node="$1" command="$2" output
  echo "  -> $command"
  output="$(timeout "$TRANSITION_TIMEOUT" ros2 lifecycle set "$node" "$command" 2>&1)" || {
    fail "$node: $command 명령 실패: $output"
    return 1
  }
  if [[ "$output" != *"Transitioning successful"* ]]; then
    fail "$node: $command 전이 실패: $output"
    return 1
  fi
}

echo "=========================================="
echo " Nav2 recovery: $ROBOT_NS"
echo "=========================================="

# node list만 보이는 것은 DDS 캐시일 수 있다. Create3의 실제 메시지를 받아
# COMM과 discovery server가 현재 살아 있는지 확인한다.
echo "[1/4] 로봇 통신 확인"
if ! timeout "$COMM_TIMEOUT" ros2 topic echo "$ROBOT_NS/dock_status" --once >/dev/null 2>&1; then
  fail "$ROBOT_NS/dock_status 메시지가 없습니다."
  echo "COMM 라이트와 Pi/Create3 통신을 복구한 뒤 다시 실행하세요."
  exit 1
fi
ok "Create3 메시지 수신"

echo "[2/4] 현재 lifecycle 상태 확인 및 복구"
FAILED=0
for step in "${STEPS[@]}"; do
  node="$ROBOT_NS/$step"
  state="$(get_state "$node")" || {
    fail "$node 상태 조회 불가 — Nav2 launch가 떠 있는지 확인하세요."
    FAILED=1
    continue
  }

  case "$state" in
    active)
      ok "$node: active (유지)"
      ;;
    inactive)
      warn "$node: inactive"
      transition "$node" activate || FAILED=1
      ;;
    unconfigured)
      warn "$node: unconfigured"
      if transition "$node" configure; then
        transition "$node" activate || FAILED=1
      else
        FAILED=1
      fi
      ;;
    finalized)
      fail "$node: finalized — 프로세스를 재실행해야 합니다."
      FAILED=1
      ;;
  esac
done

echo "[3/4] 최종 lifecycle 검증"
for step in "${STEPS[@]}"; do
  node="$ROBOT_NS/$step"
  state="$(get_state "$node")" || state="응답 없음"
  if [[ "$state" == active ]]; then
    ok "$node: active"
  else
    fail "$node: $state"
    FAILED=1
  fi
done

echo "[4/4] NavigateToPose 액션 서버 검증"
ACTION_INFO="$(timeout "$ACTION_TIMEOUT" ros2 action info "$ROBOT_NS/navigate_to_pose" 2>&1)" || ACTION_INFO=""
SERVER_COUNT="$(sed -n 's/^Action servers: //p' <<<"$ACTION_INFO" | head -n 1)"
if [[ "$SERVER_COUNT" =~ ^[1-9][0-9]*$ ]]; then
  ok "$ROBOT_NS/navigate_to_pose 서버 $SERVER_COUNT개"
else
  fail "$ROBOT_NS/navigate_to_pose 액션 서버가 없습니다."
  FAILED=1
fi

if (( FAILED != 0 )); then
  echo
  fail "자동 복구가 끝나지 않았습니다. nav 터미널 로그를 확인하세요."
  echo "프로세스가 죽었거나 finalized라면 nav 터미널을 종료한 뒤 'nav $ROBOT_NUMBER'로 다시 띄우세요."
  exit 1
fi

echo
ok "Nav2 복구 완료. 미션을 다시 배정할 수 있습니다."
