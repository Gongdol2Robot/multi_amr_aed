#!/usr/bin/env bash
set -eo pipefail

test_robot_number="${1:-1}"
if [[ ! "$test_robot_number" =~ ^[12]$ ]]; then
  echo "사용법: $0 [1|2]" >&2
  exit 2
fi

test_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_toggle="$test_workspace/tools/lidar_toggle.sh"
test_lidar_disabled=false
test_log_dir="$test_workspace/logs"
mkdir -p "$test_log_dir"
test_log="$test_log_dir/lidar_fault_cycle_robot${test_robot_number}_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$test_log") 2>&1

source /opt/ros/humble/setup.bash
if [[ -f /etc/turtlebot4_discovery/setup.bash ]]; then
  source /etc/turtlebot4_discovery/setup.bash
fi
export ROS_SUPER_CLIENT=True

wait_for_lidar_fault() {
  # 한 subscriber를 충분히 오래 유지한다. 매번 2초짜리 CLI를 새로 띄우면
  # 불안정한 DDS 환경에서 discovery가 끝나기 전에 종료되어 실제 FAULT를
  # 놓칠 수 있다. TRANSIENT_LOCAL로 현재 latched 상태도 즉시 확인한다.
  timeout 30 ros2 topic echo \
    "/robot${test_robot_number}/lidar_state" \
    std_msgs/msg/String \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --filter "m.data == 'FAULT'" \
    --once >/dev/null 2>&1
}

restore_lidar_on_exit() {
  if [[ "$test_lidar_disabled" == true ]]; then
    echo
    echo "시험이 중단되어 robot${test_robot_number} LiDAR 복구를 시도합니다."
    "$test_toggle" scan-on "$test_robot_number" || true
    test_lidar_disabled=false
  fi
}

trap restore_lidar_on_exit EXIT INT TERM

echo "LiDAR 실제 고장/복구 조작 시험: robot${test_robot_number}"
echo "시험 로그: $test_log"
echo "이 스크립트는 LiDAR OFF/ON만 제어하며 Nav2나 fallback 노드를 실행하지 않습니다."
echo "터미널 1에서 mapnav ${test_robot_number}을 실행하고 RViz Nav2 주행을 먼저 시작하세요."
echo
read -r -p "Nav2가 실제 주행 중이면 Enter를 눌러 LiDAR를 끄세요: "

# OFF 검증 도중 실패해도 EXIT trap이 반드시 LiDAR를 복구하도록 먼저 표시한다.
test_lidar_disabled=true
"$test_toggle" scan-off "$test_robot_number" --yes --allow-undocked

echo
echo "watchdog의 LiDAR FAULT 판정을 확인하는 중..."
if ! wait_for_lidar_fault; then
  echo "[경고] 30초 동안 CLI에서 /robot${test_robot_number}/lidar_state FAULT를 받지 못했습니다." >&2
  echo "DDS 확인 실패만으로 시험을 중단하거나 LiDAR를 자동 복구하지 않습니다." >&2
  echo "터미널 1의 'ALIVE -> FAULT'와 fallback 전환 로그를 직접 확인하세요." >&2
else
  echo "[OK] LiDAR 프로세스·scan 단절·watchdog FAULT까지 모두 확인했습니다."
fi
echo "자동 fallback 전환 로그를 확인하세요."
echo "fallback이 목적지에 도착해 터미널 1에 'fallback_state: ACTIVE -> SUCCEEDED'가"
echo "표시될 때까지 LiDAR를 다시 켜지 마세요."
echo
read -r -p "fallback SUCCEEDED와 로봇 정지를 확인했으면 Enter를 눌러 LiDAR를 켜세요: "

"$test_toggle" scan-on "$test_robot_number"
test_lidar_disabled=false

echo
echo "LiDAR를 복구했습니다. 로봇은 정지 상태를 유지합니다."
echo "터미널 1에서 RECOVERY_POSITION_CHECK 로그를 확인하세요."
