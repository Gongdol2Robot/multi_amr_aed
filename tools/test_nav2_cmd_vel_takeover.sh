#!/usr/bin/env bash
set -eo pipefail

test_robot_number="${1:-1}"
if [[ ! "$test_robot_number" =~ ^[12]$ ]]; then
  echo "사용법: $0 [1|2]" >&2
  exit 2
fi

test_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /etc/turtlebot4_discovery/setup.bash
export ROS_SUPER_CLIENT=True
source "$test_workspace/install/setup.bash"
set -u

test_namespace="/robot${test_robot_number}"
test_start_service="$test_namespace/fallback/manual_start"
test_stop_service="$test_namespace/fallback/manual_stop"
test_log_dir="$test_workspace/logs"
test_log="$test_log_dir/nav2_cmd_vel_takeover_robot${test_robot_number}_$(date +%Y%m%d_%H%M%S).log"
test_controller="$test_workspace/install/sensor_recovery/lib/sensor_recovery/lidar_fallback_controller"
test_node_pid=""
test_odom_calibration_args=()
if [[ "$test_robot_number" == "1" ]]; then
  test_odom_calibration_args=(
    -p "odom_translation_scale:=0.986"
    -p "odom_translation_heading_correction_deg:=4.0"
    -p "odom_yaw_delta_scale:=0.92"
  )
fi
mkdir -p "$test_log_dir"

if [[ ! -x "$test_controller" ]]; then
  echo "takeover controller 실행 파일이 없습니다. 먼저 sensor_recovery를 빌드하세요." >&2
  exit 1
fi

stop_test_node() {
  if ros2 service type "$test_stop_service" 2>/dev/null | grep -q 'Trigger'; then
    ros2 service call "$test_stop_service" std_srvs/srv/Trigger '{}' \
      >/dev/null 2>&1 || true
  fi
  if [[ -n "$test_node_pid" ]] && kill -0 "$test_node_pid" 2>/dev/null; then
    kill -INT "$test_node_pid" 2>/dev/null || true
    wait "$test_node_pid" 2>/dev/null || true
  fi
}

trap stop_test_node EXIT
trap 'stop_test_node; exit 130' INT TERM

echo "1/3: 수동 Nav2→cmd_vel takeover controller 실행"
echo "dock/undock과 LiDAR on/off는 수행하지 않습니다."
if [[ "$test_robot_number" == "1" ]]; then
  echo "robot1 실측 odom 보정 적용: 거리×0.986, 진행 방향 +4.0°, 회전×0.92"
else
  echo "robot2는 아직 실측 보정값이 없어 무보정으로 실행합니다."
fi
echo "로그: $test_log"
"$test_controller" --ros-args \
  -r "__ns:=$test_namespace" \
  -r "__node:=manual_takeover_controller" \
  --params-file "$test_workspace/src/sensor_recovery/config/lidar_fallback.yaml" \
  -p "enable_manual_trigger:=true" \
  -p "debug_enabled:=true" \
  -p "resume_nav2_after_failure:=false" \
  "${test_odom_calibration_args[@]}" \
  > >(tee -a "$test_log") 2>&1 &
test_node_pid=$!

test_service_ready=false
for _ in {1..150}; do
  if ros2 service type "$test_start_service" 2>/dev/null | grep -q 'Trigger'; then
    test_service_ready=true
    break
  fi
  if ! kill -0 "$test_node_pid" 2>/dev/null; then
    echo "takeover controller가 비정상 종료됐습니다. 로그: $test_log" >&2
    exit 1
  fi
  sleep 0.1
done
if [[ "$test_service_ready" != true ]]; then
  echo "15초 안에 manual_start 서비스를 찾지 못했습니다. 로그: $test_log" >&2
  exit 1
fi

echo "2/3: RViz2에서 Nav2 Goal을 지정하고 로봇이 실제 주행할 때까지 기다리세요."
read -r -p "주행 중간 takeover를 실행하려면 Enter: "

test_start_response="$(
  ros2 service call "$test_start_service" std_srvs/srv/Trigger '{}' 2>&1
)"
echo "$test_start_response" | tee -a "$test_log"
if ! grep -q 'success=True' <<<"$test_start_response"; then
  echo "takeover 시작이 거부됐습니다. Nav2가 실제 주행 중인지 확인하세요." >&2
  echo "로그: $test_log" >&2
  exit 1
fi

echo "3/3: Nav2 취소 확인 후 저장된 남은 경로를 cmd_vel로 추종합니다."
echo "중지하려면 Ctrl+C를 누르세요."
while kill -0 "$test_node_pid" 2>/dev/null; do
  if grep -q 'fallback_state: .* -> SUCCEEDED' "$test_log"; then
    echo "takeover 경로 주행 성공. 로그: $test_log"
    break
  fi
  if grep -q 'fallback_state: .* -> FAILED\|fallback FAILED:' "$test_log"; then
    echo "takeover 경로 주행 실패. 로그: $test_log" >&2
    exit 1
  fi
  sleep 0.2
done

stop_test_node
test_node_pid=""
