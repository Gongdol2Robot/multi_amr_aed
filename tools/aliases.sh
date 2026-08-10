# Multi-AMR AED 프로젝트 공통 단축 명령
# 설치:
#   echo 'source ~/rokey_ws/multi_amr_aed/tools/aliases.sh' >> ~/.bashrc
#   source ~/.bashrc

export AED_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

aedenv() {
  source /etc/turtlebot4_discovery/setup.bash
  export ROS_SUPER_CLIENT=True
  source "$AED_WS/install/setup.bash"
}

aedbuild() {
  (cd "$AED_WS" && source /opt/ros/humble/setup.bash && \
    PYTHONNOUSERSITE=1 colcon build --symlink-install "$@")
}

roskill() {
  local uid workspace_pattern
  uid="$(id -u)"
  workspace_pattern="$AED_WS/install/.*/lib/"
  echo "ROS2/Nav2/AED 프로세스 종료 중..."
  timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
  pkill -TERM -u "$uid" -f '(/usr/bin/python3 )?/opt/ros/humble/bin/ros2([[:space:]]|$)' 2>/dev/null || true
  pkill -TERM -u "$uid" -f '/opt/ros/humble/lib/' 2>/dev/null || true
  pkill -TERM -u "$uid" -f "$workspace_pattern" 2>/dev/null || true
  pkill -TERM -u "$uid" -f 'ros2cli\.daemon\.daemonize' 2>/dev/null || true
  sleep 3
  pkill -KILL -u "$uid" -f '/opt/ros/humble/lib/' 2>/dev/null || true
  pkill -KILL -u "$uid" -f "$workspace_pattern" 2>/dev/null || true
  pkill -KILL -u "$uid" -f 'ros2cli\.daemon\.daemonize' 2>/dev/null || true
  echo "종료 완료."
}

robotstart() {
  export ROBOT_NUMBER="${1:-1}"
  export WEBCAM_DEVICE="${2:-0}"
  terminator --no-dbus --config "$AED_WS/tools/terminator_robot.conf" \
    --layout robot_session >/dev/null 2>&1 &
  disown
}

_preflight_robot() {
  local n="$1" host="$2"
  shift 2

  # aed_interfaces 등 overlay 타입까지 preflight에서 확인할 수 있게 환경을 갱신한다.
  aedenv
  python3 "$AED_WS/tools/preflight.py" \
    --namespace "robot$n" \
    --host "$host" \
    "$@"
}

pf1() {
  _preflight_robot 1 "${ROBOT1_IP:-192.168.107.101}" "$@"
}

pf2() {
  _preflight_robot 2 "${ROBOT2_IP:-192.168.107.102}" "$@"
}

pfboth() {
  local rc=0

  echo "===== robot1 preflight ====="
  pf1 "$@" || rc=1
  echo
  echo "===== robot2 preflight ====="
  pf2 "$@" || rc=1

  return "$rc"
}

# 기존 pf 사용법도 유지한다: pf 1, pf 2, pf 1 --nav
pf() {
  local n="${1:-1}"
  if [ "$#" -gt 0 ]; then
    shift
  fi

  case "$n" in
    1) pf1 "$@" ;;
    2) pf2 "$@" ;;
    *)
      echo "사용: pf <1|2> [--localization|--nav|--detect]" >&2
      return 2
      ;;
  esac
}

fit() {
  local n=${1:-1}
  ROBOT_NS="/robot$n" python3 "$AED_WS/tools/check_fit.py" --ros-args \
    -r /tf:=/robot$n/tf -r /tf_static:=/robot$n/tf_static
}

undock() {
  local n=${1:-1}
  ros2 action send_goal /robot$n/undock irobot_create_msgs/action/Undock "{}"
}

dock() {
  local n=${1:-1}
  ros2 action send_goal /robot$n/dock irobot_create_msgs/action/Dock "{}"
}

slam() {
  local n=${1:-1}
  ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot$n
}

drive() {
  local n=${1:-1}
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r /cmd_vel:=/robot$n/cmd_vel
}

explore() {
  "$AED_WS/tools/explore.sh" "${1:-1}"
}

savemap() {
  local n=${1:-1} name=${2:-map}
  mkdir -p "$AED_WS/maps"
  ros2 run nav2_map_server map_saver_cli \
    -f "$AED_WS/maps/$name" -t "/robot$n/map" \
    --ros-args -p save_map_timeout:=15.0 || return 1
  ros2 service call /robot$n/slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '$AED_WS/maps/$name'}"
}

loc() {
  local n=${1:-1} map=${2:-$AED_WS/maps/map.yaml}
  ros2 launch turtlebot4_navigation localization.launch.py \
    namespace:=/robot$n map:="$map"
}

initpose() {
  local n=${1:-1}
  shift 2>/dev/null || true
  python3 "$AED_WS/tools/initpose.py" "$n" "$@"
}

nav() {
  local n=${1:-1}
  if ! python3 "$AED_WS/tools/preflight.py" \
      --localization --namespace "robot$n"; then
    echo "Nav2 시작 중단: loc $n -> initpose $n 순서로 복구하세요."
    return 1
  fi
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot$n \
    params_file:="$AED_WS/src/aed_bringup/config/nav2_aed.yaml"
}

rv() {
  local n=${1:-1}
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot$n
}

mapnav() {
  # LiDAR fallback은 실제 map navigation 기본 구성이다. 비교 시험에서만
  # `mapnav 1 false`처럼 두 번째 인자로 명시해 끈다.
  local n=${1:-1} fallback=${2:-true}
  aedenv
  ros2 launch turtlebot4_map_navigation map_navigation.launch.py \
    namespace:="robot$n" rviz:=true lidar_fallback:="$fallback"
}

vision() {
  local dev=${1:-0}
  ros2 run aed_vision webcam_publisher --ros-args -p device:="$dev"
}

manager() {
  ros2 run mission_manager mission_manager
}

# 중앙 노트북 통합 런타임. 기존 central 인자 순서를 그대로 유지한다.
central() {
  local dispatch=${1:-false}
  local target_time=${2:-30.0}
  local trigger_ratio=${3:-0.85}
  local dual_dispatch=${4:-true}
  aedenv
  ros2 launch aed_bringup server_runtime.launch.py \
    dispatch_enabled:="$dispatch" \
    target_arrival_time_sec:="$target_time" \
    dual_dispatch_trigger_ratio:="$trigger_ratio" \
    dual_dispatch_enabled:="$dual_dispatch"
}

executor() {
  local n=${1:-1}
  ros2 run robot_missions mission_executor --ros-args \
    -r __ns:=/robot$n -p robot_id:="robot$n"
}

siren() {
  local n=${1:-1}
  ros2 run emergency_alert siren --ros-args \
    -r __ns:=/robot$n -p audio_topic:=cmd_audio
}

recover() {
  "$AED_WS/tools/nav_recovery.sh" "${1:-1}"
}

survey() {
  local label=${1:?"사용: survey <라벨> [로봇번호]"} n=${2:-1}
  ROBOT_NS="/robot$n" python3 "$AED_WS/tools/survey_point.py" "$label" \
    --ros-args -r /tf:=/robot$n/tf -r /tf_static:=/robot$n/tf_static
}

bagrec() {
  local n=${1:-1}
  mkdir -p "$AED_WS/bags"
  ros2 bag record -o "$AED_WS/bags/$(date +%m%d_%H%M)" \
    --compression-mode file --compression-format zstd \
    /robot$n/oakd/rgb/image_raw/compressed /robot$n/scan /robot$n/odom \
    /robot$n/tf /robot$n/tf_static /robot$n/mission_assignment \
    /robot$n/mission_status /aed/emergency_event /aed/robot_state
}

alias detect='vision'
alias mstate='ros2 topic echo /aed/robot_state'
alias estate='ros2 topic echo /aed/emergency_event'
