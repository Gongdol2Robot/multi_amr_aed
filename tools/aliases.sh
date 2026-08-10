# Multi-AMR AED 프로젝트 공통 단축 명령
# 설치:
#   echo 'source ~/rokey_ws/multi_amr_aed/tools/aliases.sh' >> ~/.bashrc
#   source ~/.bashrc

export AED_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

aed() {
  cd "$AED_WS" || return 1
  source /opt/ros/humble/setup.bash || return 1

  if [ -f "$AED_WS/install/setup.bash" ]; then
    source "$AED_WS/install/setup.bash"
  else
    echo "AED 워크스페이스로 이동했습니다. install/setup.bash가 없어 ROS 2 기본 환경만 적용했습니다."
    echo "먼저 aedbuild를 실행하세요."
  fi
}

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
  if [ "$#" -gt 0 ]; then
    shift
  fi
  python3 "$AED_WS/tools/initpose.py" "$n" "$@"
}

nav() {
  local n=${1:-1}
  if ! python3 "$AED_WS/tools/preflight.py" \
      --localization --namespace "robot$n"; then
    echo "Nav2 시작 중단: loc $n -> initpose $n 순서로 복구하세요."
    return 1
  fi
  ros2 launch aed_bringup nav_with_fallback.launch.py \
    robot_name:="robot$n" \
    params_file:="$AED_WS/src/aed_bringup/config/nav2_aed.yaml"
}

rv() {
  local n=${1:-1}
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot$n
}

mapnav() {
  # robot_runtime이 map navigation과 LiDAR fallback을 함께 실행한다.
  local n=${1:-1}
  aedenv
  ros2 launch turtlebot4_map_navigation robot_runtime.launch.py \
    robot_name:="robot$n" rviz:=true
}

fallback() {
  local n=${1:-1}
  aedenv
  ros2 launch sensor_recovery lidar_fallback.launch.py \
    robot_name:="robot$n"
}

# 중앙 노트북 통합 런타임. 기존 central 인자 순서를 그대로 유지한다.
#
# 로봇마다 블루투스 스피커를 따로 붙였다면 sink 이름을 robot1,robot2 순서로
# AED_AUDIO_DEVICES 에 넣어 둔다. 스피커가 없는 팀원 환경에서 없는 장치를
# 강제하지 않도록 기본값은 비워 두며, 그 경우 OS 기본 출력으로 재생한다.
#   export AED_AUDIO_DEVICES="bluez_sink.<robot1_MAC>.a2dp_sink,bluez_sink.<robot2_MAC>.a2dp_sink"
#   pactl list short sinks | grep bluez   # sink 이름 확인
central() {
  local dispatch=${1:-true}
  local target_time=${2:-30.0}
  local trigger_ratio=${3:-0.85}
  local dual_dispatch=${4:-true}
  local vision_backend=${5:-${AED_VISION_BACKEND:-mannequin}}
  local audio_devices=${AED_AUDIO_DEVICES:-}
  local launch_args=(
    dispatch_enabled:="$dispatch"
    target_arrival_time_sec:="$target_time"
    dual_dispatch_trigger_ratio:="$trigger_ratio"
    dual_dispatch_enabled:="$dual_dispatch"
    vision_backend:="$vision_backend"
  )

  if [[ -n "$audio_devices" ]]; then
    launch_args+=(audio_devices:="$audio_devices")
  fi

  aedenv
  ros2 launch aed_bringup server_runtime.launch.py "${launch_args[@]}"
}

# 비전 backend별 통합 런타임 단축어. 첫 번째 선택 인자는 실제 출동 여부다.
centralperson() {
  central "${1:-true}" 30.0 0.85 true person_pose
}

centralmannequin() {
  central "${1:-true}" 30.0 0.85 true mannequin
}

# mannequin이 기본이므로 목각인형 모드는 기존 central을 그대로 사용한다.
# 실제 사람 Pose 모드만 짧게 구분한다.
centralp() {
  centralperson "${1:-true}"
}

centralm() {
  centralmannequin "${1:-true}"
}

# 고정 USB 카메라만 시험할 때 사용한다. 선택 인자는 카메라 번호(기본 1)다.
visionperson() {
  aedenv
  ros2 launch aed_vision camera_vision.launch.py \
    camera:="${1:-1}" backend:=person_pose
}

visionmannequin() {
  aedenv
  ros2 launch aed_vision camera_vision.launch.py \
    camera:="${1:-1}" backend:=mannequin
}

# 예전 detect/vision 사용법은 카메라 1 목각인형 모드로 호환한다.
vision() {
  visionmannequin "${1:-1}"
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
    /robot$n/lidar_state /robot$n/fallback_state \
    /robot$n/recovery_ready /robot$n/fallback_debug/path \
    /aed/mission_status /aed/emergency_event /aed/robot_state
}

alias detect='vision'
alias cperson='centralperson'
alias cmannequin='centralmannequin'
alias vperson='visionperson'
alias vmannequin='visionmannequin'
alias mstate='ros2 topic echo /aed/robot_state'
alias estate='ros2 topic echo /aed/emergency_event'
