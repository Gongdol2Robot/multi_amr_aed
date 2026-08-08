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

pf() {
  local n=${1:-1}
  if [[ -n "${2:-}" ]]; then
    python3 "$AED_WS/tools/preflight.py" --namespace "robot$n" --host "$2"
  else
    python3 "$AED_WS/tools/preflight.py" --namespace "robot$n"
  fi
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

loc() {
  local n=${1:-1} map=${2:-$AED_WS/maps/map.yaml}
  ros2 launch turtlebot4_navigation localization.launch.py \
    namespace:=/robot$n map:="$map"
}

# loc + rv를 한 번에. 맵/RViz를 보려고 매번 터미널 두 개를 따로 켜지
# 않아도 되도록 묶은 것.
locview() {
  local n=${1:-1} map=${2:-$AED_WS/maps/map.yaml}
  ros2 launch aed_bringup localization_view.launch.py \
    namespace:=/robot$n map:="$map"
}

# turtlebot4_map_navigation의 map_navigation.launch.py 실행 + 화면 출력을
# 파일로도 저장. output='screen'인 노드 로그는 화면에만 나가고 파일에는
# 안 남아서(launch.log에는 프로세스 시작/종료 이벤트만 기록됨), 나중에
# 다시 확인하려면(Claude에게 보여주는 것 포함) 이렇게 tee로 남겨야 한다.
mapnav() {
  # 실제 운용 기본값: 맵/AMCL/Nav2와 LiDAR watchdog+fallback을 함께 실행한다.
  # 필요할 때만 `mapnav 1 false`처럼 두 번째 인자로 명시해 끌 수 있다.
  local n=${1:-1} fallback=${2:-true}
  aedenv
  mkdir -p "$AED_WS/logs"
  local file="$AED_WS/logs/mapnav_robot${n}_$(date +%Y%m%d_%H%M%S).log"
  echo "로그 저장: $file"
  ros2 launch turtlebot4_map_navigation map_navigation.launch.py \
    namespace:="robot$n" rviz:=true lidar_fallback:="$fallback" \
    2>&1 | tee "$file"
}

# 두 로봇이 지도는 공유하지만 Dock 위치가 달라 초기 위치는 각자 다르다.
# 좌표는 src/aed_bringup/config/dock_poses.yaml 에서 읽는다.
#   initpose 2            robot2 의 Dock 위치를 발행
#   initpose 1 --record   현재 위치를 robot1 항목에 기록
initpose() {
  local n=${1:-1}; shift 2>/dev/null || true
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

vision() {
  local dev=${1:-0}
  ros2 run aed_vision webcam_publisher --ros-args -p device:="$dev"
}

manager() {
  ros2 run mission_manager mission_manager
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
