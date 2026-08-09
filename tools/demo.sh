#!/usr/bin/env bash
# 로봇 없이, 그러나 실제 ROS 로 관제 화면을 돌린다.
#
# `--mock` 과 다르다. 목업은 백엔드 안에서 도메인 객체를 바로 만들어 넣어
# ros/bridge.py 와 ros/converters.py 를 건너뛴다. 여기서는 rosbag 이 진짜
# 토픽에 진짜 메시지를 내고, 관제는 평소대로 구독한다. 메시지 타입과 QoS,
# 구독이 붙는지까지 전부 실물과 같은 경로로 돈다.
#
# 무엇이 뜨나
#   영상 4갈래   docs/videos 의 mp4 를 CompressedImage 로 낸다. bag 을 그대로
#                틀지 않는 이유는 두 가지다. 로봇 주행 bag 의 OAK-D 영상에는
#                검출 상자가 없고(그때는 로봇 검출 노드가 없었다), 웹캠 bag 은
#                11분 중 앞쪽 대부분이 인형이 서 있는 구간이다. 그 bag 들에서
#                쓸 만한 구간만 골라 만든 것이 이 mp4 다.
#   로봇 카드    bag 의 odom / battery_state / dock_status 에서 온 실제 값.
#                이건 bag 을 그대로 쓴다.
#   출동 이력    demo_publisher 가 내는 시나리오. 그 흐름을 담은 bag 이 없어
#                지어냈지만, 메시지 타입과 상태 전이 순서는 실물과 같다.
#
# 어느 쪽이든 관제는 실제 토픽에서 구독한다. 메시지 타입·QoS·구독이 붙는지가
# 모두 실물과 같은 경로로 검증된다.
#
# 쓰기:
#   tools/demo.sh          띄운다
#   tools/demo.sh stop     내린다
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT}/var/demo"
ROBOT_BAG="${ROOT}/bags/robot1_map_0806_1846"
VIDEOS="${ROOT}/docs/videos"
DB="${ROOT}/var/aed_hmi_demo.sqlite3"

stop_all() {
  local stopped=0
  if [ -d "${RUN_DIR}" ]; then
    for pidfile in "${RUN_DIR}"/*.pid; do
      [ -e "${pidfile}" ] || continue
      local pid; pid="$(cat "${pidfile}")"
      if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "${pid}" 2>/dev/null
        stopped=$((stopped + 1))
      fi
      rm -f "${pidfile}"
    done
  fi

  # pid 파일만 믿으면 안 된다. 지난번에 다른 방식으로 띄웠거나 파일이
  # 지워졌으면 옛 프로세스가 살아남아 포트를 쥐고, 새로 띄운 것이 조용히
  # 죽는다. 그러면 코드를 고쳐도 옛 코드가 계속 도는 것처럼 보인다.
  #
  # 패턴의 대괄호는 pkill 자신의 명령줄에 안 걸리게 하는 수법이다.
  # 정규식 [b]ackend 는 "backend" 에 맞지만, 이 줄 자체에는 "[b]ackend" 라고
  # 적혀 있어 자기 자신은 안 맞는다.
  pkill -f "[b]ackend.main" 2>/dev/null && stopped=$((stopped + 1))
  pkill -f "[d]emo_publisher.py" 2>/dev/null && stopped=$((stopped + 1))
  pkill -f "[b]ag play ${ROOT}/bags" 2>/dev/null && stopped=$((stopped + 1))
  pkill -f "[v]ideo_publisher.py" 2>/dev/null && stopped=$((stopped + 1))

  # 포트가 비었는지 확인한다. 안 비었으면 새 백엔드가 못 뜬다.
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    ss -ltn 2>/dev/null | grep -q ":8000 " || break
    sleep 0.5
  done
  echo "내림: ${stopped}개"
}

if [ "${1:-}" = "stop" ]; then
  stop_all
  exit 0
fi

if [ ! -d "${ROBOT_BAG}" ]; then
  echo "실패: ${ROBOT_BAG} 가 없습니다" >&2
  exit 1
fi
for name in camera_open_demo camera_alley_demo robot_approach_yolo; do
  if [ ! -f "${VIDEOS}/${name}.mp4" ]; then
    echo "실패: ${VIDEOS}/${name}.mp4 가 없습니다" >&2
    exit 1
  fi
done

stop_all
mkdir -p "${RUN_DIR}"

# ROS 의 setup 스크립트는 안 정해진 변수를 읽는다. set -u 를 켠 채로
# source 하면 거기서 죽는다. 이 구간만 끈다.
set +u
source /opt/ros/humble/setup.bash
[ -f /etc/turtlebot4_discovery/setup.bash ] && \
  source /etc/turtlebot4_discovery/setup.bash
[ -f "${ROOT}/install/setup.bash" ] && source "${ROOT}/install/setup.bash"
set -u
# 이걸 안 켜면 discovery server 를 거치는 토픽이 하나도 안 보인다.
export ROS_SUPER_CLIENT=True

# setsid 를 쓰지 않는다. setsid 는 fork 하므로 $! 가 실제 프로세스가 아니라
# 곧 사라질 껍데기의 pid 가 되고, 그러면 나중에 아무것도 못 죽인다.
# nohup + disown 만으로도 이 셸이 끝난 뒤까지 살아남는다.
launch() {   # launch <이름> <명령...>
  local name="$1"; shift
  nohup "$@" > "${RUN_DIR}/${name}.log" 2>&1 < /dev/null &
  local pid=$!
  disown
  echo "${pid}" > "${RUN_DIR}/${name}.pid"
  echo "  ${name} (pid ${pid})"
}

launch_in() {   # launch_in <디렉터리> <이름> <명령...>
  local dir="$1"; shift
  local name="$1"; shift
  local here; here="$(pwd)"
  cd "${dir}" || return 1
  launch "${name}" "$@"
  cd "${here}" || return 1
}

echo "띄우는 중"

# 영상 네 갈래. 로봇 둘은 같은 파일을 시작 위치만 벌려 쓴다. 한 대가
# 이동 중일 때 다른 한 대는 이미 발견한 상태라 같은 그림 둘로 안 보인다.
launch video python3 "${ROOT}/tools/video_publisher.py" \
  --stream "camera_open=${VIDEOS}/camera_open_demo.mp4" \
  --stream "camera_alley=${VIDEOS}/camera_alley_demo.mp4" \
  --stream "robot1=${VIDEOS}/robot_approach_yolo.mp4" \
  --stream "robot2=${VIDEOS}/robot_approach_yolo.mp4:0.45"

# 로봇 주행 bag 에서 상태만 뽑는다. 영상은 위에서 내므로 여기서는 뺀다.
# 같은 토픽에 둘이 내면 화면이 두 그림을 번갈아 받는다.
BAG_STATE_TOPICS="/robot1/odom /robot1/battery_state /robot1/dock_status"
launch robot1 ros2 bag play "${ROBOT_BAG}" --loop --topics ${BAG_STATE_TOPICS}

# 로봇이 한 대분만 녹화돼 있어, 2호기도 같은 bag 을 이름만 바꿔 튼다.
launch robot2 ros2 bag play "${ROBOT_BAG}" --loop --topics ${BAG_STATE_TOPICS} \
  --remap \
  /robot1/odom:=/robot2/odom \
  /robot1/battery_state:=/robot2/battery_state \
  /robot1/dock_status:=/robot2/dock_status

# bag 이 첫 값을 낼 때까지 기다렸다가 시나리오를 시작한다.
sleep 3
launch publisher python3 "${ROOT}/tools/demo_publisher.py"

# backend 는 패키지 안에서 띄워야 상대 import 가 풀린다.
launch_in "${ROOT}/src/aed_hmi" backend python3 -m backend.main --db "${DB}" --port 8000

# 백엔드가 실제로 떴는지 확인한다. 포트를 못 잡으면 조용히 죽는데, 그러면
# 화면은 옛 데이터를 보여주고 있어 알아채기 어렵다.
for _ in $(seq 1 30); do
  curl -sf http://127.0.0.1:8000/api/health > /dev/null 2>&1 && break
  sleep 0.5
done
if curl -sf http://127.0.0.1:8000/api/health > /dev/null 2>&1; then
  echo
  echo "관제 서버 떴음 — $(curl -s http://127.0.0.1:8000/api/health)"
else
  echo
  echo "실패: 관제 서버가 안 떴습니다. ${RUN_DIR}/backend.log 를 보세요." >&2
fi

echo
echo "로그: ${RUN_DIR}/*.log"
echo "화면: cd src/aed_hmi/frontend && npm run dev   →  http://localhost:5173"
echo "내릴 때: tools/demo.sh stop"
