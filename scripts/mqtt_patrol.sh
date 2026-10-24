#!/usr/bin/env bash
# MQTT 下发一次巡检，并录 bag。
#   bash scripts/mqtt_patrol.sh success
#   bash scripts/mqtt_patrol.sh fault
#   bash scripts/mqtt_patrol.sh both
set -e

MODE="${1:-both}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
QOS_FILE="${WS_DIR}/scripts/bag_qos.yaml"
BROKER_PID=""
LAUNCH_PID=""
BAG_PID=""
ECHO_PID=""
FFMPEG_PID=""

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models"
export GAZEBO_MODEL_DATABASE_URI=""

cleanup() {
  if [[ -n "${ECHO_PID}" ]]; then
    kill "${ECHO_PID}" 2>/dev/null || true
  fi
  if [[ -n "${BAG_PID}" ]]; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${FFMPEG_PID}" ]]; then
    kill -INT "${FFMPEG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT -"${LAUNCH_PID}" 2>/dev/null || kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  fi
  sleep 2
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -KILL -"${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${BROKER_PID}" ]]; then
    kill "${BROKER_PID}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
}
trap cleanup EXIT

port_open() {
  python3 - "${1}" <<'PY'
import socket, sys
port = int(sys.argv[1])
s = socket.socket()
s.settimeout(0.3)
try:
    s.connect(("127.0.0.1", port))
except OSError:
    sys.exit(1)
finally:
    s.close()
PY
}

start_broker() {
  if port_open 1883; then
    echo "1883 已有 broker"
    return
  fi
  if command -v mosquitto >/dev/null 2>&1; then
    mosquitto -p 1883 &
    BROKER_PID=$!
  else
    python3 "${WS_DIR}/scripts/mqtt_broker.py" --port 1883 &
    BROKER_PID=$!
  fi
  for _ in $(seq 1 25); do
    if port_open 1883; then
      return
    fi
    sleep 0.2
  done
  echo "MQTT broker 没有起来" >&2
  exit 1
}

publish_start() {
  if command -v mosquitto_pub >/dev/null 2>&1 && port_open 1883; then
    mosquitto_pub -h 127.0.0.1 -p 1883 -t sitl/patrol/task -m start
  else
    python3 "${WS_DIR}/scripts/mqtt_broker.py" --publish sitl/patrol/task start
  fi
}

wait_status() {
  local want="$1"
  local limit="$2"
  local deadline=$((SECONDS + limit))
  while (( SECONDS < deadline )); do
    if [[ -f "${STATUS_FILE}" ]] && grep -q "${want}" "${STATUS_FILE}"; then
      return 0
    fi
    sleep 1
  done
  echo "等待状态超时: ${want}" >&2
  echo "---- patrol_status ----" >&2
  cat "${STATUS_FILE}" >&2 || true
  return 1
}

record_one() {
  local kind="$1"
  local bag_dir="${WS_DIR}/bags/patrol_${kind}"
  local log_file="${WS_DIR}/bags/patrol_${kind}.log"
  STATUS_FILE="${WS_DIR}/bags/patrol_${kind}.status"
  rm -rf "${bag_dir}" "${STATUS_FILE}"
  : > "${STATUS_FILE}"

  echo "启动仿真 mode=${kind}"
  setsid ros2 launch sitl_bringup sitl_patrol.launch.py \
    auto_start:=false mqtt_enable:=true \
    > "${log_file}" 2>&1 &
  LAUNCH_PID=$!

  local ready=0
  for _ in $(seq 1 60); do
    if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; then
      ready=1
      break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      echo "launch 提前退出，看 ${log_file}" >&2
      return 1
    fi
    sleep 2
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "120 s 内 bt_navigator 没有 Active" >&2
    return 1
  fi

  ros2 bag record --use-sim-time \
    --qos-profile-overrides-path "${QOS_FILE}" \
    -o "${bag_dir}" \
    /patrol_status /cmd_vel /cmd_vel_safe /odom /clock /scan \
    > "${WS_DIR}/bags/patrol_${kind}.bag.log" 2>&1 &
  BAG_PID=$!

  PYTHONUNBUFFERED=1 ros2 topic echo /patrol_status --field data \
    --qos-reliability reliable --qos-durability transient_local \
    > "${STATUS_FILE}" 2>/dev/null &
  ECHO_PID=$!
  sleep 1

  if [[ "${kind}" == "success" && -n "${DISPLAY:-}" ]] && command -v ffmpeg >/dev/null 2>&1; then
    local size
    size="$(xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2; exit}')"
    if [[ -z "${size}" ]]; then
      size="1920x1080"
    fi
    ffmpeg -y -loglevel error -f x11grab -video_size "${size}" -framerate 10 -i "${DISPLAY}" \
      -vf scale=1280:-2 -c:v libx264 -preset veryfast -crf 30 -pix_fmt yuv420p \
      "${WS_DIR}/docs/demo.mp4" \
      > "${WS_DIR}/bags/demo_ffmpeg.log" 2>&1 &
    FFMPEG_PID=$!
  fi

  echo "MQTT start"
  publish_start

  if [[ "${kind}" == "fault" ]]; then
    wait_status "running" 90
    sleep 8
    ros2 param set /mission_node scan_health_enabled false
    wait_status "fault scan_disabled" 30
  else
    wait_status "succeeded" 240
  fi

  sleep 2
  kill -INT "${BAG_PID}" 2>/dev/null || true
  wait "${BAG_PID}" 2>/dev/null || true
  BAG_PID=""
  if [[ -n "${FFMPEG_PID}" ]]; then
    kill -INT "${FFMPEG_PID}" 2>/dev/null || true
    wait "${FFMPEG_PID}" 2>/dev/null || true
    FFMPEG_PID=""
  fi
  kill "${ECHO_PID}" 2>/dev/null || true
  ECHO_PID=""
  kill -INT -"${LAUNCH_PID}" 2>/dev/null || true
  sleep 3
  kill -KILL -"${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
  LAUNCH_PID=""
  for _ in $(seq 1 15); do
    if ! pgrep -x gzserver >/dev/null && ! pgrep -x gzclient >/dev/null; then
      break
    fi
    sleep 1
  done
  if pgrep -x gzserver >/dev/null; then
    pkill -x gzserver || true
    pkill -x gzclient || true
  fi

  if [[ ! -f "${bag_dir}/metadata.yaml" ]]; then
    echo "bag 没有写完: ${bag_dir}" >&2
    return 1
  fi
  (
    cd "${WS_DIR}/bags"
    sha256sum "patrol_${kind}/metadata.yaml" "patrol_${kind}"/*.db3 > "patrol_${kind}.sha256"
  )
  echo "完成 ${bag_dir}"
  ros2 bag info "${bag_dir}"
}

start_broker
case "${MODE}" in
  success) record_one success ;;
  fault) record_one fault ;;
  both)
    record_one success
    record_one fault
    ;;
  *)
    echo "用法: $0 success|fault|both" >&2
    exit 2
    ;;
esac
