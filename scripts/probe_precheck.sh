#!/usr/bin/env bash
# 仿真正在跑时，把探针开跑前必须记下的三个数打出来。
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# setup.bash 会读未设置的 AMENT_TRACE_SETUP_FILES，不能开 set -u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

if ! ros2 topic type /cmd_vel >/dev/null 2>&1; then
  echo "现在还读不到 /cmd_vel。"
  echo "先在另一个终端跑:  ${WS_DIR}/scripts/start.sh"
  echo "等到 Gazebo 里的车开始动，再回来执行本脚本。"
  exit 1
fi

OUT="${WS_DIR}/bags/run_precheck.txt"
mkdir -p "${WS_DIR}/bags"

{
  echo "date: $(date -Iseconds)"
  echo
  echo "===== 1) /cmd_vel 怎么发（QoS）====="
  ros2 topic info /cmd_vel --verbose
  echo
  echo "===== 2) 控制器标称周期 ====="
  ros2 param get /controller_server controller_frequency
  echo
  echo "===== 3) /clock 实际频率（约 5 秒）====="
  timeout 5 ros2 topic hz /clock || true
} | tee "${OUT}"

echo
echo "已写入 ${OUT}"
echo "看 Publisher count: 1 下面的 Reliability：一般是 RELIABLE，才能对上探针里的 reliable()。"
