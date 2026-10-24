#!/usr/bin/env bash
# 仿真正在跑时检查 /scan 延迟中继。每条命令都有超时，不必 Ctrl+C。
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

if ! ros2 topic type /scan >/dev/null 2>&1; then
  echo "读不到 /scan。先保持另一个终端的 ./scripts/start.sh 开着，等车动了再跑本脚本。"
  exit 1
fi

OUT="${WS_DIR}/bags/run_scan_check.txt"
mkdir -p "${WS_DIR}/bags"

{
  echo "date: $(date -Iseconds)"
  echo

  echo "===== /scan 有没有 ====="
  echo "type: $(ros2 topic type /scan)"
  timeout 3 ros2 topic hz /scan || true
  echo

  echo "===== /scan_delayed 有没有 ====="
  if ros2 topic type /scan_delayed >/dev/null 2>&1; then
    echo "type: $(ros2 topic type /scan_delayed)"
    timeout 3 ros2 topic hz /scan_delayed || true
  else
    echo "现在没有 /scan_delayed。中继没发出去。"
  fi
  echo

  echo "===== /scan header（只看 stamp / frame_id）====="
  timeout 4 ros2 topic echo /scan --once 2>/dev/null | sed -n '1,12p' || true
  echo

  echo "===== /scan_delayed header ====="
  timeout 4 ros2 topic echo /scan_delayed --once 2>/dev/null | sed -n '1,12p' || true
  echo

  echo "===== costmap 参数 ====="
  ros2 param dump /local_costmap/local_costmap 2>/dev/null | grep -E 'topic:|expected_update|keep_time|transform_tolerance|observation' || echo "dump local_costmap 失败"
  echo "---"
  ros2 param dump /global_costmap/global_costmap 2>/dev/null | grep -E 'topic:|expected_update|keep_time|transform_tolerance|observation' || echo "dump global_costmap 失败"
} | tee "${OUT}"

echo
echo "已写入 ${OUT}"
echo "对照：两个 stamp 应相同；costmap 的 topic 应是 /scan_delayed。"
echo "hz 约 5Hz 是雷达频率，不是延迟量。要看的是两个 stamp 是否相同。"
