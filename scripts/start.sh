#!/usr/bin/env bash
# 图形界面下启动整套仿真。额外参数会传给 launch，例如 patrol_loops:=0。
# 不要 set -u：Humble 的 setup.bash 会读到未设置的变量。
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models"
# 置空，避免 Gazebo 启动时去拉在线模型库。
export GAZEBO_MODEL_DATABASE_URI=""

ros2 launch sitl_bringup sitl_patrol.launch.py "$@"