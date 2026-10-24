#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models"
export GAZEBO_MODEL_DATABASE_URI=""

ros2 launch sitl_bringup sitl_patrol.launch.py "$@"