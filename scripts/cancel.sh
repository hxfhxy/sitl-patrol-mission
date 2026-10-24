#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /cancel_patrol std_msgs/msg/Empty "{}"