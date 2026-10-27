#!/usr/bin/env bash
# 发一次空消息。mission_node 收到后停树并锁存零速度，进程不退出。
source /opt/ros/humble/setup.bash
ros2 topic pub --once /cancel_patrol std_msgs/msg/Empty "{}"