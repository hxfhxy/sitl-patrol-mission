# 参与修改

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select sitl_adapters sitl_mission_bt sitl_mission_node sitl_bringup
source install/setup.bash
colcon test --packages-select sitl_adapters sitl_mission_bt sitl_mission_node
colcon test-result --verbose
```

GTest 不启动 Gazebo。改了适配器或行为树之后，至少跑上面这组测试。改了话题接线或模型之后，再用 `scripts/start.sh` 或 `scripts/mqtt_patrol.sh` 看一次仿真。

## 保持这些边界

- 路点写在 `src/sitl_mission_bt/behavior_trees/patrol.xml`。
- 雷达延迟和 `odom_tf_gate` 的 `stall` 分成两次运行。
- `scan_delay_node` 不改 `LaserScan` 的 stamp、frame 和 payload。
- `stall` 只停止 `odom → base_footprint`。零速度由 `EStop` 发到 `/cmd_vel_safe`。
- 只有设定的圈数走完才 `rclcpp::shutdown()`。默认 `patrol_loops` 是 1。失败和取消留下节点，让零速度定时器继续跑。成功后的下一圈不要 `haltTree()`，避免把已经完成的点再锁存成零速度。
- `scripts/*.sh` 在 source ROS 环境时不要开 `set -u`。Humble 的 `setup.bash` 会读到未设置的变量。

## 许可

贡献按 [Apache-2.0](../LICENSE) 接收。模型或 Nav2 参数若再改，把改动写进 [NOTICE](../NOTICE)。
