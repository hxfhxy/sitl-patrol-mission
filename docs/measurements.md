# 已记录的运行

原始 rosbag 不在 git 里。下面的数字来自当时的日志和 `bags/` 中仍保留的 CSV、校验和。

## 2026-10-24 MQTT 巡检

broker 是仓库里的 `scripts/mqtt_broker.py`，不是 Eclipse Mosquitto。

成功，`bags/patrol_success`：

| 项 | 值 |
| --- | --- |
| `/patrol_status` | `accepted`，`running`，`goal_sent 1.900`，`succeeded` |
| 单点 | A 20.10 s，B 8.00 s，C 8.70 s（节点钟，`Goal 端到端`） |
| 里程计 | 起点 `(-2.00, -0.50)`；按 A、B、C 的顺序进入 0.25 m；终点 `(0.08, 1.70)`，离 C 0.21 m |
| `/cmd_vel_safe` | 736 条，其中 730 条非零 |
| `/clock` | 0 条。当时录包写成 reliable，Gazebo 发布的是 best effort。`scripts/bag_qos.yaml` 现已改成 best effort |

`xy_goal_tolerance` 是 0.25 m，终点在容差内。

故障，`bags/patrol_fault`：

| 项 | 值 |
| --- | --- |
| `/patrol_status` | `accepted`，`running`，`goal_sent 3.200`，`fault scan_disabled` |
| 位置 | 没到 A。锁存时约 `(0.32, -0.58)`，之后漂移 0.022 m |
| `/cmd_vel_safe` | 末尾 44 条为 0 |

校验和：

- `bags/patrol_success.sha256`
- `bags/patrol_fault.sha256`

它们对应本地的 `metadata.yaml` 和 `db3`。这两份数据库被 `.gitignore` 排除。换一次运行就要重新生成校验和。

成功结束后，launch 被脚本关掉，Nav2 日志里会出现 `bt_navigator` 心跳丢失。巡检状态在那之前已经是 `succeeded`。

## 2026-10-18 `/cmd_vel` 到达间隔

控制器 `controller_frequency` 为 20.0。`/clock` 约 10 Hz。探针是独立进程。CSV 在仓库里。

| 探针时钟 | 样本 | P50 | P99 | 文件 |
| --- | --- | --- | --- | --- |
| `use_sim_time:=true` | 236 | 0.0 ms | 100.0 ms | `bags/cmd_vel_arrival_baseline.csv` |
| `use_sim_time:=false` | 691 | 50.00 ms | 50.10 ms | `bags/cmd_vel_arrival_wall.csv` |

仿真钟那次只有 0 ms 和 100 ms 两档，各 118 个。20 Hz 的速度被 10 Hz 的 `/clock` 量化。墙钟那次探针不跟 `/clock`，间隔落在 50 ms 附近，对得上 20 Hz。两次时刻不能相减。

这次探针订的是 `/cmd_vel`。2026-10-18 的车还直接订 `/cmd_vel`。现在车订 `/cmd_vel_safe`，锁存期间两条话题会分开。

## 2026-10-18 注入

同一套世界、分开的运行：

- `delay_ms` 200 和 1000：A→B→C 都成功。导航期间没有 TF lookup 超时。stamp 保持原样。
- `odom_tf_gate stall:=true`，打在第一个 Goal 发出之后：第一点没有成功。`Transform data too old` 192 条，变换停在 33.312 s。车沿上一拍速度直行。

代价地图当时的运行时参数见 [interfaces.md](interfaces.md)。1 s 延迟仍走完，不能写成已经不避障。
