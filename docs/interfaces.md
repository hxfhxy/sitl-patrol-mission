# 接口

路点只以 `src/sitl_mission_bt/behavior_trees/patrol.xml` 为准。`auto_start` 默认 true，`scripts/start.sh` 不用另发任务。

## 状态字

`/patrol_status` 是 `std_msgs/String`，reliable、transient_local、深度 1。后连上的订阅者只能看到最新一条。MQTT 打开时，同一段文本发到 `sitl/patrol/status`。

| 文本 | 含义 |
| --- | --- |
| `accepted` | 收到 start，还在等 Nav2 |
| `running` | 开始 tick |
| `goal_sent <秒>` | 仅外部任务。任务到达到 Nav2 接受第一个 Goal，三位小数 |
| `succeeded` | 三点完成，随后进程退出 |
| `canceled` | 用户取消，进程仍在 |
| `fault <reason>` | 树失败，或 60 s 墙钟内 Nav2 没 Active |

`reason` 来自黑板 `fault_reason`：

| reason | 来源 |
| --- | --- |
| `scan_disabled` | `scan_health_enabled` 为 false |
| `scan_stale` | 已经收过帧，超过 `max_age_sec` 没有新回调 |
| `scan_unavailable` | 还没有可用帧 |
| `nav_timeout` | 该点超过 `timeout_sec` |
| `nav_rejected` | 服务器拒绝 Goal |
| `nav_aborted` | 导航中止 |
| `nav_canceled` | 结果是取消，且不是本节点主动停的那次 |
| `nav_inactive` | 等 Nav2 Active 超过 60 s 墙钟 |
| `unknown` | 树失败时黑板是空的 |

雷达原因先写上。`onHalted` 只有黑板还空时才写 `canceled`，避免把关雷达记成用户取消。用户取消由 `mission_node` 直接发 `canceled`，不读黑板。

深度 1 加上成功后退出：录 `succeeded` 的订阅者要在进程退出前已经连上。

## 任务入口

`/patrol_task` 和 MQTT 载荷共用一份解析：去掉首尾空白、转成小写，只认 `start` 和 `cancel`。

`/cancel_patrol`（`std_msgs/Empty`）与文本 `cancel` 相同：停 tick，`haltTree()`，锁存零速度，节点不退出。

Nav2 还没 Active 时不发 Goal。指标一只打在外部任务上，时刻是 Nav2 接受当前这次 Goal，不是 `async_send_goal` 返回的那一下。被拒绝的 Goal 不记。

## 取消还在飞的 Goal

`async_send_goal` 立刻返回，句柄稍后才到。`GoalSlot` 把这段记成「等句柄」。取消发生在句柄到达前时，句柄一到就取消它。新的一次发送会清掉这个标记，避免上一次的取消打到重试的新 Goal 上。

每次发送有序号。过期的句柄和过期的结果丢掉。过期句柄若非空，还会取消那个 Goal。

## 速度

`EStop` 订阅 `/cmd_vel`，发布 `/cmd_vel_safe`。差速模型只订后者。`halt()` 之后不再转发，并以 50 ms 定时器发 0，直到下一次 `NavigateTo::onStart` 调用 `release()`。FAULT 和用户取消都不会自动解开。

`/clock` 约 10 Hz，这个 50 ms 定时器在仿真时间上会落到大约 100 ms 一拍。

`cmd_vel_probe` 订的是 `/cmd_vel`。锁存期间它看到的仍是 Nav2 的输出，车执行的是 `/cmd_vel_safe` 上的 0。

`geometry_msgs/Twist` 没有 `header`。探针量的是回调里 `now()` 的相邻间隔，不是消息自带时间戳，也不是控制器周期本身。

## 时钟

打点用节点钟 `now()`，跟随 `use_sim_time`。

- 等 Nav2 的 200 ms 轮询和 60 s 上限是墙钟。
- 行为树 tick 和导航超时是节点钟。
- 墙钟和仿真钟的时刻不能相减。两套配置各自算间隔，再比较分布。
- `use_sim_time:=true` 时，读数被 `/clock` 的发布间隔量化。控制器 20 Hz、`/clock` 10 Hz 时，间隔直方图会出现 0 和约 100 ms 两档。那是时钟发布频率。开跑前用 `ros2 topic hz /clock` 量，不靠记忆里的默认值。
- 单独跑探针时要带 `-p use_sim_time:=true`，和主节点同一套钟。

Goal 的 `header.stamp` 用节点钟。

## 雷达

`SensorHealth` 订参数 `scan_health_topic`，默认 `/scan`，不订 `/scan_delayed`。记下的是回调到达时的 `now()`，不看 `LaserScan.stamp`。延迟实验会让数据晚到，stamp 仍是采集时刻；用 stamp 判断断连会把延迟看成雷达死亡。

`mark_fresh()` 只在还没有任何到达记录时记一笔启动宽限。已经收过帧，或这次宽限已经记过，就不再改时刻。第一帧到来之前，第一次任务会有大约 5 s 看起来是健康的。

树是外层 `ReactiveSequence`。雷达不健康会 halt 正在跑的 `NavigateTo`，整棵树失败，不进入该点的 3 次重试。

## 两套注入，分开跑

延迟实验：`scan_delay_node` 把 `/scan` 原样晚一点发到 `/scan_delayed`。不改 payload、stamp、`frame_id`。代价地图订 `/scan_delayed`，AMCL 仍订 `/scan`。中继和主节点使用同一个 `use_sim_time`。`/scan` 通常是 best effort，中继用 `SensorDataQoS`。

对照实验：`odom_tf_gate` 的 `stall` 停止发布 `odom → base_footprint`。Nav2 查不到新 TF 就不发新速度，也不会自己发 0，车沿上一拍速度继续走。零速度不写进这个门控，否则对照没了。

`nav2_params.yaml` 把代价地图的激光话题写成 `/scan_delayed`。它没有覆盖 `expected_update_rate`、`observation_persistence` 和 costmap 的 `transform_tolerance`。2026-10-18 的运行时 dump 是：

| 参数 | 当时的值 |
| --- | --- |
| `topic` | `/scan_delayed` |
| `expected_update_rate` | `0.0` |
| `observation_persistence` | `0.0` |
| `transform_tolerance` | `0.3` |

`expected_update_rate` 为 0 时，观测缓冲不因更新变慢而判过期。1 s 延迟大于 0.3 s 的容差，观测仍被收下：代价地图用扫描自带的旧 stamp 去查 TF 历史，不是用 `now() - stamp` 把这帧丢掉。因此延迟跑次走完 A→B→C，不能写成已经不避障。要下那个结论，需要当时的代价地图是空的，或有一次撞障记录。

| 注入 | 结果 |
| --- | --- |
| delay 200 ms、1000 ms | A→B→C 成功，导航期间没有 `Transform data too old` |
| `stall true`（Goal 已发出） | 第一点未到达；`Transform data too old` 192 条，TF 停在 33.312 s；车继续直行 |

两次不要混在同一次运行里。

## 三个数，分开报

| 指标 | 定义 | 记录位置 |
| --- | --- | --- |
| 任务响应 | 外部 `start` 到达 → Nav2 接受第一个 Goal | 日志 `指标一`，以及 `goal_sent` |
| 单点耗时 | 该点 `onStart` → 成功结果，节点钟 | 日志 `Goal 端到端` |
| `/cmd_vel` 到达间隔 | 独立探针相邻回调的 `now()` 之差 | P50 / P99 / max，对照 `controller_frequency` |

探针是独立进程。和 `mission_node` 共用一个 executor 时，量到的是那个进程自己的排队。开跑前用 `ros2 topic info /cmd_vel --verbose` 核对 QoS，订阅不匹配会静默收不到。
