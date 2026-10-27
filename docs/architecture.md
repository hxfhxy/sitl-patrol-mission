# 结构

四个包的边界：

| 包 | 放在这里 | 不放在这里 |
| --- | --- | --- |
| `sitl_bringup` | launch、参数、模型 | C++ |
| `sitl_adapters` | 导航、急停、雷达新鲜度 | 行为树、MQTT |
| `sitl_mission_bt` | 树节点和 `patrol.xml` | 任务阶段、网络 |
| `sitl_mission_node` | 阶段机、状态话题、MQTT 客户端，以及三个独立可执行文件 | 规划器 |

`cmd_vel_probe`、`scan_delay_node`、`odom_tf_gate` 和 `mission_node` 同包，但是分开的进程。它们不链进任务节点。

## 话题

```text
MQTT sitl/patrol/task  或  /patrol_task
        │
        ▼
   mission_node  ── tick ──►  patrol.xml
        │                      │
        │                      ├─ IsScanHealthy  订 /scan（到达时刻）
        │                      └─ NavigateTo     ──► /navigate_to_pose
        ▼
   /patrol_status  （以及可选的 sitl/patrol/status）

Nav2 /cmd_vel  ──►  EStop  ──►  /cmd_vel_safe  ──►  Gazebo diff-drive

/scan  ──►  scan_delay_node  ──►  /scan_delayed  ──►  costmap
/scan  ──►  AMCL

/odom  ──►  odom_tf_gate  ──►  TF odom → base_footprint
```

`/cmd_vel` 上有控制器经过平滑之后的输出，也有 `behavior_server`。急停若也往这条话题发 0，只是多个发布者之一。模型的 `command_topic` 因此改成 `cmd_vel_safe`，由 `EStop` 独占发布。锁存后丢掉输入，并按 50 ms 持续发 0，因为 Gazebo 会保持最后一条 `Twist`。

Gazebo 自己的 odom TF 关掉了。`odom_tf_gate` 以 50 ms 重发，stamp 用节点钟的 `now()`。`stall:=true` 时只停止发布 TF，不发零速度。零速度只走 `EStop`。

## 行为树

```text
ReactiveSequence
  IsScanHealthy          每拍重新检查
  Sequence               记住已经完成的点
    Retry(3) NavigateTo A
    Retry(3) NavigateTo B
    Retry(3) NavigateTo C
```

外层用 `ReactiveSequence`，是为了车已经在走时仍能看到雷达失败，并 halt 当前 Goal。三个点放在普通 `Sequence` 里，是为了 A 成功之后不会被外层下一拍重新发送。

`num_attempts="3"` 含第一次。重试只包住导航失败和超时。雷达失败在重试外面，整棵树 `FAILURE`。

`NavigateTo` 是 `StatefulActionNode`。超时发生在 `onRunning`：返回 `FAILURE` 之后状态已经不是 `RUNNING`，随后的 halt 不会再进 `onHalted`。`onHalted` 留给雷达失败和用户取消。下一次尝试的 `onStart` 会先 `release()` 急停，再发新 Goal。最后一次失败后面没有新的 `onStart`，锁存留着。

## 阶段

| 阶段 | 含义 |
| --- | --- |
| `WaitTask` | `auto_start` 为 false，还没有 start |
| `Arming` | 已接任务，正在等 Nav2 Active |
| `Running` | 10 Hz tick |
| `Fault` | 树失败或 60 s 内导航没就绪。节点还在 |
| `Canceled` | 用户取消。节点还在 |
| `Done` | 设定的圈数走完，进程随后退出。`patrol_loops` 为 0 时不会进入这里 |

等 Nav2 用墙钟。导航超时用节点钟。仿真时钟还没走的时候，墙钟仍然能把等待判超时；仿真暂停时，节点钟不会把还在执行的 Goal 判成超时。

构造函数里不调用 `wait_for_action_server`。那会占住 executor。从定时器里再 `spin_until_future_complete` 会自己等自己。

## 编排库

树用 `behaviortree_cpp_v3` 的 `tickRoot()`。Humble 的系统包是这个版本。任务树在单次 `NavigateToPose` 之上：三个点、雷达条件和重试。Nav2 内部还有自己的恢复树，两棵树不合并。
