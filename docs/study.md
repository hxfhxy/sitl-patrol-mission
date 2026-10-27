# 代码讲解

这份笔记顺着一次巡检的调用顺序，讲语法和逻辑。话题接线、状态字和已经记下的数字分别在 [architecture.md](architecture.md)、[interfaces.md](interfaces.md)、[measurements.md](measurements.md)。

建议顺着这几个文件读：`mission_node.cpp`、`patrol.xml`、`bt_nodes.cpp`、`nav_adapter.cpp`、`goal_slot.hpp`、`estop.cpp`、`sensor_health.cpp`。

一次巡检就是 `mission_node` 用单线程 `spin` 把行为树以 10 Hz 往下 tick。树只做两件事：每拍看 `/scan` 还在不在，然后按 `patrol.xml` 把 A、B、C 发给 Nav2。导航、停车、雷达新鲜度都在适配器里，树节点拿的是裸指针。

## 语法上先认这几样

节点都是 `class X : public rclcpp::Node`。构造函数里 `declare_parameter` 声明参数，后面才能 `get_parameter`。订阅和定时器的回调写成捕获 `this` 的 lambda，ROS 在 `rclcpp::spin` 里调用它们。

这个进程只有一个线程在跑这些回调。`GoalSlot`、`latched_`、阶段 `phase_` 因此不加锁。MQTT 是唯一的另一条线程，它只把文本推进队列，真正的 `start` / `cancel` 仍回到 `poll()` 里处理。

行为树节点不拥有适配器。`MissionNode` 用 `unique_ptr` 持有 `NavAdapter`、`EStop`、`SensorHealth`，树在它们后面声明。C++ 成员按声明的反序析构，所以树先销毁，适配器还在。析构函数里先停定时器，再 `mqtt_.reset()`，让 MQTT 线程先退出。

测试用的是同一套虚函数。`FakeNav` 继承 `NavAdapter` 并 `override` `send_goal`，所以行为树测试不用真的 Action 服务器。

## 阶段机

`Phase` 是 `enum class`：`WaitTask`、`Arming`、`Running`、`Fault`、`Canceled`、`Done`。

构造结束时，`auto_start` 为 true 就直接 `request_start(false)`。否则停在 `WaitTask`，等 `/patrol_task` 或 MQTT 送来 `start`。

`request_start` 只在 `WaitTask`、`Fault`、`Canceled` 接受。它停掉上一轮的 tick 定时器，`haltTree()`，把黑板 `fault_reason` 清掉，圈数归零，阶段改成 `Arming`，墙钟 60 秒为限。外部任务还会记下 `task_arrival_`，后面用来打指标一。

```cpp
void MissionNode::request_start(bool external)
{
  if (phase_ != Phase::WaitTask && phase_ != Phase::Fault && phase_ != Phase::Canceled) {
    RCLCPP_WARN(get_logger(), "已有任务在执行或正在等待 Nav2，忽略新的 start。");
    return;
  }
  if (bt_timer_) {
    bt_timer_->cancel();
  }
  tree_.haltTree();
  if (const auto blackboard = tree_.rootBlackboard()) {
    blackboard->set("fault_reason", std::string());
  }
  lifecycle_future_.reset();
  external_task_ = external;
  goal_metric_logged_ = false;
  laps_finished_ = 0;
  if (external) {
    task_arrival_ = now();
  }
  phase_ = Phase::Arming;
  arm_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  publish_status("accepted");
}
```

`poll()` 是 200 ms 的墙钟定时器：先把队列里的命令取走，再问 `/bt_navigator/get_state`。服务还没好，或者 future 还没就绪，就返回，下一拍再问。这里不能 `spin_until_future_complete`，因为当前已经在 executor 的定时器回调里，再 spin 会自己等自己。

Nav2 的状态 id 是 `PRIMARY_STATE_ACTIVE`，并且 `/navigate_to_pose` 的 `wait_for_server(0s)` 也成功，才 `begin_ticking()`。那一拍调用 `mark_fresh()`，发 `running`，然后用节点钟建 100 ms 定时器。`use_sim_time:=true` 时，这个 100 ms 跟的是 `/clock`，不是笔记本的墙钟。

`tick_tree()` 只在 `Running` 时做事。树返回 `SUCCESS` 就加一圈：

```cpp
void MissionNode::tick_tree()
{
  if (phase_ != Phase::Running) {
    return;
  }
  const BT::NodeStatus status = tree_.tickRoot();
  if (status == BT::NodeStatus::SUCCESS) {
    laps_finished_ += 1;
    if (const auto blackboard = tree_.rootBlackboard()) {
      blackboard->set("fault_reason", std::string());
    }
    if (!patrol_laps_complete(patrol_loops_, laps_finished_)) {
      publish_status("lap " + std::to_string(laps_finished_));
      return;
    }
    phase_ = Phase::Done;
    bt_timer_->cancel();
    publish_status("succeeded");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::shutdown();
  } else if (status == BT::NodeStatus::FAILURE) {
    phase_ = Phase::Fault;
    bt_timer_->cancel();
    estop_->halt();
    // 原因来自黑板 fault_reason，空的时候写成 unknown
    publish_status("fault " + reason);
  }
}
```

成功后的下一拍不要 `haltTree()`。点已经成功，再 halt 可能进 `onHalted`，把零速度锁存到下一圈。`tickRoot()` 在 `SUCCESS` 之后已经把根设回 `IDLE`，下一拍会从 A 再走。

`patrol_laps_complete` 的规则是：`patrol_loops > 0` 且已经走完的圈数达到这个数，才算结束。默认 `1`，走完就 `shutdown()`。参数 `0` 时条件永远不成立，定时器继续 tick。负数在构造里改成 `1`。

```cpp
inline bool patrol_laps_complete(int patrol_loops, int laps_finished)
{
  return patrol_loops > 0 && laps_finished >= patrol_loops;
}
```

失败时阶段变成 `Fault`，`estop_->halt()`，状态是 `fault` 加上黑板里的原因，进程留着。取消时，若还在 `Arming` 就直接 `Canceled`；若在 `Running` 就停掉 tick 并 `haltTree()`。两种情况都不退出。`WaitTask`、`Fault`、`Canceled` 下再来 `start`，会重新装订。

`/patrol_status` 的 QoS 是 reliable、transient_local、深度 1。后连上的订阅者只能看到最新一条。`goal_sent 1.900` 这种只在外部 `start` 的第一个被 Nav2 接受的 Goal 上打，`auto_start` 没有这个时刻。`note_goal_sent()` 用 `goal_metric_logged_` 保证只打一次，下一圈不会再打。

## `patrol.xml` 的语法

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence name="patrol_with_scan">
      <IsScanHealthy name="scan_health" max_age_sec="1.0"/>
      <Sequence name="patrol_sequence">
        <RetryUntilSuccessful num_attempts="3">
          <NavigateTo name="wp_A" x="1.5" y="0.0" yaw="0.0" timeout_sec="60"/>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <NavigateTo name="wp_B" x="1.5" y="1.5" yaw="1.57" timeout_sec="60"/>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <NavigateTo name="wp_C" x="0.0" y="1.5" yaw="3.14" timeout_sec="60"/>
        </RetryUntilSuccessful>
      </Sequence>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

`main_tree_to_execute` 指定要跑哪一棵 `BehaviorTree`。标签名必须和 `registerBuilder` 里的字符串一致：`NavigateTo`、`IsScanHealthy`。`RetryUntilSuccessful`、`Sequence`、`ReactiveSequence` 是 BehaviorTree.CPP v3 自带的。属性 `x`、`y`、`yaw`、`timeout_sec`、`max_age_sec` 进 `InputPort`，节点里用 `getInput("x", x)` 读出来。读不到就返回 `FAILURE`。

注册时把适配器指针捕获进 lambda。XML 每出现一个 `<NavigateTo>`，工厂就调一次这个 lambda，新建一个节点：

```cpp
factory.registerBuilder<NavigateTo>(
  "NavigateTo",
  [nav_adapter, estop](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<NavigateTo>(name, config, nav_adapter, estop);
  });
```

`tickRoot()` 从根往下调用每个节点的 `tick()`。子节点返回 `RUNNING` 时，这一拍停在那里，100 ms 后再 tick。返回 `SUCCESS` 或 `FAILURE` 后，`tickRoot` 把根设回 `IDLE`，子节点自己的成员还在，所以下一圈不用重建树。

外层是 `ReactiveSequence`：每一拍都从第一个孩子重新 tick。第一个孩子是 `IsScanHealthy`。它失败时，这个控制节点会 halt 后面正在跑的孩子，整棵树 `FAILURE`。所以雷达失败不进入那个点的 3 次重试。

三个点包在普通 `Sequence` 里。`Sequence` 记住当前走到第几个孩子：A 已经 `SUCCESS` 之后，下一拍不会再发 A，只会 tick 还在 `RUNNING` 的 B。如果三个点也放进 `ReactiveSequence`，每拍都会把 A 再发一次。

`RetryUntilSuccessful num_attempts="3"` 包住一个点。孩子 `SUCCESS` 就停，`FAILURE` 再来，一共 3 次，含第一次。

## `NavigateTo` 的三个函数

它继承 `BT::StatefulActionNode`。状态从 `IDLE` 进来时框架调用 `onStart()`；已经是 `RUNNING` 时调用 `onRunning()`；别人 halt 它、且当时状态是 `RUNNING` 时才调用 `onHalted()`。从 `onRunning` 返回 `FAILURE` 之后状态已经不是 `RUNNING`，紧接着的 Retry halt 不会再进 `onHalted()`。

```cpp
BT::NodeStatus NavigateTo::onStart()
{
  const uint32_t gen = ++goal_gen_;
  goal_done_.store(false);
  start_time_ = nav_adapter_->now();
  if (estop_) {
    estop_->release();
  }
  nav_adapter_->send_goal(
    x, y, yaw,
    [this, gen](sitl_adapters::NavResult res) {
      if (gen != goal_gen_.load()) {
        return;
      }
      last_result_.store(res);
      goal_done_.store(true);
    });
  return BT::NodeStatus::RUNNING;
}
```

`goal_gen_` 是 `std::atomic<uint32_t>`。Action 的结果回调和 tick 都会碰它。回调里如果 `gen` 已经不是现在这一代，说明这是上一轮取消或超时留下的结果，丢掉。

`onRunning` 里 Goal 还没结束时，用节点钟和 `start_time_` 比。超过 `timeout_sec` 就先 `++goal_gen_`，让在飞的回调失效，再 `cancel_current_goal()`、`estop_->halt()`，黑板写成 `nav_timeout`，返回 `FAILURE`。超时这一拍代数加一次，下一次重试的 `onStart` 再加一次，所以日志里会看到 gen 从 1 跳到 3。

结果到了以后：`SUCCEEDED` 返回成功。`REJECTED` 写成 `nav_rejected`，`CANCELED` 写成 `nav_canceled`，其余写成 `nav_aborted`。原因用 `set_fault_reason(..., overwrite=true)` 写进黑板。

`onHalted` 也是先 `++goal_gen_`，再取消 Goal、急停。它写 `canceled` 时 `overwrite` 是 false：黑板上已经有 `scan_disabled` 这类原因时，不覆盖。

```cpp
void set_fault_reason(const BT::TreeNode & node, const std::string & reason, bool overwrite)
{
  const auto blackboard = node.config().blackboard;
  if (!blackboard) {
    return;
  }
  std::string existing;
  if (!overwrite && blackboard->get("fault_reason", existing) && !existing.empty()) {
    return;
  }
  blackboard->set("fault_reason", reason);
}
```

用户取消不读这块黑板，`mission_node` 直接发 `canceled`。

`IsScanHealthy::tick()` 每拍问 `sensor_health_->status(max_age_sec)`。不健康就按状态写成 `scan_disabled`、`scan_stale` 或 `scan_unavailable`，返回 `FAILURE`。原因字符串和上次一样时不再打日志，避免 10 Hz 刷屏。`was_healthy_` 用来在从失败恢复时打一条「`/scan` 恢复」。

## 句柄还没回来时怎么取消

`async_send_goal` 立刻返回。服务器接受后才在 `goal_response_callback` 里给出句柄。`GoalSlot` 把这段记成 `WaitingHandle`。

```cpp
void begin_send()
{
  phase_ = Phase::WaitingHandle;
  cancel_requested_ = false;
}

bool handle_arrived_should_cancel()
{
  phase_ = Phase::Active;
  return cancel_requested_;
}

CancelAction request_cancel()
{
  if (phase_ == Phase::Idle) {
    return CancelAction::None;
  }
  cancel_requested_ = true;
  if (phase_ == Phase::WaitingHandle) {
    return CancelAction::Armed;
  }
  return CancelAction::CancelHandle;
}
```

`begin_send()` 会把上次的取消标记清掉。否则上一次超时留下的「等句柄后取消」会打到重试的新 Goal 上。

每次 `send_goal` 先 `++send_seq_`，把序号捕获进两个回调。句柄到达时先看序号。序号已经不是这一次，说明更新的 `send_goal` 已经发出，这个旧句柄若非空就 `async_cancel_goal`，然后返回。序号还对、句柄是空的，就是服务器拒绝，`goal_slot_.finish()` 后回调 `REJECTED`。句柄有效才保存起来，并调用 `on_goal_sent_`，这就是指标一的时刻。若等待期间已经 `request_cancel()`，`handle_arrived_should_cancel()` 返回 true，这里立刻取消这个句柄。

结果回调同样先对序号。过期结果直接丢掉。对得上才按 `ResultCode` 映射成 `SUCCEEDED`、`CANCELED` 或 `ABORTED`，并 `finish()` 把槽放回 `Idle`。

位姿写在 `map` 坐标系。平面朝向只绕 Z：`orientation.z = sin(yaw/2)`，`orientation.w = cos(yaw/2)`。这是 yaw 对应的四元数。`header.stamp` 用节点钟 `now()`。

## 停车

Gazebo 差速模型的 `command_topic` 是 `cmd_vel_safe`。`EStop` 订阅 `/cmd_vel`，未锁存时原样转发到 `/cmd_vel_safe`。两边 QoS 都是 reliable、volatile、深度 10。

```cpp
cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
  "/cmd_vel", qos,
  [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (latched_ || !cmd_vel_pub_) {
      return;
    }
    cmd_vel_pub_->publish(*msg);
  });
```

`halt()` 把 `latched_` 设为 true。之后输入直接丢掉，并用节点钟每 50 ms 发一条全零 `Twist`。`Twist` 的默认构造就是六个 0。Gazebo 会保持最后一条速度，所以要持续发零，不能只发一次。`/clock` 大约 10 Hz，这个 50 ms 定时器在仿真时间上会落到大约 100 ms 一拍。`ensure_latch_timer()` 发现定时器已经在就直接返回，所以多次 `halt()` 只会打一次锁存日志。

`release()` 把 `latched_` 设回 false 并 `cancel()` 定时器，再 `reset()` 把 `SharedPtr` 放掉。下一轮 `onStart` 在 `send_goal` 之前调用它，重试发出的速度才不会被零盖住。最后一次失败后面没有新的 `onStart`，锁存留着。

`mission_node` 若在成功路径上 `shutdown()`，进程没了，就没有人再发 `/cmd_vel_safe`。失败和取消故意不退出，零速度定时器才能继续跑。

## 雷达新鲜度

`SensorHealth` 订阅的是 `/scan`，QoS 用 `SensorDataQoS()`。回调里记下的是 `node_->now()` 的纳秒，不看 `LaserScan.stamp`。`scan_delay_node` 会把消息晚转发，stamp 仍是采集时刻；用 stamp 判断会把延迟看成雷达死了。

```cpp
scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
  topic, rclcpp::SensorDataQoS(),
  [this](const sensor_msgs::msg::LaserScan::SharedPtr) {
    ever_received_.store(true);
    freshness_.mark(node_->now().nanoseconds());
  });
```

`ScanFreshness::healthy` 在还没有时间戳（`last_ns_ < 0`），或者现在比上次到达还早时返回 false。否则看 `now_ns - last_ns` 是否不超过允许年龄。`last_ns_` 是 `mutable std::atomic`，所以 `healthy` 可以是 `const` 成员函数，时钟回跳时仍然能改这个时刻。

`status()` 先看参数 `scan_health_enabled`。false 就是 `Disabled`，也就是实验里的 `fault scan_disabled`。仿真钟回跳超过 1 秒时，把到达时刻改成现在，避免旧时间和新时间相减。还没收到过帧、宽限记在 1 秒以内、而现在已经超过 2 秒时，再记一笔，交给启动宽限。

还没收到过帧时，允许的年龄是 `max(max_age_sec, 5 秒)`，所以第一帧之前大约有 5 秒看起来是健康的。比较失败时，已经收过帧就是 `Stale`，否则是 `NoData`。树里的 `max_age_sec` 是 1。

`mark_fresh()` 只在 `ever_received_` 为 false、且还没有任何时间戳时记一笔。开始 tick 时调用它，给的是启动宽限，不会把已经断流的时刻改成现在。

## 任务文本和 MQTT

`/patrol_task` 和 MQTT 载荷走同一个解析。`trim_lower_copy` 用 `std::find_if` 去掉首尾空白，再逐个字符 `tolower`。只认 `start` 和 `cancel`，别的是 `Ignore`，`drain_commands()` 打一条警告。

```cpp
inline TaskCommand parse_task_command(const std::string & payload)
{
  const std::string text = trim_lower_copy(payload);
  if (text == "start") {
    return TaskCommand::Start;
  }
  if (text == "cancel") {
    return TaskCommand::Cancel;
  }
  return TaskCommand::Ignore;
}
```

MQTT 回调和 `poll()` 不是同一条线程，所以入队要加锁。`drain_commands()` 在锁里 `swap` 出整个队列，锁外再解析，避免拿着锁去做 ROS 的事。

`mqtt_enable` 默认 false，所以 `scripts/start.sh` 只走 `auto_start`。打开之后，`MqttTaskClient` 在自己的线程里做 MQTT 3.1.1：连 `127.0.0.1:1883`，客户端 id 是 `sitl-mission`，订阅 `sitl/patrol/task`，状态发到 `sitl/patrol/status`。QoS 是 0。第二个 `mission_node` 用同一个客户端 id 连上时，broker 会踢掉前一个。

收到的载荷只交给构造时传入的那个函数，函数里是 `enqueue_command`。状态字符串同时发 ROS 话题和 MQTT。发出去的队列最多 16 条，满了丢最旧的。有一根 wake pipe，这样发布状态不用干等读超时。断线在那条线程里重连，不占 ROS 的 executor。`getaddrinfo` 仍可能堵住这条 MQTT 线程，不会堵住 `spin`。

## 两个对照进程

它们和 `mission_node` 是不同的可执行文件，默认 launch 会拉起来，但默认不制造故障。

`scan_delay_node` 把 `/scan` 放进 `deque<pair<释放时刻, LaserScan>>`。释放时刻是「到达时的节点钟 + delay_ms」。默认为 0。10 ms 定时器看到现在已经不早于队首，就把原来的消息原样发到 `/scan_delayed`。stamp、frame、点云都不改。Nav2 代价地图看的是 `/scan_delayed`，AMCL 仍看 `/scan`。

`odom_tf_gate` 订 `/odom`，回调里只把最新消息存下来。另有 50 ms 定时器发 `odom → base_footprint`。平移和旋转用这份里程，`header.stamp` 用 `now()`。订阅回调和定时器都会碰 `last_odom_`，所以这里有一把 `mutex`，和任务节点里单线程的状态不一样。`stall:=true` 时定时器直接 return，TF 停发，不发零速度。车会按最后一条 `Twist` 继续滑。这个开关和雷达延迟要分成两次跑。
