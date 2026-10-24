# sitl-patrol-mission

A handwritten ROS 2 Humble learning project: schedule an indoor patrol in simulation, and keep the application layer separate from the navigator.

这是我手写来学 ROS 2 应用层的项目。导航和车体用现成的 Nav2 与 TurtleBot3

## 目的

把一次室内巡检从「在 RViz 里点目标」收成一条能复现的任务：外部下发 `start`，行为树走完三个点，超时会重试，雷达检查失败会停车，过程能用测试和记录对上。

## 里面有什么

- **任务编排。** `patrol.xml` 里 A→B→C。每拍先看 `/scan`，每个点最多 3 次（含第一次）。
- **Nav2 接口。** `NavAdapter` 发 `/navigate_to_pose`，处理句柄还没回来时的取消。
- **停车。** 车只听 `/cmd_vel_safe`。失败或取消后锁存零速度，节点不退出。
- **任务入口。** MQTT `sitl/patrol/task`，或话题 `/patrol_task`。状态写到 `/patrol_status`。
- **仿真。** Gazebo Classic 里的 TurtleBot3 waffle，AMCL 自己设初始位姿。
- **对照实验。** 雷达延迟中继不改 stamp；里程 TF 可以单独停发，用来和雷达故障分开看。
- **验证。** 不启动仿真的 GTest；一次成功、一次故障的运行记录在 [docs/measurements.md](docs/measurements.md)。

## 状态

2026-10-24 在本机 Gazebo Classic 上用 MQTT `start` 跑完 A→B→C。外部任务到 Nav2 接受第一个 Goal 的间隔是 1.900 s。另一次在走向 A 时关闭雷达检查，状态为 `fault scan_disabled`，车停下，`mission_node` 仍在。GTest 不启动仿真。细节见 [docs/measurements.md](docs/measurements.md)。

## 环境

- Ubuntu 22.04
- ROS 2 Humble

```bash
sudo apt update
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-gazebo-ros \
  ros-humble-rosbag2 \
  ros-humble-behaviortree-cpp-v3 \
  ros-humble-rviz2
```

脚本会自己设置 `TURTLEBOT3_MODEL=waffle`，并把 `GAZEBO_MODEL_PATH` 设成 Gazebo 官方模型加上 turtlebot3 模型。`GAZEBO_MODEL_DATABASE_URI` 置空，避免启动时去拉在线模型。

MQTT 用本机 1883。装了 `mosquitto` 就用它。没装时 `scripts/mqtt_patrol.sh` 拉起仓库里的 `scripts/mqtt_broker.py`（MQTT 3.1.1，QoS 0，只监听 127.0.0.1，没有鉴权）。

## 编译和测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select sitl_adapters sitl_mission_bt sitl_mission_node sitl_bringup
source install/setup.bash
colcon test --packages-select sitl_adapters sitl_mission_bt sitl_mission_node
colcon test-result --verbose
```

GTest 覆盖雷达新鲜度、Goal 取消槽、急停锁存、行为树超时重试、雷达失败取消、任务文本和 MQTT 往返。这些测试不开 Gazebo。推到 GitHub 之后，`.github/workflows/ci.yml` 会在 Humble 容器里做同样的编译和测试。

## 跑仿真

需要图形界面。在仓库根目录：

```bash
bash scripts/start.sh
```

`auto_start` 默认为 true。`/bt_navigator` 进入 Active 且 `/navigate_to_pose` 可连之后，巡检自己开始。不要在 RViz 里点 Goal。AMCL 使用 `set_initial_pose`，不用再给 2D Pose Estimate。

取消：

```bash
bash scripts/cancel.sh
```

车应停下，`/mission_node` 还在。三个点都成功后，mission 进程自己退出。这条路径不经过 MQTT。

## 用 MQTT 跑

```bash
bash scripts/mqtt_patrol.sh success   # 录一份成功 bag
bash scripts/mqtt_patrol.sh fault     # 走到一半关掉雷达检查
bash scripts/mqtt_patrol.sh both
```

脚本以 `auto_start:=false mqtt_enable:=true` 启动仿真，等 Nav2 Active 后向 `sitl/patrol/task` 发 `start`。成功时 `/patrol_status` 出现 `succeeded`，故障时出现 `fault scan_disabled`。

有显示器且装了 ffmpeg 时，成功那次会把整块桌面录到 `docs/demo.mp4`。这个文件不进 git：画面里是整桌，不只是仿真窗口。

bag 目录也不进 git。仓库里留的是校验和 `bags/patrol_success.sha256`、`bags/patrol_fault.sha256`，以及探针导出的 CSV。回放本地 bag：

```bash
ros2 bag info bags/patrol_success
ros2 bag play bags/patrol_success
```

已录上的话题是 `/patrol_status`、`/cmd_vel`、`/cmd_vel_safe`、`/odom`、`/scan`。`/clock` 当时因 QoS 不一致是 0 条：Gazebo 用 best effort，录包配置写成了 reliable。

没有 broker 时，也可以直接发 ROS 话题：

```bash
ros2 topic pub --once /patrol_task std_msgs/msg/String "{data: start}"
```

载荷只有 `start` 或 `cancel`。外部 `start` 才会记录指标一（任务到达到 Nav2 接受第一个 Goal）。`auto_start` 没有这个事件。

## 仓库布局

| 路径 | 内容 |
| --- | --- |
| `src/sitl_bringup` | launch、Nav2 参数、改过的 waffle 模型 |
| `src/sitl_adapters` | `NavAdapter`、`EStop`、`SensorHealth` |
| `src/sitl_mission_bt` | 行为树节点和 `patrol.xml` |
| `src/sitl_mission_node` | 任务阶段、MQTT、探针、雷达延迟、TF 门控 |
| `scripts/` | 启动、取消、MQTT 巡检、broker |
| `docs/architecture.md` | 树的形状和话题怎么接 |
| `docs/interfaces.md` | 状态字、时钟和注入实验的判定 |
| `docs/measurements.md` | 已经记录下来的数字 |

路点只写在 `src/sitl_mission_bt/behavior_trees/patrol.xml`。

## 失败时

- 单个点超时或导航失败：一共 3 次，含第一次。下一次 `onStart` 会 `release()`，速度重新转到 `/cmd_vel_safe`。
- 三次用尽，或雷达检查失败：`/patrol_status` 为 `fault <reason>`，零速度锁存，节点不退出。再发 `start` 可以重做。
- 关开关的原因是 `scan_disabled`。超过 1 s 没有新的 `/scan` 回调才是 `scan_stale`。还没有第一帧是 `scan_unavailable`。
- `/cancel_patrol`、MQTT `cancel`、`/patrol_task` 的 `cancel` 都停车并停树，状态是 `canceled`。这不是 FAULT，进程也不退出。
- 60 s 墙钟内 Nav2 没有 Active：`fault nav_inactive`。

仿真已经在跑、车已经出发时，可以这样注入雷达检查关闭：

```bash
ros2 param set /mission_node scan_health_enabled false
```

这次检查订的是 `/scan`，不是 `/scan_delayed`。雷达延迟和 `odom_tf_gate` 的 `stall` 分成两次跑。

## 许可

Apache-2.0。见 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。模型与 Nav2 参数来自 TurtleBot3 / Nav2，修改点写在 NOTICE 里。
