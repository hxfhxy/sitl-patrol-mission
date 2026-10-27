#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sitl_adapters/estop.hpp"
#include "sitl_adapters/nav_adapter.hpp"
#include "sitl_adapters/sensor_health.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace sitl_mission_node {

class MqttTaskClient;

// 任务阶段。只有 WaitTask / Fault / Canceled 能再接受 start。
enum class Phase {
  WaitTask,  // 还没有任务。auto_start 为 false 时停在这里。
  Arming,    // 已接任务，等 Nav2 Active。此阶段用墙钟，最多 60 s。
  Running,   // 10 Hz tick 行为树。
  Fault,     // 树失败或 Nav2 没起来。节点留着，零速度继续发。
  Canceled,  // 用户取消。同样留着节点。
  Done       // 设定圈数走完，随后 shutdown。
};

// 巡检调度。单线程 spin：阶段、Goal 槽、急停锁存都不加锁。
// 成员声明顺序有意义：树在适配器后面，析构时树先销毁，适配器指针仍然有效。
class MissionNode : public rclcpp::Node {
public:
  MissionNode();
  ~MissionNode() override;

private:
  // 200 ms 墙钟。先消化命令队列，Arming 时再问 Nav2 是否 Active。
  void poll();
  // MQTT 线程调用。只入队，不改阶段。
  void enqueue_command(const std::string & payload);
  // 在 poll 里把队列 swap 出来再解析，避免拿着锁做 ROS 的事。
  void drain_commands();
  // external 为 true 时记下到达时刻，后面打指标一。
  void request_start(bool external);
  void request_cancel();
  // 非阻塞查 /bt_navigator/get_state。不能在定时器里 spin_until_future_complete。
  bool nav_is_active();
  void begin_ticking();
  void tick_tree();
  // 仅外部任务的第一个被接受的 Goal。auto_start 和后续圈都不记。
  void note_goal_sent();
  // 同一段文本发到 /patrol_status，MQTT 开着时再镜像一份。
  void publish_status(const std::string & text);

  // 适配器先声明。树节点拿裸指针，不能比它们先析构。
  std::unique_ptr<sitl_adapters::NavAdapter> nav_adapter_;
  std::unique_ptr<sitl_adapters::EStop> estop_;
  std::unique_ptr<sitl_adapters::SensorHealth> sensor_health_;
  BT::Tree tree_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  // transient_local、深度 1：后连上的订阅者只看到最新一条。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr lifecycle_client_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  // 节点钟 100 ms。use_sim_time 时跟 /clock，不跟墙钟。
  rclcpp::TimerBase::SharedPtr bt_timer_;
  // 最后声明，并在析构函数体里先 reset，让线程在别的成员拆掉之前退出。
  std::unique_ptr<MqttTaskClient> mqtt_;

  // 上一拍发出、还没回来的 get_state。空表示这一拍可以再发。
  std::optional<rclcpp::Client<lifecycle_msgs::srv::GetState>::FutureAndRequestId> lifecycle_future_;

  std::mutex command_mutex_;
  std::deque<std::string> commands_;

  Phase phase_{Phase::WaitTask};
  bool auto_start_{true};
  // 1：走完退出。0：成功后继续。负数在构造里改成 1。
  int patrol_loops_{1};
  int laps_finished_{0};
  bool external_task_{false};
  bool goal_metric_logged_{false};
  bool arm_announced_{false};
  // 外部 start 到达时的节点钟。和 arm_deadline_ 的墙钟不能相减。
  rclcpp::Time task_arrival_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point arm_deadline_{};
};

}  // namespace sitl_mission_node
