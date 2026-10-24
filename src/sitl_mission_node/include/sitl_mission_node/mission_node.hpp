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

enum class Phase {
  WaitTask,
  Arming,
  Running,
  Fault,
  Canceled,
  Done
};

class MissionNode : public rclcpp::Node {
public:
  MissionNode();
  ~MissionNode() override;

private:
  void poll();
  void enqueue_command(const std::string & payload);
  void drain_commands();
  void request_start(bool external);
  void request_cancel();
  bool nav_is_active();
  void begin_ticking();
  void tick_tree();
  void note_goal_sent();
  void publish_status(const std::string & text);

  std::unique_ptr<sitl_adapters::NavAdapter> nav_adapter_;
  std::unique_ptr<sitl_adapters::EStop> estop_;
  std::unique_ptr<sitl_adapters::SensorHealth> sensor_health_;
  BT::Tree tree_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr lifecycle_client_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr bt_timer_;
  std::unique_ptr<MqttTaskClient> mqtt_;

  std::optional<rclcpp::Client<lifecycle_msgs::srv::GetState>::FutureAndRequestId> lifecycle_future_;

  std::mutex command_mutex_;
  std::deque<std::string> commands_;

  Phase phase_{Phase::WaitTask};
  bool auto_start_{true};
  int patrol_loops_{1};
  int laps_finished_{0};
  bool external_task_{false};
  bool goal_metric_logged_{false};
  bool arm_announced_{false};
  rclcpp::Time task_arrival_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point arm_deadline_{};
};

}  // namespace sitl_mission_node
