#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sitl_adapters/estop.hpp"
#include "sitl_adapters/nav_adapter.hpp"
#include "sitl_adapters/sensor_health.hpp"

namespace sitl_mission_bt {

class NavigateTo : public BT::StatefulActionNode {
public:
  NavigateTo(
    const std::string & name,
    const BT::NodeConfiguration & config,
    sitl_adapters::NavAdapter * nav_adapter,
    sitl_adapters::EStop * estop = nullptr);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("yaw"),
      BT::InputPort<double>("timeout_sec", 60.0, "Navigation timeout in seconds (sim clock)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  sitl_adapters::NavAdapter * nav_adapter_{nullptr};
  sitl_adapters::EStop * estop_{nullptr};

  std::atomic<uint32_t> goal_gen_{0};
  std::atomic<bool> goal_done_{false};
  std::atomic<sitl_adapters::NavResult> last_result_{sitl_adapters::NavResult::ABORTED};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double timeout_sec_{60.0};
};

class IsScanHealthy : public BT::ConditionNode {
public:
  IsScanHealthy(
    const std::string & name,
    const BT::NodeConfiguration & config,
    sitl_adapters::SensorHealth * sensor_health);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_age_sec", 1.0, "Max /scan arrival age in seconds")
    };
  }

  BT::NodeStatus tick() override;

private:
  sitl_adapters::SensorHealth * sensor_health_{nullptr};
  bool was_healthy_{true};
};

void register_bt_nodes(
  BT::BehaviorTreeFactory & factory,
  sitl_adapters::NavAdapter * nav_adapter,
  sitl_adapters::EStop * estop,
  sitl_adapters::SensorHealth * sensor_health);

}  // namespace sitl_mission_bt