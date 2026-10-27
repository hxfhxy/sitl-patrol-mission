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

// 一个路点。IDLE 进 onStart，RUNNING 进 onRunning。
// onHalted 只在状态仍是 RUNNING 时由框架调用；onRunning 返回 FAILURE 后再 halt 不会进来。
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
  sitl_adapters::NavAdapter * nav_adapter_{nullptr};  // 不拥有，生命周期归 MissionNode。
  sitl_adapters::EStop * estop_{nullptr};

  // 每开始一次或作废一次就加一。结果回调捕获旧值，对不上就丢掉。
  std::atomic<uint32_t> goal_gen_{0};
  std::atomic<bool> goal_done_{false};
  std::atomic<sitl_adapters::NavResult> last_result_{sitl_adapters::NavResult::ABORTED};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};  // 节点钟，超时不跟墙钟。
  double timeout_sec_{60.0};
};

// 条件节点，每拍同步返回。失败会让外层 ReactiveSequence halt 正在跑的 NavigateTo。
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
  bool was_healthy_{true};  // 从失败变回健康时打一条恢复日志。
};

void register_bt_nodes(
  BT::BehaviorTreeFactory & factory,
  sitl_adapters::NavAdapter * nav_adapter,
  sitl_adapters::EStop * estop,
  sitl_adapters::SensorHealth * sensor_health);

}  // namespace sitl_mission_bt