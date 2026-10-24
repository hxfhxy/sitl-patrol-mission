#include "sitl_mission_bt/bt_nodes.hpp"

#include "rclcpp/rclcpp.hpp"

namespace sitl_mission_bt {

namespace {

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

const char * scan_reason(sitl_adapters::ScanHealthStatus status)
{
  switch (status) {
    case sitl_adapters::ScanHealthStatus::Disabled:
      return "scan_disabled";
    case sitl_adapters::ScanHealthStatus::Stale:
      return "scan_stale";
    case sitl_adapters::ScanHealthStatus::NoData:
      return "scan_unavailable";
    case sitl_adapters::ScanHealthStatus::Ok:
      return "";
  }
  return "scan_unavailable";
}

}  // namespace

NavigateTo::NavigateTo(
  const std::string & name,
  const BT::NodeConfiguration & config,
  sitl_adapters::NavAdapter * nav_adapter,
  sitl_adapters::EStop * estop)
: BT::StatefulActionNode(name, config),
  nav_adapter_(nav_adapter),
  estop_(estop) {}

BT::NodeStatus NavigateTo::onStart()
{
  if (!nav_adapter_) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_nodes"), "[BT:%s] nav_adapter 为空！", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_nodes"), "[BT:%s] 缺少 x/y/yaw 端口输入！", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("timeout_sec", timeout_sec_)) {
    timeout_sec_ = 60.0;
  }

  const uint32_t gen = ++goal_gen_;
  goal_done_.store(false);
  start_time_ = nav_adapter_->now();
  if (estop_) {
    estop_->release();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("bt_nodes"),
    "[BT:%s] onStart (gen=%u) -> 发送目标: (x=%.2f, y=%.2f, yaw=%.2f, timeout=%.1fs)",
    name().c_str(), gen, x, y, yaw, timeout_sec_);

  nav_adapter_->send_goal(
    x, y, yaw,
    [this, gen](sitl_adapters::NavResult res) {
      if (gen != goal_gen_.load()) {
        // 丢弃上一轮被取消/超时的旧 Goal 异步回调
        return;
      }
      last_result_.store(res);
      goal_done_.store(true);
    });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning()
{
  if (!goal_done_.load()) {
    const double elapsed = (nav_adapter_->now() - start_time_).seconds();
    if (elapsed > timeout_sec_) {
      // 超时：使当前 gen 失效，取消 Goal 并 halt，把这一次失败交给 RetryUntilSuccessful
      ++goal_gen_;
      RCLCPP_WARN(
        rclcpp::get_logger("bt_nodes"),
        "[BT:%s] onRunning -> 导航超时 (%.2fs > %.2fs)，取消后交给重试",
        name().c_str(), elapsed, timeout_sec_);
      set_fault_reason(*this, "nav_timeout", true);
      nav_adapter_->cancel_current_goal();
      if (estop_) {
        estop_->halt();
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto res = last_result_.load();
  if (res == sitl_adapters::NavResult::SUCCEEDED) {
    const double elapsed = (nav_adapter_->now() - start_time_).seconds();
    RCLCPP_INFO(
      rclcpp::get_logger("bt_nodes"),
      "[BT:%s] onRunning -> SUCCEEDED | Goal 端到端 %.2f s",
      name().c_str(), elapsed);
    return BT::NodeStatus::SUCCESS;
  }

  const char * reason = "nav_aborted";
  if (res == sitl_adapters::NavResult::REJECTED) {
    reason = "nav_rejected";
  } else if (res == sitl_adapters::NavResult::CANCELED) {
    reason = "nav_canceled";
  }
  set_fault_reason(*this, reason, true);
  RCLCPP_ERROR(
    rclcpp::get_logger("bt_nodes"),
    "[BT:%s] onRunning -> FAILURE (NavResult=%d)",
    name().c_str(), static_cast<int>(res));
  return BT::NodeStatus::FAILURE;
}

void NavigateTo::onHalted()
{
  ++goal_gen_;
  RCLCPP_WARN(
    rclcpp::get_logger("bt_nodes"),
    "[BT:%s] onHalted -> 1. cancel_current_goal() | 2. estop->halt()",
    name().c_str());
  if (nav_adapter_) {
    nav_adapter_->cancel_current_goal();
  }
  if (estop_) {
    estop_->halt();
  }
  set_fault_reason(*this, "canceled", false);
}

IsScanHealthy::IsScanHealthy(
  const std::string & name,
  const BT::NodeConfiguration & config,
  sitl_adapters::SensorHealth * sensor_health)
: BT::ConditionNode(name, config), sensor_health_(sensor_health)
{
}

BT::NodeStatus IsScanHealthy::tick()
{
  if (!sensor_health_) {
    return BT::NodeStatus::FAILURE;
  }
  double max_age_sec = 1.0;
  getInput("max_age_sec", max_age_sec);
  const auto health = sensor_health_->status(max_age_sec);
  const bool healthy = health == sitl_adapters::ScanHealthStatus::Ok;
  if (!healthy) {
    const char * reason = scan_reason(health);
    std::string existing;
    const auto blackboard = config().blackboard;
    const bool same = blackboard && blackboard->get("fault_reason", existing) && existing == reason;
    if (!same) {
      if (health == sitl_adapters::ScanHealthStatus::Disabled) {
        RCLCPP_ERROR(
          rclcpp::get_logger("bt_nodes"),
          "[BT:%s] scan_health_enabled=false，雷达检查已关闭", name().c_str());
      } else if (health == sitl_adapters::ScanHealthStatus::NoData) {
        RCLCPP_ERROR(
          rclcpp::get_logger("bt_nodes"),
          "[BT:%s] /scan 还没有收到可用帧", name().c_str());
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("bt_nodes"),
          "[BT:%s] /scan 超过 %.2f s 没有新到达", name().c_str(), max_age_sec);
      }
    }
    set_fault_reason(*this, reason, true);
  } else if (!was_healthy_ && healthy) {
    RCLCPP_INFO(rclcpp::get_logger("bt_nodes"), "[BT:%s] /scan 恢复", name().c_str());
  }
  was_healthy_ = healthy;
  return healthy ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void register_bt_nodes(
  BT::BehaviorTreeFactory & factory,
  sitl_adapters::NavAdapter * nav_adapter,
  sitl_adapters::EStop * estop,
  sitl_adapters::SensorHealth * sensor_health)
{
  factory.registerBuilder<NavigateTo>(
    "NavigateTo",
    [nav_adapter, estop](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<NavigateTo>(name, config, nav_adapter, estop);
    });
  factory.registerBuilder<IsScanHealthy>(
    "IsScanHealthy",
    [sensor_health](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<IsScanHealthy>(name, config, sensor_health);
    });
}

}  // namespace sitl_mission_bt