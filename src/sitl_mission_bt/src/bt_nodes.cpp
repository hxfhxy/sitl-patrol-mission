#include "sitl_mission_bt/bt_nodes.hpp"

#include "rclcpp/rclcpp.hpp"

namespace sitl_mission_bt {

namespace {

// overwrite 为 false 时，已有原因留下。onHalted 的 canceled 不能盖掉雷达故障。
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
  // 端口来自 XML 属性。缺 x/y/yaw 是树写错了，直接失败，不发 Goal。
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_nodes"), "[BT:%s] 缺少 x/y/yaw 端口输入！", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 没写 timeout_sec 时用端口默认值 60。读失败也回到 60。
  if (!getInput("timeout_sec", timeout_sec_)) {
    timeout_sec_ = 60.0;
  }

  const uint32_t gen = ++goal_gen_;
  goal_done_.store(false);
  start_time_ = nav_adapter_->now();
  // 先解开急停再发 Goal。重试的新速度才不会被零速盖住。
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
      // 超时和 halt 都会把 goal_gen_ 往前推，旧回调到了也不能结束新的一次。
      if (gen != goal_gen_.load()) {
        // 丢弃上一轮被取消/超时的旧 Goal 异步回调
        return;
      }
      last_result_.store(res);
      goal_done_.store(true);
    });

  // 即使结果回调已经在 send_goal 里同步触发，这一拍也先返回 RUNNING，下一拍 onRunning 再收。
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning()
{
  if (!goal_done_.load()) {
    const double elapsed = (nav_adapter_->now() - start_time_).seconds();
    if (elapsed > timeout_sec_) {
      // 先作废这一代，再取消。返回 FAILURE 后状态不再是 RUNNING，随后的 Retry halt 不会进 onHalted。
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
  // 成功不写 fault_reason。这一圈里更早的 nav_timeout 由 mission_node 在整圈成功后清掉。
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
  // 雷达失败和用户取消走这里。超时不走，因为那时已经不是 RUNNING。
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
  // 不覆盖。黑板上已有 scan_* 或 nav_* 时，不要改写成 canceled。
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
  // 同一原因每拍都会 tick。字符串没变就不再打日志。
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
    // 先写原因再返回 FAILURE。随后 onHalted 的 canceled 不能盖掉它。
    set_fault_reason(*this, reason, true);
  } else if (!was_healthy_ && healthy) {
    RCLCPP_INFO(rclcpp::get_logger("bt_nodes"), "[BT:%s] /scan 恢复", name().c_str());
  }
  was_healthy_ = healthy;
  return healthy ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// XML 标签名就是这里的字符串。三个点共用同一对适配器指针。
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