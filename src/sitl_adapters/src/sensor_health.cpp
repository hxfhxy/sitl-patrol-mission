#include "sitl_adapters/sensor_health.hpp"

#include <algorithm>
#include <string>

namespace sitl_adapters {

namespace {
constexpr int64_t kNsPerSec = 1000000000LL;
constexpr double kStartupGraceSec = 5.0;
}

SensorHealth::SensorHealth(rclcpp::Node * node) : node_(node)
{
  if (!node_) {
    return;
  }

  node_->declare_parameter<bool>("scan_health_enabled", true);
  node_->declare_parameter<std::string>("scan_health_topic", "/scan");
  const std::string topic = node_->get_parameter("scan_health_topic").as_string();

  // 订原始 /scan，不订 /scan_delayed。延迟实验不能被看成断连。
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr) {
      ever_received_.store(true);
      freshness_.mark(node_->now().nanoseconds());
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "SensorHealth 订阅 %s（到达时刻，不改 stamp）", topic.c_str());
}

ScanHealthStatus SensorHealth::status(double max_age_sec) const
{
  // 无节点是测试里的空壳，按还没数据处理。
  if (!node_) {
    return ScanHealthStatus::NoData;
  }
  // 实验用参数关掉检查。这是 scan_disabled，不是雷达真的断了。
  if (!node_->get_parameter("scan_health_enabled").as_bool()) {
    return ScanHealthStatus::Disabled;
  }

  int64_t now_ns = node_->now().nanoseconds();
  int64_t last_ns = freshness_.last_ns();
  // /clock 回跳时，旧的到达时刻不能拿来和现在相减。
  if (last_ns >= 0 && now_ns + kNsPerSec < last_ns) {
    freshness_.mark(now_ns);
    last_ns = now_ns;
  }
  // 仿真钟从 0 跳到正式时间，且还没有新雷达帧。重新记一笔，交给下面的启动宽限。
  if (!ever_received_.load() && last_ns >= 0 && last_ns < kNsPerSec && now_ns > 2 * kNsPerSec) {
    freshness_.mark(now_ns);
    last_ns = now_ns;
  }

  const bool ever = ever_received_.load();
  // 第一帧之前放宽到至少 5 s。收到过之后就按树上的 max_age_sec。
  const double limit = ever ? max_age_sec : std::max(max_age_sec, kStartupGraceSec);
  if (freshness_.healthy(now_ns, static_cast<int64_t>(limit * kNsPerSec))) {
    return ScanHealthStatus::Ok;
  }
  // 收过帧再超时是断流；从没收到是还没有第一帧。
  return ever ? ScanHealthStatus::Stale : ScanHealthStatus::NoData;
}

void SensorHealth::mark_fresh()
{
  // 只补「还没有任何到达记录」的启动宽限。已经收过帧，或这次宽限已经记过，就不要再改时刻。
  if (!node_ || ever_received_.load() || freshness_.last_ns() >= 0) {
    return;
  }
  freshness_.mark(node_->now().nanoseconds());
}

}  // namespace sitl_adapters
