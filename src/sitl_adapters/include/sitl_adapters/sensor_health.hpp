#pragma once

#include <atomic>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace sitl_adapters {

// 到达时刻的新鲜度。不看 LaserScan.stamp：stamp 是采集时间，延迟实验会故意让它变旧。
class ScanFreshness {
public:
  void mark(int64_t now_ns) const { last_ns_.store(now_ns); }

  int64_t last_ns() const { return last_ns_.load(); }

  bool healthy(int64_t now_ns, int64_t max_age_ns) const
  {
    const int64_t last = last_ns_.load();
    if (last < 0 || now_ns < last) {
      return false;
    }
    return now_ns - last <= max_age_ns;
  }

private:
  mutable std::atomic<int64_t> last_ns_{-1};
};

enum class ScanHealthStatus {
  Ok,
  Disabled,
  Stale,
  NoData
};

class SensorHealth {
public:
  explicit SensorHealth(rclcpp::Node * node = nullptr);
  virtual ~SensorHealth() = default;

  // max_age_sec 只约束「已经收到过」的雷达。还没收到第一帧时给启动宽限，避免 /clock 刚起来就 FAULT。
  virtual ScanHealthStatus status(double max_age_sec) const;
  bool is_healthy(double max_age_sec) const
  {
    return status(max_age_sec) == ScanHealthStatus::Ok;
  }
  // 只在还没有任何到达记录时记一笔启动宽限。已经收过帧就不会再改时刻。
  virtual void mark_fresh();

private:
  rclcpp::Node * node_{nullptr};
  ScanFreshness freshness_;
  // 收到过真实帧之后，断流就是 Stale，不再享受 5 s 启动宽限。
  mutable std::atomic<bool> ever_received_{false};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace sitl_adapters
