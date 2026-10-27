#include <algorithm>
#include <fstream>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// 量 /cmd_vel 回调间隔，不是 /cmd_vel_safe，也不是 Twist 自带的时间戳（Twist 没有 header）。
// 不在默认 launch 里。析构时把间隔写到 /tmp/cmd_vel_jitter.csv。
class CmdVelProbe : public rclcpp::Node
{
public:
  CmdVelProbe()
  : Node("cmd_vel_probe")
  {
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).reliable().durability_volatile(),
      [this](const geometry_msgs::msg::Twist::SharedPtr) {
        // 记的是回调到达时的节点钟。use_sim_time 时会被 /clock 量化成 0 或约 100 ms。
        stamps_ns_.push_back(get_clock()->now().nanoseconds());
      });

    RCLCPP_INFO(
      get_logger(), "CmdVelProbe 已启动，监听 /cmd_vel | use_sim_time=%s",
      get_parameter("use_sim_time").as_bool() ? "true" : "false");
  }

  ~CmdVelProbe() override
  {
    dump_csv_and_summary();
  }

private:
  void dump_csv_and_summary()
  {
    if (stamps_ns_.size() < 2) {
      RCLCPP_WARN(get_logger(), "采集到的 /cmd_vel 样本数不足 (<2)，未生成统计。");
      return;
    }

    std::vector<double> dt_ms;
    dt_ms.reserve(stamps_ns_.size() - 1);

    std::ofstream csv("/tmp/cmd_vel_jitter.csv");
    csv << "index,stamp_ns,dt_ms\n";

    size_t zero_count = 0;
    for (size_t i = 1; i < stamps_ns_.size(); ++i) {
      const double dt = static_cast<double>(stamps_ns_[i] - stamps_ns_[i - 1]) / 1e6;
      // 仿真钟 10 Hz、控制 20 Hz 时，相邻两条可能落在同一个 /clock 拍上，间隔就是 0。
      if (dt == 0.0) {
        ++zero_count;
      }
      dt_ms.push_back(dt);
      csv << i << "," << stamps_ns_[i] << "," << dt << "\n";
    }
    csv.close();

    std::sort(dt_ms.begin(), dt_ms.end());
    const auto pct = [&](double p) {
      return dt_ms[static_cast<size_t>(p * (dt_ms.size() - 1))];
    };

    RCLCPP_INFO(get_logger(), "cmd_vel_probe 统计结果");
    RCLCPP_INFO(
      get_logger(),
      "样本总数: %zu (其中 dt=0 样本数: %zu) | P50 = %.2f ms | P99 = %.2f ms | max = %.2f ms",
      dt_ms.size(), zero_count, pct(0.50), pct(0.99), dt_ms.back());
    RCLCPP_INFO(get_logger(), "CSV 已写入: /tmp/cmd_vel_jitter.csv");
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::vector<int64_t> stamps_ns_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelProbe>());
  rclcpp::shutdown();
  return 0;
}
