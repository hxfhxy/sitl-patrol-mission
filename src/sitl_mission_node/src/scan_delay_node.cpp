#include <deque>
#include <string>
#include <utility>

#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanDelayNode : public rclcpp::Node
{
public:
  using LaserScan = sensor_msgs::msg::LaserScan;

  ScanDelayNode()
  : Node("scan_delay_node")
  {
    declare_parameter<int>("delay_ms", 0);
    declare_parameter<std::string>("input_topic", "/scan");
    declare_parameter<std::string>("output_topic", "/scan_delayed");

    delay_ms_ = get_parameter("delay_ms").as_int();
    const std::string input_topic = get_parameter("input_topic").as_string();
    const std::string output_topic = get_parameter("output_topic").as_string();

    pub_ = create_publisher<LaserScan>(output_topic, rclcpp::SensorDataQoS());
    sub_ = create_subscription<LaserScan>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const LaserScan::SharedPtr msg) {
        const rclcpp::Time release_time =
          get_clock()->now() + rclcpp::Duration::from_seconds(delay_ms_ / 1000.0);
        queue_.emplace_back(release_time, *msg);
      });

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_nanoseconds(10LL * 1000LL * 1000LL),
      [this]() { on_timer(); });

    RCLCPP_INFO(
      get_logger(),
      "scan_delay_node 已启动: %s -> %s | delay_ms=%d | use_sim_time=%s",
      input_topic.c_str(), output_topic.c_str(), delay_ms_,
      get_parameter("use_sim_time").as_bool() ? "true" : "false");
  }

private:
  void on_timer()
  {
    const rclcpp::Time now = get_clock()->now();
    while (!queue_.empty() && now >= queue_.front().first) {
      pub_->publish(queue_.front().second);
      queue_.pop_front();
    }
  }

  int delay_ms_{0};
  rclcpp::Subscription<LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<std::pair<rclcpp::Time, LaserScan>> queue_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDelayNode>());
  rclcpp::shutdown();
  return 0;
}
