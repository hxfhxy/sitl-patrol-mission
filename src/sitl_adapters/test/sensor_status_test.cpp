#include "sitl_adapters/sensor_health.hpp"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

namespace {

class SensorStatusTest : public ::testing::Test {
public:
  static void SetUpTestSuite()
  {
    setenv("ROS_DOMAIN_ID", "27", 1);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(SensorStatusTest, NoDataThenFreshThenDisabled)
{
  auto node = std::make_shared<rclcpp::Node>("sensor_status_test");
  sitl_adapters::SensorHealth health(node.get());

  EXPECT_EQ(health.status(1.0), sitl_adapters::ScanHealthStatus::NoData);

  health.mark_fresh();
  EXPECT_EQ(health.status(1.0), sitl_adapters::ScanHealthStatus::Ok);
  health.mark_fresh();
  EXPECT_EQ(health.status(1.0), sitl_adapters::ScanHealthStatus::Ok);

  node->set_parameter(rclcpp::Parameter("scan_health_enabled", false));
  EXPECT_EQ(health.status(1.0), sitl_adapters::ScanHealthStatus::Disabled);
  EXPECT_FALSE(health.is_healthy(1.0));
}

TEST_F(SensorStatusTest, MarkFreshDoesNotHideARealGap)
{
  auto node = std::make_shared<rclcpp::Node>(
    "sensor_stale_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  sitl_adapters::SensorHealth health(node.get());
  auto pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "base_scan";
  bool fresh = false;
  for (int i = 0; i < 30 && !fresh; ++i) {
    pub->publish(scan);
    rclcpp::spin_some(node);
    fresh = health.status(10.0) == sitl_adapters::ScanHealthStatus::Ok;
    if (!fresh) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  ASSERT_TRUE(fresh);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  health.mark_fresh();
  EXPECT_EQ(health.status(0.05), sitl_adapters::ScanHealthStatus::Stale);
}

}  // namespace
