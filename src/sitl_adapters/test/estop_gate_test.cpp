#include "sitl_adapters/estop.hpp"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"

namespace {

class EStopGateTest : public ::testing::Test {
public:
  static void SetUpTestSuite()
  {
    setenv("ROS_DOMAIN_ID", "28", 1);
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

TEST_F(EStopGateTest, LatchDropsNavAndReleaseForwardsAgain)
{
  auto node = std::make_shared<rclcpp::Node>("estop_gate_test");
  sitl_adapters::EStop estop(node.get());
  const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);

  geometry_msgs::msg::Twist latest;
  bool have = false;
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_safe", qos,
    [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
      latest = *msg;
      have = true;
    });

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.2;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline && !(have && latest.linear.x > 0.1)) {
    pub->publish(moving);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(have);
  EXPECT_GT(latest.linear.x, 0.1);

  estop.halt();
  have = false;
  const auto halted_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < halted_deadline) {
    pub->publish(moving);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(have);
  EXPECT_DOUBLE_EQ(latest.linear.x, 0.0);

  estop.release();
  have = false;
  const auto released_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < released_deadline && !(have && latest.linear.x > 0.1)) {
    pub->publish(moving);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(have);
  EXPECT_GT(latest.linear.x, 0.1);
  (void)sub;
}

}  // namespace
