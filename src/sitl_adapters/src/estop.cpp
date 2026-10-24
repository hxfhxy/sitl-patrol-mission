#include "sitl_adapters/estop.hpp"

#include "rclcpp/create_timer.hpp"

namespace sitl_adapters {

EStop::EStop(rclcpp::Node * node)
: node_(node)
{
  if (!node_) {
    return;
  }
  const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", qos);
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      if (latched_ || !cmd_vel_pub_) {
        return;
      }
      cmd_vel_pub_->publish(*msg);
    });
  RCLCPP_INFO(node_->get_logger(), "EStop 转发 /cmd_vel -> /cmd_vel_safe，锁存时只发 0。");
}

void EStop::halt()
{
  if (!node_ || !cmd_vel_pub_) {
    return;
  }
  if (!latched_) {
    RCLCPP_WARN(node_->get_logger(), "[EStop] 锁存零速度，直到下一次导航开始。");
  }
  latched_ = true;
  publish_zero();
  ensure_latch_timer();
}

void EStop::ensure_latch_timer()
{
  if (latch_timer_) {
    return;
  }
  latch_timer_ = rclcpp::create_timer(
    node_, node_->get_clock(), rclcpp::Duration::from_nanoseconds(50LL * 1000LL * 1000LL),
    [this]() {
      if (!latched_) {
        return;
      }
      publish_zero();
    });
}

void EStop::publish_zero()
{
  geometry_msgs::msg::Twist zero_msg;
  cmd_vel_pub_->publish(zero_msg);
}

void EStop::release()
{
  latched_ = false;
  if (latch_timer_) {
    latch_timer_->cancel();
    latch_timer_.reset();
  }
}

}  // namespace sitl_adapters
