#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace sitl_adapters {

// /cmd_vel 到 /cmd_vel_safe 的唯一发布者。锁存后丢掉 Nav2 的速度，改发零。
class EStop {
public:
  explicit EStop(rclcpp::Node * node = nullptr);
  virtual ~EStop() = default;

  // 锁存零速度，直到 release()。车只听 /cmd_vel_safe，Nav2 的 /cmd_vel 在锁存期间被丢掉。
  virtual void halt();

  // 下一次导航开始前解开锁存。否则 Retry 刚发出的新速度会被零速盖住。
  virtual void release();

private:
  void publish_zero();
  void ensure_latch_timer();

  rclcpp::Node * node_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // 锁存期间按节点钟 50 ms 发零。/clock 10 Hz 时实际大约 100 ms 一拍。
  rclcpp::TimerBase::SharedPtr latch_timer_;
  bool latched_{false};
};

}  // namespace sitl_adapters