#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sitl_adapters/goal_slot.hpp"

namespace sitl_adapters {

enum class NavResult {
  SUCCEEDED,
  CANCELED,
  ABORTED,
  REJECTED
};

class NavAdapter {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ResultCallback = std::function<void(NavResult)>;

  explicit NavAdapter(rclcpp::Node * node = nullptr);
  virtual ~NavAdapter() = default;

  // 等待 /navigate_to_pose Action Server 上线
  virtual bool wait_for_server(std::chrono::seconds timeout);

  // 发送 map 坐标系下的目标位姿 (x, y, yaw)
  virtual void send_goal(double x, double y, double yaw, ResultCallback on_result);

  // 取消当前 Goal。句柄还没回来时也会记上一笔，等句柄到达后立刻取消。
  // 只有根本没有在飞的 Goal 时才返回 false。
  virtual bool cancel_current_goal();

  // 获取当前节点钟时间（供 BT 节点按仿真时钟判断超时，测试时可 override）
  virtual rclcpp::Time now() const;

  // Nav2 接受当前这次 Goal 时调用。拒绝、或已经过期的句柄不算。
  void set_on_goal_sent(std::function<void()> callback);

private:
  rclcpp::Node * node_{nullptr};
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_{nullptr};
  std::function<void()> on_goal_sent_;
  GoalSlot goal_slot_;
  uint32_t send_seq_{0};
};

}  // namespace sitl_adapters