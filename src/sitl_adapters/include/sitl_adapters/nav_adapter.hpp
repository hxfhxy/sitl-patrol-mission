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

// 把 rclcpp_action 的 ResultCode 收成行为树能分支的四种结果。
enum class NavResult {
  SUCCEEDED,
  CANCELED,
  ABORTED,
  REJECTED  // 服务器拒绝，或节点还没有 action client。
};

// /navigate_to_pose 的客户端。虚函数是为了让 GTest 用 FakeNav 换掉真服务器。
// 假定和 mission_node 同一个单线程 executor，send_seq_ 和句柄不加锁。
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
  rclcpp::Node * node_{nullptr};  // 不拥有。测试传 nullptr，所有发送走 REJECTED。
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_{nullptr};
  std::function<void()> on_goal_sent_;
  GoalSlot goal_slot_;
  // 每次 send_goal 加一。回调捕获当时的序号，对不上就是过期句柄或过期结果。
  uint32_t send_seq_{0};
};

}  // namespace sitl_adapters