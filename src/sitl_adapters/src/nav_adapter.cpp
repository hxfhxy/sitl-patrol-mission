#include "sitl_adapters/nav_adapter.hpp"

#include <cmath>

namespace sitl_adapters {

NavAdapter::NavAdapter(rclcpp::Node * node)
: node_(node)
{
  // 测试里的 FakeNav 传 nullptr，不创建客户端。
  if (node_) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
  }
}

bool NavAdapter::wait_for_server(std::chrono::seconds timeout)
{
  if (!action_client_) {
    return false;
  }
  return action_client_->wait_for_action_server(timeout);
}

void NavAdapter::send_goal(double x, double y, double yaw, ResultCallback on_result)
{
  // 没有客户端时同步拒绝，行为树不用再等句柄。
  if (!node_ || !action_client_) {
    if (on_result) {
      on_result(NavResult::REJECTED);
    }
    return;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  // 只绕 Z 的朝向。四元数是 (0, 0, sin(yaw/2), cos(yaw/2))。
  goal_msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
  goal_msg.pose.pose.orientation.w = std::cos(yaw * 0.5);

  const uint32_t seq = ++send_seq_;
  current_goal_handle_.reset();
  // 清掉上一轮「句柄一到就取消」的标记，否则会打到这次重试上。
  goal_slot_.begin_send();

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  options.goal_response_callback =
    [this, on_result, seq](const GoalHandleNavigateToPose::SharedPtr & handle) {
      // 更新的 send_goal 已经发出。这个旧句柄不能再当成当前 Goal。
      if (seq != send_seq_) {
        if (handle) {
          action_client_->async_cancel_goal(handle);
        }
        return;
      }
      // 空句柄是服务器拒绝，不是「还在等」。
      if (!handle) {
        goal_slot_.finish();
        if (on_result) {
          on_result(NavResult::REJECTED);
        }
        return;
      }
      current_goal_handle_ = handle;
      // 指标一打在「Nav2 接受」这一下，不打在 async_send_goal 返回的那一下。
      if (on_goal_sent_) {
        on_goal_sent_();
      }
      // 句柄还在飞的时候已经请求过取消。
      if (goal_slot_.handle_arrived_should_cancel()) {
        action_client_->async_cancel_goal(handle);
      }
    };

  options.result_callback =
    [this, on_result, seq](const GoalHandleNavigateToPose::WrappedResult & result) {
      // 超时或新 Goal 已经把序号往前推了，这个结果属于上一轮。
      if (seq != send_seq_) {
        return;
      }
      current_goal_handle_.reset();
      goal_slot_.finish();
      if (!on_result) {
        return;
      }

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          on_result(NavResult::SUCCEEDED);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          on_result(NavResult::CANCELED);
          break;
        case rclcpp_action::ResultCode::ABORTED:
        default:
          on_result(NavResult::ABORTED);
          break;
      }
    };

  action_client_->async_send_goal(goal_msg, options);
}

void NavAdapter::set_on_goal_sent(std::function<void()> callback)
{
  on_goal_sent_ = std::move(callback);
}

bool NavAdapter::cancel_current_goal()
{
  // Armed 时这里不发取消包，等 goal_response_callback 拿到句柄再取消。
  const GoalSlot::CancelAction action = goal_slot_.request_cancel();
  if (action == GoalSlot::CancelAction::None) {
    return false;
  }
  if (action == GoalSlot::CancelAction::CancelHandle && current_goal_handle_ && action_client_) {
    action_client_->async_cancel_goal(current_goal_handle_);
  }
  return true;
}

rclcpp::Time NavAdapter::now() const
{
  if (node_) {
    return node_->now();
  }
  return rclcpp::Time(0, 0, RCL_ROS_TIME);
}

}  // namespace sitl_adapters
