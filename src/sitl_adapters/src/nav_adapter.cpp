#include "sitl_adapters/nav_adapter.hpp"

#include <cmath>

namespace sitl_adapters {

NavAdapter::NavAdapter(rclcpp::Node * node)
: node_(node)
{
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
  goal_msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
  goal_msg.pose.pose.orientation.w = std::cos(yaw * 0.5);

  const uint32_t seq = ++send_seq_;
  current_goal_handle_.reset();
  goal_slot_.begin_send();

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  options.goal_response_callback =
    [this, on_result, seq](const GoalHandleNavigateToPose::SharedPtr & handle) {
      if (seq != send_seq_) {
        if (handle) {
          action_client_->async_cancel_goal(handle);
        }
        return;
      }
      if (!handle) {
        goal_slot_.finish();
        if (on_result) {
          on_result(NavResult::REJECTED);
        }
        return;
      }
      current_goal_handle_ = handle;
      if (on_goal_sent_) {
        on_goal_sent_();
      }
      if (goal_slot_.handle_arrived_should_cancel()) {
        action_client_->async_cancel_goal(handle);
      }
    };

  options.result_callback =
    [this, on_result, seq](const GoalHandleNavigateToPose::WrappedResult & result) {
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
