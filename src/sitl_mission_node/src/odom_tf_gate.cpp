#include <memory>
#include <mutex>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTfGate : public rclcpp::Node
{
public:
  using Odometry = nav_msgs::msg::Odometry;

  OdomTfGate()
  : Node("odom_tf_gate")
  {
    // stall 只停 TF。零速度不在这个节点发。
    declare_parameter<bool>("stall", false);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<Odometry>(
      "/odom", rclcpp::QoS(10).reliable().durability_volatile(),
      [this](const Odometry::SharedPtr msg) {
        // 回调只存最新一帧。发布在定时器里，两边都会碰 last_odom_。
        std::lock_guard<std::mutex> lock(mutex_);
        last_odom_ = msg;
      });

    // /clock 只有 10 Hz。TF 若仍盖 odom 的旧 stamp，now() 一跳就会报 Transform data too old。
    // 这里表示当前车位，stamp 用节点钟。scan 中继必须保留采集时刻，两条规则不要混。
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_nanoseconds(50LL * 1000LL * 1000LL),
      [this]() { on_timer(); });

    RCLCPP_INFO(
      get_logger(),
      "odom_tf_gate 已启动 | stall=%s | use_sim_time=%s",
      get_parameter("stall").as_bool() ? "true" : "false",
      get_parameter("use_sim_time").as_bool() ? "true" : "false");
  }

private:
  void on_timer()
  {
    // 停发之后 Gazebo 按最后一条 Twist 滑行。这里不补零速度。
    if (get_parameter("stall").as_bool()) {
      return;
    }

    Odometry::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = last_odom_;
    }
    if (!msg) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::mutex mutex_;
  Odometry::SharedPtr last_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfGate>());
  rclcpp::shutdown();
  return 0;
}
