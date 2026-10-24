#include "sitl_mission_node/mission_node.hpp"

#include <iomanip>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/create_timer.hpp"
#include "sitl_mission_bt/bt_nodes.hpp"
#include "sitl_mission_node/mqtt_task_client.hpp"
#include "sitl_mission_node/task_command.hpp"

namespace sitl_mission_node {

MissionNode::MissionNode() : Node("mission_node")
{
  auto_start_ = declare_parameter<bool>("auto_start", true);
  patrol_loops_ = declare_parameter<int>("patrol_loops", 1);
  if (patrol_loops_ < 0) {
    RCLCPP_WARN(get_logger(), "patrol_loops=%d 无效，改用 1。", patrol_loops_);
    patrol_loops_ = 1;
  }
  const bool mqtt_enable = declare_parameter<bool>("mqtt_enable", false);
  const std::string mqtt_host = declare_parameter<std::string>("mqtt_host", "127.0.0.1");
  const int mqtt_port = declare_parameter<int>("mqtt_port", 1883);
  const std::string mqtt_topic = declare_parameter<std::string>("mqtt_topic", "sitl/patrol/task");
  const std::string mqtt_status_topic =
    declare_parameter<std::string>("mqtt_status_topic", "sitl/patrol/status");
  const std::string lifecycle_node =
    declare_parameter<std::string>("nav_lifecycle_node", "bt_navigator");

  nav_adapter_ = std::make_unique<sitl_adapters::NavAdapter>(this);
  estop_ = std::make_unique<sitl_adapters::EStop>(this);
  sensor_health_ = std::make_unique<sitl_adapters::SensorHealth>(this);
  nav_adapter_->set_on_goal_sent([this]() { note_goal_sent(); });

  BT::BehaviorTreeFactory factory;
  sitl_mission_bt::register_bt_nodes(
    factory, nav_adapter_.get(), estop_.get(), sensor_health_.get());
  const std::string bt_xml_path =
    ament_index_cpp::get_package_share_directory("sitl_mission_bt") + "/behavior_trees/patrol.xml";
  tree_ = factory.createTreeFromFile(bt_xml_path);
  RCLCPP_INFO(get_logger(), "已加载行为树: %s", bt_xml_path.c_str());

  cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/cancel_patrol", 10,
    [this](const std_msgs::msg::Empty::SharedPtr) { request_cancel(); });
  task_sub_ = create_subscription<std_msgs::msg::String>(
    "/patrol_task", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { enqueue_command(msg->data); });
  status_pub_ = create_publisher<std_msgs::msg::String>(
    "/patrol_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  lifecycle_client_ = create_client<lifecycle_msgs::srv::GetState>(lifecycle_node + "/get_state");

  poll_timer_ = create_wall_timer(
    std::chrono::milliseconds(200), [this]() { poll(); });

  if (mqtt_enable) {
    mqtt_ = std::make_unique<MqttTaskClient>(
      mqtt_host, mqtt_port, mqtt_topic, mqtt_status_topic,
      [this](const std::string & payload) { enqueue_command(payload); },
      [this](const std::string & line) { RCLCPP_INFO(get_logger(), "%s", line.c_str()); });
    mqtt_->start();
    RCLCPP_INFO(
      get_logger(), "任务入口: MQTT %s:%d 订阅 %s ，状态发到 %s。镜像话题 /patrol_task 和 /patrol_status",
      mqtt_host.c_str(), mqtt_port, mqtt_topic.c_str(), mqtt_status_topic.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "任务入口: /patrol_task（std_msgs/String, start|cancel）。MQTT 未启用。");
  }

  if (patrol_loops_ == 0) {
    RCLCPP_INFO(get_logger(), "patrol_loops=0，一圈成功后继续下一圈，直到取消或故障。");
  } else {
    RCLCPP_INFO(get_logger(), "patrol_loops=%d，走完后退出。", patrol_loops_);
  }

  if (auto_start_) {
    request_start(false);
  } else {
    RCLCPP_INFO(get_logger(), "auto_start=false，等待 start 后才连接 Nav2。");
  }
}

MissionNode::~MissionNode()
{
  if (poll_timer_) {
    poll_timer_->cancel();
  }
  if (bt_timer_) {
    bt_timer_->cancel();
  }
  mqtt_.reset();
}

void MissionNode::enqueue_command(const std::string & payload)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  commands_.push_back(payload);
}

void MissionNode::drain_commands()
{
  std::deque<std::string> local;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    local.swap(commands_);
  }
  for (const std::string & payload : local) {
    switch (parse_task_command(payload)) {
      case TaskCommand::Start:
        request_start(true);
        break;
      case TaskCommand::Cancel:
        request_cancel();
        break;
      case TaskCommand::Ignore:
        RCLCPP_WARN(get_logger(), "忽略任务文本: '%s'（只要 start 或 cancel）", payload.c_str());
        break;
    }
  }
}

void MissionNode::request_start(bool external)
{
  if (phase_ != Phase::WaitTask && phase_ != Phase::Fault && phase_ != Phase::Canceled) {
    RCLCPP_WARN(get_logger(), "已有任务在执行或正在等待 Nav2，忽略新的 start。");
    return;
  }
  if (bt_timer_) {
    bt_timer_->cancel();
  }
  tree_.haltTree();
  if (const auto blackboard = tree_.rootBlackboard()) {
    blackboard->set("fault_reason", std::string());
  }
  lifecycle_future_.reset();
  external_task_ = external;
  goal_metric_logged_ = false;
  laps_finished_ = 0;
  if (external) {
    task_arrival_ = now();
  }
  phase_ = Phase::Arming;
  arm_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  arm_announced_ = false;
  RCLCPP_INFO(
    get_logger(), "接受巡检任务（%s）。Nav2 未 Active 之前不会发 Goal。",
    external ? "外部任务" : "auto_start");
  publish_status("accepted");
}

void MissionNode::request_cancel()
{
  if (phase_ == Phase::Arming) {
    phase_ = Phase::Canceled;
    RCLCPP_WARN(get_logger(), "Nav2 尚未开始巡检，取消这次任务。节点保持存活。");
    publish_status("canceled");
    return;
  }
  if (phase_ != Phase::Running) {
    RCLCPP_WARN(get_logger(), "当前没有正在执行的巡检，忽略取消。");
    return;
  }
  RCLCPP_WARN(get_logger(), "取消巡检: 停止 tick 并 haltTree。节点保持存活。");
  if (bt_timer_) {
    bt_timer_->cancel();
  }
  tree_.haltTree();
  phase_ = Phase::Canceled;
  publish_status("canceled");
}

bool MissionNode::nav_is_active()
{
  if (!lifecycle_client_->service_is_ready()) {
    return false;
  }
  if (!lifecycle_future_) {
    lifecycle_future_ = lifecycle_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    return false;
  }
  if (lifecycle_future_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return false;
  }
  const auto response = lifecycle_future_->get();
  lifecycle_future_.reset();
  return response &&
         response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void MissionNode::poll()
{
  drain_commands();
  if (phase_ != Phase::Arming) {
    return;
  }
  if (std::chrono::steady_clock::now() > arm_deadline_) {
    RCLCPP_ERROR(get_logger(), "60 s 内 Nav2 没有进入 Active，放弃这次任务。");
    phase_ = Phase::Fault;
    publish_status("fault nav_inactive");
    return;
  }
  if (!arm_announced_) {
    RCLCPP_INFO(get_logger(), "正在等待 /bt_navigator Active 和 /navigate_to_pose ...");
    arm_announced_ = true;
  }
  if (!nav_is_active()) {
    return;
  }
  if (!nav_adapter_->wait_for_server(std::chrono::seconds(0))) {
    return;
  }
  begin_ticking();
}

void MissionNode::begin_ticking()
{
  // 没收到过雷达时记一次启动宽限。已经断流的话，这里不会把到达时刻改成现在。
  sensor_health_->mark_fresh();
  phase_ = Phase::Running;
  RCLCPP_INFO(get_logger(), "Nav2 已 Active，开始 10 Hz tick 巡检树。");
  publish_status("running");
  bt_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_nanoseconds(100LL * 1000LL * 1000LL),
    [this]() { tick_tree(); });
}

void MissionNode::tick_tree()
{
  if (phase_ != Phase::Running) {
    return;
  }
  const BT::NodeStatus status = tree_.tickRoot();
  if (status == BT::NodeStatus::SUCCESS) {
    laps_finished_ += 1;
    if (const auto blackboard = tree_.rootBlackboard()) {
      blackboard->set("fault_reason", std::string());
    }
    // tickRoot 在 SUCCESS 后把根设回 IDLE，下一拍会从 A 再走。不要 haltTree：
    // 点已经成功，再 halt 可能进 onHalted，把零速度锁存到下一圈。
    if (!patrol_laps_complete(patrol_loops_, laps_finished_)) {
      publish_status("lap " + std::to_string(laps_finished_));
      RCLCPP_INFO(get_logger(), "第 %d 圈完成，继续下一圈。", laps_finished_);
      return;
    }
    phase_ = Phase::Done;
    bt_timer_->cancel();
    if (patrol_loops_ == 1) {
      RCLCPP_INFO(get_logger(), "行为树返回 SUCCESS，三点巡检完成。");
    } else {
      RCLCPP_INFO(get_logger(), "第 %d 圈完成，巡检结束。", laps_finished_);
    }
    publish_status("succeeded");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::shutdown();
  } else if (status == BT::NodeStatus::FAILURE) {
    phase_ = Phase::Fault;
    bt_timer_->cancel();
    estop_->halt();
    std::string reason = "unknown";
    if (const auto blackboard = tree_.rootBlackboard()) {
      std::string stored;
      if (blackboard->get("fault_reason", stored) && !stored.empty()) {
        reason = stored;
      }
    }
    publish_status("fault " + reason);
    RCLCPP_ERROR(
      get_logger(),
      "[FAULT] %s。已锁存零速度，节点保持存活。再发 start 可以重新装订任务。",
      reason.c_str());
  }
}

void MissionNode::note_goal_sent()
{
  if (!external_task_ || goal_metric_logged_) {
    return;
  }
  goal_metric_logged_ = true;
  const double seconds = (now() - task_arrival_).seconds();
  std::ostringstream text;
  text.setf(std::ios::fixed);
  text << std::setprecision(3) << "goal_sent " << seconds;
  publish_status(text.str());
  RCLCPP_INFO(
    get_logger(), "指标一 任务到达→第一个 Goal: %.3f s（节点钟，仅外部任务）", seconds);
}

void MissionNode::publish_status(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  if (status_pub_) {
    status_pub_->publish(msg);
  }
  if (mqtt_) {
    mqtt_->publish_status(text);
  }
  RCLCPP_INFO(get_logger(), "[状态] %s", text.c_str());
}

}  // namespace sitl_mission_node

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sitl_mission_node::MissionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
