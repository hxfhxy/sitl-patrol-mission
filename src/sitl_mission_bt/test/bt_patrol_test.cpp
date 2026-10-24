#include "sitl_mission_bt/bt_nodes.hpp"

#include <gtest/gtest.h>

namespace {

class FakeNav : public sitl_adapters::NavAdapter {
public:
  enum class Mode { Hold, ImmediateSuccess };

  FakeNav() : NavAdapter(nullptr) {}

  bool wait_for_server(std::chrono::seconds) override { return true; }

  void send_goal(double, double, double, ResultCallback on_result) override
  {
    ++sends;
    inflight = true;
    callback = std::move(on_result);
    if (mode == Mode::ImmediateSuccess && callback) {
      inflight = false;
      callback(sitl_adapters::NavResult::SUCCEEDED);
    }
  }

  bool cancel_current_goal() override
  {
    ++cancels;
    inflight = false;
    return true;
  }

  rclcpp::Time now() const override { return rclcpp::Time(now_ns, RCL_ROS_TIME); }

  Mode mode{Mode::Hold};
  ResultCallback callback;
  int sends{0};
  int cancels{0};
  bool inflight{false};
  int64_t now_ns{1'000'000'000};
};

class FakeHealth : public sitl_adapters::SensorHealth {
public:
  FakeHealth() : SensorHealth(nullptr) {}

  sitl_adapters::ScanHealthStatus status(double) const override
  {
    return healthy ? sitl_adapters::ScanHealthStatus::Ok :
           sitl_adapters::ScanHealthStatus::Disabled;
  }
  void mark_fresh() override {}

  bool healthy{true};
};

BT::Tree make_tree(const std::string & xml, FakeNav & nav, FakeHealth & health)
{
  BT::BehaviorTreeFactory factory;
  sitl_mission_bt::register_bt_nodes(factory, &nav, nullptr, &health);
  return factory.createTreeFromText(xml);
}

const char * kOnePoint = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence>
      <IsScanHealthy max_age_sec="1.0"/>
      <RetryUntilSuccessful num_attempts="3">
        <NavigateTo name="wp_A" x="1.0" y="0.0" yaw="0.0" timeout_sec="1.0"/>
      </RetryUntilSuccessful>
    </ReactiveSequence>
  </BehaviorTree>
</root>
)";

}  // namespace

class BtPatrolTest : public ::testing::Test {
public:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(BtPatrolTest, SuccessCompletesOneGoal)
{
  FakeNav nav;
  nav.mode = FakeNav::Mode::ImmediateSuccess;
  FakeHealth health;
  auto tree = make_tree(kOnePoint, nav, health);

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  for (int i = 0; i < 6; ++i) {
    status = tree.tickRoot();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(nav.sends, 1);
  EXPECT_EQ(nav.cancels, 0);
}

TEST_F(BtPatrolTest, SuccessStartsTheNextLapWithoutCancel)
{
  FakeNav nav;
  nav.mode = FakeNav::Mode::ImmediateSuccess;
  FakeHealth health;
  auto tree = make_tree(kOnePoint, nav, health);

  auto finish_one = [&]() {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    for (int i = 0; i < 6; ++i) {
      status = tree.tickRoot();
      if (status == BT::NodeStatus::SUCCESS) {
        return status;
      }
    }
    return status;
  };

  EXPECT_EQ(finish_one(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(nav.sends, 1);
  EXPECT_EQ(finish_one(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(nav.sends, 2);
  EXPECT_EQ(nav.cancels, 0);
}

TEST_F(BtPatrolTest, SequenceRestartsFromTheFirstPoint)
{
  const char * xml = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence>
      <IsScanHealthy max_age_sec="1.0"/>
      <Sequence>
        <RetryUntilSuccessful num_attempts="3">
          <NavigateTo name="wp_A" x="1.0" y="0.0" yaw="0.0" timeout_sec="1.0"/>
        </RetryUntilSuccessful>
        <RetryUntilSuccessful num_attempts="3">
          <NavigateTo name="wp_B" x="1.5" y="1.5" yaw="1.57" timeout_sec="1.0"/>
        </RetryUntilSuccessful>
      </Sequence>
    </ReactiveSequence>
  </BehaviorTree>
</root>
)";
  FakeNav nav;
  nav.mode = FakeNav::Mode::ImmediateSuccess;
  FakeHealth health;
  auto tree = make_tree(xml, nav, health);

  auto finish_one = [&]() {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    for (int i = 0; i < 8; ++i) {
      status = tree.tickRoot();
      if (status == BT::NodeStatus::SUCCESS) {
        return status;
      }
    }
    return status;
  };

  EXPECT_EQ(finish_one(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(nav.sends, 2);
  EXPECT_EQ(finish_one(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(nav.sends, 4);
  EXPECT_EQ(nav.cancels, 0);
}

TEST_F(BtPatrolTest, TimeoutRetriesThenFails)
{
  FakeNav nav;
  FakeHealth health;
  auto tree = make_tree(kOnePoint, nav, health);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 12 && status == BT::NodeStatus::RUNNING; ++i) {
    status = tree.tickRoot();
    if (status == BT::NodeStatus::RUNNING && nav.inflight) {
      nav.now_ns += 2'000'000'000;
    }
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(nav.sends, 3);
  EXPECT_GE(nav.cancels, 3);
}

TEST_F(BtPatrolTest, StaleResultDoesNotFinishTheNextAttempt)
{
  FakeNav nav;
  FakeHealth health;
  auto tree = make_tree(kOnePoint, nav, health);

  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  auto stale = nav.callback;
  nav.now_ns += 2'000'000'000;
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(nav.sends, 2);
  ASSERT_TRUE(static_cast<bool>(stale));
  stale(sitl_adapters::NavResult::SUCCEEDED);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(nav.sends, 2);
}

TEST_F(BtPatrolTest, ScanLossFaultsAndCancels)
{
  FakeNav nav;
  FakeHealth health;
  auto tree = make_tree(kOnePoint, nav, health);

  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(nav.sends, 1);
  health.healthy = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_GE(nav.cancels, 1);

  std::string reason;
  ASSERT_TRUE(tree.rootBlackboard()->get("fault_reason", reason));
  EXPECT_EQ(reason, "scan_disabled");
  tree.rootBlackboard()->set("fault_reason", std::string());
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  ASSERT_TRUE(tree.rootBlackboard()->get("fault_reason", reason));
  EXPECT_EQ(reason, "scan_disabled");
}
