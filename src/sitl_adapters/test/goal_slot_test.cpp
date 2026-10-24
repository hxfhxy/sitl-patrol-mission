#include "sitl_adapters/goal_slot.hpp"

#include <gtest/gtest.h>

namespace {

using sitl_adapters::GoalSlot;

TEST(GoalSlot, IdleCancelDoesNothing)
{
  GoalSlot slot;
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::None);
}

TEST(GoalSlot, CancelBeforeHandleIsArmed)
{
  GoalSlot slot;
  slot.begin_send();
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::Armed);
  EXPECT_TRUE(slot.handle_arrived_should_cancel());
}

TEST(GoalSlot, CancelAfterHandleAsksToCancel)
{
  GoalSlot slot;
  slot.begin_send();
  EXPECT_FALSE(slot.handle_arrived_should_cancel());
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::CancelHandle);
}

TEST(GoalSlot, FinishReturnsToIdle)
{
  GoalSlot slot;
  slot.begin_send();
  slot.handle_arrived_should_cancel();
  slot.finish();
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::None);
}

TEST(GoalSlot, NewSendClearsAnArmedCancel)
{
  GoalSlot slot;
  slot.begin_send();
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::Armed);
  slot.begin_send();
  EXPECT_FALSE(slot.handle_arrived_should_cancel());
}

}  // namespace
