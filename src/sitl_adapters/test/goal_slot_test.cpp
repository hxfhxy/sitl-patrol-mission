#include "sitl_adapters/goal_slot.hpp"

#include <gtest/gtest.h>

namespace {

using sitl_adapters::GoalSlot;

// 没有在飞的 Goal 时，取消是空操作。
TEST(GoalSlot, IdleCancelDoesNothing)
{
  GoalSlot slot;
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::None);
}

// 句柄还没到：先记一笔，句柄到达时再取消。
TEST(GoalSlot, CancelBeforeHandleIsArmed)
{
  GoalSlot slot;
  slot.begin_send();
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::Armed);
  EXPECT_TRUE(slot.handle_arrived_should_cancel());
}

// 句柄已经在：调用方现在就 async_cancel。
TEST(GoalSlot, CancelAfterHandleAsksToCancel)
{
  GoalSlot slot;
  slot.begin_send();
  EXPECT_FALSE(slot.handle_arrived_should_cancel());
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::CancelHandle);
}

// 结果到了之后槽回到空闲，再取消不再打到这个 Goal。
TEST(GoalSlot, FinishReturnsToIdle)
{
  GoalSlot slot;
  slot.begin_send();
  slot.handle_arrived_should_cancel();
  slot.finish();
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::None);
}

// 新的一次发送清掉上一轮的取消标记，避免打到重试的新 Goal。
TEST(GoalSlot, NewSendClearsAnArmedCancel)
{
  GoalSlot slot;
  slot.begin_send();
  EXPECT_EQ(slot.request_cancel(), GoalSlot::CancelAction::Armed);
  slot.begin_send();
  EXPECT_FALSE(slot.handle_arrived_should_cancel());
}

}  // namespace
