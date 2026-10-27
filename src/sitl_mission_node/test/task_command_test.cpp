#include "sitl_mission_node/task_command.hpp"

#include <gtest/gtest.h>

// 大小写和首尾空白都收成 start / cancel。
TEST(TaskCommand, ParsesStartAndCancel)
{
  using sitl_mission_node::TaskCommand;
  using sitl_mission_node::parse_task_command;
  EXPECT_EQ(parse_task_command("start"), TaskCommand::Start);
  EXPECT_EQ(parse_task_command("  START\n"), TaskCommand::Start);
  EXPECT_EQ(parse_task_command("cancel"), TaskCommand::Cancel);
  EXPECT_EQ(parse_task_command(" Cancel "), TaskCommand::Cancel);
}

// 0 一直转。负数在节点里先改成 1，这里本身不把负数当成完成。
TEST(PatrolLoops, CountsCompletedLaps)
{
  using sitl_mission_node::patrol_laps_complete;
  EXPECT_FALSE(patrol_laps_complete(1, 0));
  EXPECT_TRUE(patrol_laps_complete(1, 1));
  EXPECT_FALSE(patrol_laps_complete(2, 1));
  EXPECT_TRUE(patrol_laps_complete(2, 2));
  EXPECT_FALSE(patrol_laps_complete(0, 5));
  EXPECT_FALSE(patrol_laps_complete(-1, 1));
}

// 空串、多出来的词都不当成命令。
TEST(TaskCommand, IgnoresAnythingElse)
{
  using sitl_mission_node::parse_task_command;
  using sitl_mission_node::TaskCommand;
  EXPECT_EQ(parse_task_command(""), TaskCommand::Ignore);
  EXPECT_EQ(parse_task_command("patrol"), TaskCommand::Ignore);
  EXPECT_EQ(parse_task_command("start now"), TaskCommand::Ignore);
}
