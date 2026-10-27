#include "sitl_adapters/sensor_health.hpp"

#include <gtest/gtest.h>

// 还没有到达时刻时不能算健康。
TEST(ScanFreshness, NeverMarkedIsUnhealthy)
{
  sitl_adapters::ScanFreshness freshness;
  EXPECT_FALSE(freshness.healthy(1'000'000'000, 1'000'000'000));
}

// 年龄在上限以内。
TEST(ScanFreshness, FreshArrivalIsHealthy)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(1'000'000'000);
  EXPECT_TRUE(freshness.healthy(1'200'000'000, 1'000'000'000));
}

// 刚好等于上限仍算健康，比较用的是 <=。
TEST(ScanFreshness, AgeEqualToLimitIsHealthy)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(1'000'000'000);
  EXPECT_TRUE(freshness.healthy(2'000'000'000, 1'000'000'000));
}

// 超过上限 1 ns 就不健康。
TEST(ScanFreshness, StaleArrivalIsUnhealthy)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(1'000'000'000);
  EXPECT_FALSE(freshness.healthy(2'000'000'001, 1'000'000'000));
}

// 现在比上次到达还早时不能相减。重新 mark 之后才恢复。
TEST(ScanFreshness, ClockBackwardsIsUnhealthyUntilRemarked)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(5'000'000'000);
  EXPECT_FALSE(freshness.healthy(1'000'000'000, 1'000'000'000));
}
