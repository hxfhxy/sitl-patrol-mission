#include "sitl_adapters/sensor_health.hpp"

#include <gtest/gtest.h>

TEST(ScanFreshness, NeverMarkedIsUnhealthy)
{
  sitl_adapters::ScanFreshness freshness;
  EXPECT_FALSE(freshness.healthy(1'000'000'000, 1'000'000'000));
}

TEST(ScanFreshness, FreshArrivalIsHealthy)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(1'000'000'000);
  EXPECT_TRUE(freshness.healthy(1'200'000'000, 1'000'000'000));
}

TEST(ScanFreshness, AgeEqualToLimitIsHealthy)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(1'000'000'000);
  EXPECT_TRUE(freshness.healthy(2'000'000'000, 1'000'000'000));
}

TEST(ScanFreshness, StaleArrivalIsUnhealthy)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(1'000'000'000);
  EXPECT_FALSE(freshness.healthy(2'000'000'001, 1'000'000'000));
}

TEST(ScanFreshness, ClockBackwardsIsUnhealthyUntilRemarked)
{
  sitl_adapters::ScanFreshness freshness;
  freshness.mark(5'000'000'000);
  EXPECT_FALSE(freshness.healthy(1'000'000'000, 1'000'000'000));
}
