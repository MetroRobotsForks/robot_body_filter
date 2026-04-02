#include "gtest/gtest.h"
#include <robot_body_filter/utils/time_utils.hpp>
#include <ros/duration.h>
#include <ros/time.h>

using namespace robot_body_filter;
using namespace rclcpp;

TEST(TimeUtils, TimeNotInitialized)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>();
  EXPECT_EQ(remainingTime(clock_ptr, Time(99), Duration(2)).toSec(), 0.0); // time hasn't been initialized
}

TEST(TimeUtils, RemainingTimeDuration)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>();
  Time::init();
  Time::setNow(Time(100));

  EXPECT_EQ(9.0, remainingTime(clock_ptr, Time(99), Duration(10)).toSec());
  EXPECT_EQ(1.0, remainingTime(clock_ptr, Time(99), Duration(2)).toSec());
  EXPECT_EQ(0.0, remainingTime(clock_ptr, Time(90), Duration(2)).toSec()); // time's up
}

TEST(TimeUtils, RemainingTimeDouble)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>();
  Time::init();
  Time::setNow(Time(100));

  EXPECT_EQ(9.0, remainingTime(clock_ptr, Time(99), 10.0).toSec());
  EXPECT_EQ(1.0, remainingTime(clock_ptr, Time(99), 2.0).toSec());
  EXPECT_EQ(0.0, remainingTime(clock_ptr, Time(90), 2.0).toSec()); // time's up
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
