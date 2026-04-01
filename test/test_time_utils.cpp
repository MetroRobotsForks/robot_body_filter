#include "gtest/gtest.h"
#include <robot_body_filter/utils/time_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

using namespace robot_body_filter;
using namespace rclcpp;

TEST(TimeUtils, TimeNotInitialized)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>();
  EXPECT_EQ(remainingTime(clock_ptr, Time(99, 0), Duration::from_seconds(2)).seconds(), 0.0); // time hasn't been initialized
}

TEST(TimeUtils, RemainingTimeDuration)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>();
  Time::init();
  Time::setNow(Time(100));

  EXPECT_EQ(9.0, remainingTime(clock_ptr, Time(99, 0), Duration::from_seconds(10)).seconds());
  EXPECT_EQ(1.0, remainingTime(clock_ptr, Time(99, 0), Duration::from_seconds(2)).seconds());
  EXPECT_EQ(0.0, remainingTime(clock_ptr, Time(90, 0), Duration::from_seconds(2)).seconds()); // time's up
}

TEST(TimeUtils, RemainingTimeDouble)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>();
  Time::init();
  Time::setNow(Time(100));

  EXPECT_EQ(9.0, remainingTime(clock_ptr, Time(99, 0), 10.0).seconds());
  EXPECT_EQ(1.0, remainingTime(clock_ptr, Time(99, 0), 2.0).seconds());
  EXPECT_EQ(0.0, remainingTime(clock_ptr, Time(90, 0), 2.0).seconds()); // time's up
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
