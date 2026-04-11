// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"
#include <robot_body_filter/utils/time_utils.hpp>
#include <rcl/time.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

using namespace robot_body_filter;
using namespace rclcpp;

TEST(TimeUtils, TimeNotInitialized)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  // TODO figure out uninit case
  // ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock_ptr->get_clock_handle()));
  // time hasn't been initialized
  EXPECT_EQ(remainingTime(clock_ptr, Time(99, 0, RCL_ROS_TIME), Duration::from_seconds(2)).seconds(), 0.0);
}

TEST(TimeUtils, RemainingTimeDuration)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock_ptr->get_clock_handle()));
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock_ptr->get_clock_handle(), 100'000'000'000));

  EXPECT_EQ(9.0, remainingTime(clock_ptr, Time(99, 0, RCL_ROS_TIME), Duration::from_seconds(10)).seconds());
  EXPECT_EQ(1.0, remainingTime(clock_ptr, Time(99, 0, RCL_ROS_TIME), Duration::from_seconds(2)).seconds());
  EXPECT_EQ(0.0, remainingTime(clock_ptr, Time(90, 0, RCL_ROS_TIME), Duration::from_seconds(2)).seconds()); // time's up
}

TEST(TimeUtils, RemainingTimeDouble)
{
  auto clock_ptr = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock_ptr->get_clock_handle()));
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock_ptr->get_clock_handle(), 100'000'000'000));

  EXPECT_EQ(9.0, remainingTime(clock_ptr, Time(99, 0, RCL_ROS_TIME), 10.0).seconds());
  EXPECT_EQ(1.0, remainingTime(clock_ptr, Time(99, 0, RCL_ROS_TIME), 2.0).seconds());
  EXPECT_EQ(0.0, remainingTime(clock_ptr, Time(90, 0, RCL_ROS_TIME), 2.0).seconds()); // time's up
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
