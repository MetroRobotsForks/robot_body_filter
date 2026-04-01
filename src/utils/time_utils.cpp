#include "robot_body_filter/utils/time_utils.hpp"

namespace robot_body_filter {

rclcpp::Duration remainingTime(const rclcpp::Clock::SharedPtr& clock_ptr, const rclcpp::Time &query, const double timeout)
{
  ros::Time::waitForValid(ros::WallDuration().fromSec(timeout));
  if (!ros::Time::isValid()) {
    ROS_ERROR("ROS time is not yet initialized");
    return ros::Duration(0);
  }

  const auto passed = (clock_ptr->now() - query).seconds();
  return rclcpp::Duration::from_seconds(std::max(0.0, timeout - passed));
}

rclcpp::Duration remainingTime(const rclcpp::Clock::SharedPtr& clock_ptr, const rclcpp::Time &query,
                            const rclcpp::Duration &timeout)
{
  ros::Time::waitForValid(ros::WallDuration(timeout.sec, timeout.nsec));
  if (!ros::Time::isValid()) {
    ROS_ERROR("ROS time is not yet initialized");
    return ros::Duration(0);
  }

  const auto passed = clock_ptr->now() - query;
  const auto remaining = timeout - passed;
  return (remaining.seconds() >= 0) ? remaining : rclcpp::Duration::from_seconds(0);
}

};
