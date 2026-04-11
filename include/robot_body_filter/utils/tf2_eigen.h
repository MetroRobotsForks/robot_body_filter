#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point32.hpp>

namespace tf2 {
void toMsg(const Eigen::Vector3d& in, geometry_msgs::msg::Point32& out);
}
