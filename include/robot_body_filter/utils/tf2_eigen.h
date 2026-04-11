#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point32.hpp>

namespace tf2 {
void toMsg(const Eigen::Vector3d& in, geometry_msgs::msg::Point32& out);
}
