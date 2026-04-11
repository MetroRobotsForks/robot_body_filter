#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/aabb.h>

#include <geometric_shapes/obb.h>

namespace bodies
{

typedef bodies::AABB AxisAlignedBoundingBox;
typedef bodies::OBB OrientedBoundingBox;

/** \brief Compute AABB for the body at different pose. Can't use setPose() because we want `body` to be const. */
void computeBoundingBoxAt(const bodies::Body* body, AxisAlignedBoundingBox& bbox, const Eigen::Isometry3d& pose);

}
