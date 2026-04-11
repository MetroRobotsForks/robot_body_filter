#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <urdf_model/types.h>
#include <geometric_shapes/shapes.h>

namespace robot_body_filter
{

/** \brief Construct a masking shape out of the given URDF geometry.
 *
 * Just a helper function to convert urdf::Geometry to the corresponding shapes::Shape.
 *
 * \param geometry The URDF geometry object to convert.
 */
shapes::ShapeConstPtr constructShape(const urdf::Geometry& geometry);

}
