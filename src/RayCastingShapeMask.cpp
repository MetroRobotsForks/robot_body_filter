// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/* HACK HACK HACK */
/* We want to subclass ShapeMask and use its private members. */
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private protected  // NOLINT
#include <moveit/point_containment_filter/shape_mask.hpp>
#undef private
/* HACK END HACK */

#include <list>

#include <cras_cpp_common/cloud.hpp>
#include <geometric_shapes/body_operations.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <robot_body_filter/RayCastingShapeMask.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robot_body_filter {

struct RayCastingShapeMask::RayCastingShapeMaskPIMPL {
  std::set<SeeShape, SortBodies> bodies_for_contains_test;
  std::set<SeeShape, SortBodies> bodies_for_shadow_test;
  std::set<SeeShape, SortBodies> bodies_for_bsphere;
  std::set<SeeShape, SortBodies> bodies_for_bbox;
  std::map<point_containment_filter::ShapeHandle, std::string> shape_names;

  using MultiBodyTuple = std::tuple<MultiShapeHandle, SeeShape, SeeShape, SeeShape, SeeShape>;
  std::list<MultiBodyTuple> multi_bodies;

  std::map<point_containment_filter::ShapeHandle, MultiShapeHandle> shapes_to_multi_shapes;

  bodies::BoundingSphere bounding_sphere;
  bodies::BoundingSphere bounding_sphere_for_contains_test;
};

RayCastingShapeMask::RayCastingShapeMask(
  rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr& clock,
  const TransformCallback& transform_callback,
  const double min_sensor_dist,
  const double max_sensor_dist,
  const bool do_clipping,
  const bool do_contains_test,
  const bool do_shadow_test,
  const double max_shadow_dist)
  : ShapeMask(transform_callback),
    logger_(logger.get_child("ray_casting_shape_mask")),
    clock_(clock),
    min_sensor_dist_(min_sensor_dist),
    max_sensor_dist_(max_sensor_dist),
    max_shadow_dist_(max_shadow_dist),
    do_clipping_(do_clipping),
    do_contains_test_(do_contains_test),
    do_shadow_test_(do_shadow_test) {
  this->data_ = std::make_unique<RayCastingShapeMaskPIMPL>();
}

RayCastingShapeMask::~RayCastingShapeMask() = default;

std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere>
RayCastingShapeMask::getBoundingSpheres() const {
  std::lock_guard _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere> map;

  size_t body_index = 0, sphere_index = 0;
  for (const auto& see_shape : this->bodies_) {
    if (sphere_index >= this->bspheres_body_indices_.size()) {
      break;
    }
    if (this->bspheres_body_indices_[sphere_index] == body_index) {
      map[see_shape.handle] = this->bspheres_[sphere_index];
      sphere_index++;
    }
    body_index++;
  }

  return map;
}

std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere>
RayCastingShapeMask::getBoundingSpheresForContainsTest() const {
  std::lock_guard _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere> map;

  size_t body_index = 0, sphere_index = 0;
  for (const auto& see_shape : this->bodies_) {
    if (sphere_index >= this->bspheres_for_contains_test_body_indices_.size()) {
      break;
    }
    if (this->bspheres_for_contains_test_body_indices_[sphere_index] == body_index) {
      map[see_shape.handle] = this->bspheres_for_contains_test_[sphere_index];
      sphere_index++;
    }
    body_index++;
  }

  return map;
}

bodies::BoundingSphere RayCastingShapeMask::getBoundingSphere() const {
  std::lock_guard _(this->shapes_lock_);
  return this->getBoundingSphereNoLock();
}

bodies::BoundingSphere RayCastingShapeMask::getBoundingSphereNoLock() const {
  return this->data_->bounding_sphere;
}

bodies::BoundingSphere RayCastingShapeMask::getBoundingSphereForContainsTestNoLock() const {
  return this->data_->bounding_sphere_for_contains_test;
}

void RayCastingShapeMask::updateBodyPoses() {
  std::lock_guard _(this->shapes_lock_);
  this->updateBodyPosesNoLock();
}

void RayCastingShapeMask::updateBodyPosesNoLock() {
  auto transform = Eigen::Isometry3d::Identity();
  point_containment_filter::ShapeHandle contains_handle;
  bodies::Body* contains_body;
  bodies::Body* shadow_body;
  bodies::Body* bsphere_body;
  bodies::Body* bbox_body;
  std::set<const bodies::Body*> valid_bodies;

  for (const auto& multi_body : this->data_->multi_bodies) {
    contains_handle = std::get<0>(multi_body).contains;
    contains_body = std::get<1>(multi_body).body;
    shadow_body = std::get<2>(multi_body).body;
    bsphere_body = std::get<3>(multi_body).body;
    bbox_body = std::get<4>(multi_body).body;

    if (this->transform_callback_(contains_handle, transform)) {
      contains_body->setPose(transform);
      valid_bodies.insert(contains_body);

      if (contains_body != shadow_body) {
        shadow_body->setPose(transform);
        valid_bodies.insert(shadow_body);
      }

      if (bsphere_body != contains_body && bsphere_body != shadow_body) {
        bsphere_body->setPose(transform);
        valid_bodies.insert(bsphere_body);
      }

      if (bbox_body != contains_body && bbox_body != shadow_body && bbox_body != bsphere_body) {
        bbox_body->setPose(transform);
        valid_bodies.insert(bbox_body);
      }
    } else {
      if (contains_body == nullptr) {
        RCLCPP_ERROR_STREAM_THROTTLE(
          logger_, *clock_, 3, "Missing transform for shape with handle " << contains_handle << " without a body");
      } else {
        std::string name;
        if (this->data_->shape_names.find(contains_handle) != this->data_->shape_names.end()) {
          name = this->data_->shape_names.at(contains_handle);
        }

        if (name.empty()) {
          RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, *clock_, 3,
            "Missing transform for shape " << contains_body->getType() << " with handle " << contains_handle);
        } else {
          RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, *clock_, 3, "Missing transform for shape " << name << " (" << contains_body->getType() << ")");
        }
      }
    }
  }

  // TODO: prevent frequent reallocations of memory
  this->bspheres_.resize(this->bodies_.size());
  this->bspheres_body_indices_.resize(this->bodies_.size());
  this->bspheres_for_contains_test_.resize(this->bodies_.size());
  this->bspheres_for_contains_test_body_indices_.resize(this->bodies_.size());

  size_t body_idx = 0, valid_body_idx = 0, valid_contains_test_idx = 0;
  for (const auto& see_shape : this->bodies_) {
    const auto& shape_handle = see_shape.handle;
    const auto body = see_shape.body;
    const auto& multi_shape = this->data_->shapes_to_multi_shapes.at(shape_handle);
    if (valid_bodies.find(body) != valid_bodies.end()) {
      this->bspheres_body_indices_[valid_body_idx] = body_idx;
      body->computeBoundingSphere(this->bspheres_[valid_body_idx]);

      if (shape_handle == multi_shape.contains &&
          this->ignore_in_contains_test_.find(multi_shape) == this->ignore_in_contains_test_.end()) {
        this->bspheres_for_contains_test_body_indices_[valid_contains_test_idx] = body_idx;
        this->bspheres_for_contains_test_[valid_contains_test_idx] = this->bspheres_[valid_body_idx];
        valid_contains_test_idx++;
      }

      valid_body_idx++;
    }

    body_idx++;
  }
  this->bspheres_.resize(valid_body_idx);
  this->bspheres_for_contains_test_.resize(valid_contains_test_idx);
  this->bspheres_body_indices_.resize(valid_body_idx);
  this->bspheres_for_contains_test_body_indices_.resize(valid_contains_test_idx);

  bodies::mergeBoundingSpheres(this->bspheres_, this->data_->bounding_sphere);
  bodies::mergeBoundingSpheres(this->bspheres_for_contains_test_, this->data_->bounding_sphere_for_contains_test);
}

void RayCastingShapeMask::maskContainmentAndShadows(
    const sensor_msgs::msg::PointCloud2& data, std::vector<MaskValue>& mask, const Eigen::Vector3d& sensor_pos) {
  std::lock_guard _(this->shapes_lock_);

  const auto np = cras::numPoints(data);
  mask.resize(np);

  this->updateBodyPosesNoLock();

  // we now decide which points we keep
  cras::CloudConstIter iter_x(data, "x");
  cras::CloudConstIter iter_y(data, "y");
  cras::CloudConstIter iter_z(data, "z");

  // Cloud iterators are not incremented in the for loop, because of the pragma
  // Comment out below parallelization as it can result in very high CPU consumption
  // #pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < np; ++i) {
    const Eigen::Vector3d pt(static_cast<double>(*(iter_x + i)),
                             static_cast<double>(*(iter_y + i)),
                             static_cast<double>(*(iter_z + i)));
    this->classifyPointNoLock(pt, mask[i], sensor_pos);
  }
}

void RayCastingShapeMask::maskContainmentAndShadows(
  const Eigen::Vector3f& data, MaskValue& mask, const Eigen::Vector3d& sensor_pos, const bool update_body_poses) {
  if (data.hasNaN()) {
    mask = MaskValue::OUTSIDE;
    return;
  }

  std::lock_guard _(this->shapes_lock_);

  if (update_body_poses) {
    this->updateBodyPosesNoLock();
  }

  this->classifyPointNoLock(data.cast<double>(), mask, sensor_pos);
}

void RayCastingShapeMask::classifyPointNoLock(
    const Eigen::Vector3d& data, MaskValue& mask, const Eigen::Vector3d& sensor_pos) const {
  mask = MaskValue::OUTSIDE;

  if (data.hasNaN()) {
    return;
  }

  // direction from measured point to sensor
  Eigen::Vector3d dir(sensor_pos - data);
  const auto distance = dir.norm();

  if (this->do_clipping_ && (distance < this->min_sensor_dist_ ||
    (this->max_sensor_dist_ > 0.0 && distance > this->max_sensor_dist_))) {
    // check if the point is inside measurement range
    mask = MaskValue::CLIP;
    return;
  }

  // check if it is inside the scaled body
  const auto radius_squared = pow(this->data_->bounding_sphere_for_contains_test.radius, 2);
  if (this->do_contains_test_ &&
      (this->data_->bounding_sphere_for_contains_test.center - data).squaredNorm() < radius_squared) {
    for (const auto& see_shape : this->data_->bodies_for_contains_test) {
      if (see_shape.body->containsPoint(data)) {
        mask = MaskValue::INSIDE;
        return;
      }
    }
  }

  if (this->do_shadow_test_ && (this->max_shadow_dist_ <= 0.0 || distance <= this->max_shadow_dist_)) {
    // point is not inside the robot, check if it is a shadow point
    dir /= distance;
    EigenSTL::vector_Vector3d intersections;
    for (const auto& see_shape : this->data_->bodies_for_shadow_test) {
      // get the 1st intersection of ray pt->sensor
      intersections.clear();  // intersectsRay doesn't clear the vector...
      if (see_shape.body->intersectsRay(data, dir, &intersections, 1)) {
        // is the intersection between point and sensor?
        if (dir.dot(sensor_pos - intersections[0]) >= 0.0) {
          mask = MaskValue::SHADOW;
          return;
        }
      }
    }
  }
}

void RayCastingShapeMask::setIgnoreInContainsTest(
    std::unordered_set<MultiShapeHandle> ignore_in_contains_test, const bool update_internal_structures) {
  this->ignore_in_contains_test_ = std::move(ignore_in_contains_test);
  if (update_internal_structures) {
    this->updateInternalShapeLists();
  }
}

void RayCastingShapeMask::setIgnoreInShadowTest(
    std::unordered_set<MultiShapeHandle> ignore_in_shadow_test, const bool update_internal_structures) {
  this->ignore_in_shadow_test_ = std::move(ignore_in_shadow_test);
  if (update_internal_structures) {
    this->updateInternalShapeLists();
  }
}

MultiShapeHandle RayCastingShapeMask::addShape(
    const shapes::ShapeConstPtr& shape, const double scale, const double padding, const bool update_internal_structures,
    const std::string& name) {
  return this->addShape(
    shape, scale, padding, scale, padding, scale, padding, scale, padding, update_internal_structures, name);
}

MultiShapeHandle RayCastingShapeMask::addShape(
    const shapes::ShapeConstPtr& shape, const double contains_scale, const double contains_padding,
    const double shadow_scale, const double shadow_padding, const double bsphere_scale, const double bsphere_padding,
    const double bbox_scale, const double bbox_padding, const bool update_internal_structures,
    const std::string& name) {
  MultiShapeHandle result;

  result.contains = ShapeMask::addShape(shape, contains_scale, contains_padding);
  this->data_->shape_names[result.contains] = name;

  result.shadow = result.contains;
  if (std::abs(contains_scale - shadow_scale) > 1e-6 || std::abs(contains_padding - shadow_padding) > 1e-6) {
    result.shadow = ShapeMask::addShape(shape, shadow_scale, shadow_padding);
    this->data_->shape_names[result.shadow] = name;
  }

  result.bsphere = result.contains;
  if (std::abs(contains_scale - bsphere_scale) > 1e-6 || std::abs(contains_padding - bsphere_padding) > 1e-6) {
    result.bsphere = ShapeMask::addShape(shape, bsphere_scale, bsphere_padding);
    this->data_->shape_names[result.bsphere] = name;
  }

  result.bbox = result.contains;
  if (std::abs(contains_scale - bbox_scale) > 1e-6 || std::abs(contains_padding - bbox_padding) > 1e-6) {
    result.bbox = ShapeMask::addShape(shape, bbox_scale, bbox_padding);
    this->data_->shape_names[result.bbox] = name;
  }

  auto contains_see_shape = *this->used_handles_.at(result.contains);
  auto shadow_see_shape = *this->used_handles_.at(result.shadow);
  auto bsphere_see_shape = *this->used_handles_.at(result.bsphere);
  auto bbox_see_shape = *this->used_handles_.at(result.bbox);
  this->data_->multi_bodies.emplace_back(
    result, contains_see_shape, shadow_see_shape, bsphere_see_shape, bbox_see_shape);

  this->data_->shapes_to_multi_shapes[result.contains] = result;
  this->data_->shapes_to_multi_shapes[result.shadow] = result;
  this->data_->shapes_to_multi_shapes[result.bsphere] = result;
  this->data_->shapes_to_multi_shapes[result.bbox] = result;

  if (update_internal_structures) {
    this->updateInternalShapeLists();
  }
  return result;
}

void RayCastingShapeMask::removeShape(const MultiShapeHandle& handle, const bool update_internal_structures) {
  this->data_->multi_bodies.remove_if([handle](const RayCastingShapeMaskPIMPL::MultiBodyTuple& t) {
    return std::get<0>(t) == handle;
  });

  ShapeMask::removeShape(handle.contains);
  this->data_->shape_names.erase(handle.contains);
  this->data_->shapes_to_multi_shapes.erase(handle.contains);

  if (handle.contains != handle.shadow) {
    ShapeMask::removeShape(handle.shadow);
    this->data_->shape_names.erase(handle.shadow);
    this->data_->shapes_to_multi_shapes.erase(handle.shadow);
  }

  if (handle.contains != handle.bsphere) {
    ShapeMask::removeShape(handle.bsphere);
    this->data_->shape_names.erase(handle.bsphere);
    this->data_->shapes_to_multi_shapes.erase(handle.bsphere);
  }

  if (handle.contains != handle.bbox) {
    ShapeMask::removeShape(handle.bbox);
    this->data_->shape_names.erase(handle.bbox);
    this->data_->shapes_to_multi_shapes.erase(handle.bbox);
  }

  if (update_internal_structures) {
    this->updateInternalShapeLists();
  }
}

void RayCastingShapeMask::setTransformCallback(const ShapeMask::TransformCallback& transform_callback) {
  ShapeMask::setTransformCallback(transform_callback);
}

void RayCastingShapeMask::updateInternalShapeLists() {
  std::lock_guard _(this->shapes_lock_);

  this->data_->bodies_for_contains_test.clear();
  this->data_->bodies_for_shadow_test.clear();
  this->data_->bodies_for_bsphere.clear();
  this->data_->bodies_for_bbox.clear();

  for (const auto& multiBody : this->data_->multi_bodies) {
    const auto handle = std::get<0>(multiBody);
    const auto contains_see_shape = std::get<1>(multiBody);
    const auto shadow_see_shape = std::get<2>(multiBody);
    const auto bsphere_see_shape = std::get<3>(multiBody);
    const auto bbox_see_shape = std::get<4>(multiBody);

    if (this->ignore_in_contains_test_.find(handle) == this->ignore_in_contains_test_.end()) {
      this->data_->bodies_for_contains_test.insert(contains_see_shape);
    }
    if (this->ignore_in_shadow_test_.find(handle) == this->ignore_in_shadow_test_.end()) {
      this->data_->bodies_for_shadow_test.insert(shadow_see_shape);
    }
    if (this->ignore_in_bsphere_.find(handle) == this->ignore_in_bsphere_.end()) {
      this->data_->bodies_for_bsphere.insert(bsphere_see_shape);
    }
    if (this->ignore_in_bbox_.find(handle) == this->ignore_in_bbox_.end()) {
      this->data_->bodies_for_bbox.insert(bbox_see_shape);
    }
  }
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*> RayCastingShapeMask::getBodies() const {
  std::lock_guard _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& see_shape : this->bodies_) {
    result[see_shape.handle] = see_shape.body;
  }

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForContainsTest() const {
  std::lock_guard _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& see_shape : this->data_->bodies_for_contains_test) {
    result[see_shape.handle] = see_shape.body;
  }

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForShadowTest() const {
  std::lock_guard _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& see_shape : this->data_->bodies_for_shadow_test) {
    result[see_shape.handle] = see_shape.body;
  }

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForBoundingSphere() const {
  std::lock_guard _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& see_shape : this->data_->bodies_for_bsphere) {
    result[see_shape.handle] = see_shape.body;
  }

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForBoundingBox() const {
  std::lock_guard _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& see_shape : this->data_->bodies_for_bbox) {
    result[see_shape.handle] = see_shape.body;
  }

  return result;
}

bool MultiShapeHandle::operator==(const MultiShapeHandle& other) const {
  return this->contains == other.contains && this->shadow == other.shadow &&
    this->bsphere == other.bsphere && this->bbox == other.bbox;
}

bool MultiShapeHandle::operator!=(const MultiShapeHandle& other) const {
  return !(*this == other);
}

}  // namespace robot_body_filter
