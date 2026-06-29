// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <optional>
#include <utility>

#include <cras_cpp_common/time_utils.hpp>
#include <robot_body_filter/TfFramesWatchdog.h>

namespace robot_body_filter {
TFFramesWatchdog::TFFramesWatchdog(
  rclcpp::Logger logger, const rclcpp::Clock::SharedPtr& clock,
  std::string robot_frame, std::set<std::string> monitored_frames,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer, rclcpp::Duration unreachable_tf_lookup_timeout,
  rclcpp::Rate::SharedPtr unreachable_frames_check_rate)
  : robot_frame_(std::move(robot_frame)), monitored_frames_(std::move(monitored_frames)),
    tf_buffer_(std::move(tf_buffer)), unreachable_tf_lookup_timeout_(std::move(unreachable_tf_lookup_timeout)),
    unreachable_frames_check_rate_(std::move(unreachable_frames_check_rate)),
    logger_(logger.get_child("tf_frames_watchdog")), clock_(clock) {
  if (unreachable_frames_check_rate_ == nullptr) {
    unreachable_frames_check_rate_ = std::make_shared<rclcpp::Rate>(1.0, clock_);
  }
}

void TFFramesWatchdog::start() {
  this->should_stop_ = false;
  this->this_thread_ = std::thread(&TFFramesWatchdog::run, this);
  this->unpause();
}

void TFFramesWatchdog::run() {
  this->started_ = true;

  while (!this->should_stop_ && rclcpp::ok()) {
    if (!this->paused_) {  // the thread is paused_ when we want to change the stuff protected by frames_mutex_.
      this->searchForReachableFrames();
    }
    this->unreachable_frames_check_rate_->sleep();
  }
}

bool TFFramesWatchdog::isRunning() const {
  return this->started_;
}

void TFFramesWatchdog::searchForReachableFrames() {
  const rclcpp::Time time = clock_->now();

  // detect all unreachable frames
  // we can't join this loop with the following one, because we need to lock the frames_mutex_, but
  // canTransform can hang for pretty long, which would result in deadlocks

  std::set<std::string> unreachable_frames;
  {
    std::lock_guard<std::mutex> guard(this->frames_mutex_);
    std::set_difference(
      this->monitored_frames_.begin(), this->monitored_frames_.end(),
      this->reachable_frames_.begin(), this->reachable_frames_.end(),
      std::inserter(unreachable_frames, unreachable_frames.end()));
  }

  // now, the mutex is unlocked and we try to get transforms to all of the unreachable links... that
  // could take a while
  for (auto& frame : unreachable_frames) {
    if (this->paused_) {
      break;
    }
    std::string err;
    if (this->tf_buffer_->canTransform(this->robot_frame_, frame, time, this->unreachable_tf_lookup_timeout_, &err)) {
      this->markReachable(frame);
      RCLCPP_DEBUG(
        logger_, "TFFramesWatchdog (%s): Frame %s became reachable at %f.%li",
        this->robot_frame_.c_str(), frame.c_str(), time.seconds(), time.nanoseconds());
    } else {
      // TODO: Originally delayed-throttle
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 3, "TFFramesWatchdog (%s): Frame %s is not reachable! Cause: %s",
        this->robot_frame_.c_str(), frame.c_str(), err.c_str());
    }
  }
}

void TFFramesWatchdog::pause() {
  this->paused_ = true;
}

void TFFramesWatchdog::unpause() {
  this->paused_ = false;
}

void TFFramesWatchdog::stop() {
  RCLCPP_INFO(logger_, "Stopping TF watchdog.");
  this->should_stop_ = true;
  this->paused_ = true;

  if (this->started_ && this->this_thread_.joinable()) {
    this->this_thread_.join();  // segfaults without this line
  }
  RCLCPP_INFO(logger_, "TF watchdog stopped.");
}

void TFFramesWatchdog::clear() {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  monitored_frames_.clear();
  tf_buffer_->clear();
  reachable_frames_.clear();
}

std::optional<geometry_msgs::msg::TransformStamped> TFFramesWatchdog::lookupTransform(
  const std::string& frame, const rclcpp::Time& time, const rclcpp::Duration& timeout, std::string* errstr) {
  if (!this->started_) {
    throw std::runtime_error("TFFramesWatchdog has not been started.");
  }

  {
    std::lock_guard<std::mutex> guard(this->frames_mutex_);
    if (!this->isMonitoredNoLock(frame)) {
      RCLCPP_WARN(
        logger_, "TFFramesWatchdog (%s): Frame %s is not yet monitored, starting monitoring it.",
        this->robot_frame_.c_str(), frame.c_str());
      this->addMonitoredFrameNoLock(frame);
      // this lookup is lost, same as if the frame is unreachable
      return {};
    }

    // Return immediately for unreachable frames
    if (!this->isReachableNoLock(frame)) {
      return {};
    }
  }

  std::string tmp_errstr;
  if (errstr == nullptr) {
    errstr = &tmp_errstr;
  }

  if (!this->tf_buffer_->canTransform(
    this->robot_frame_, frame, time, cras::remainingTime(time, timeout, clock_), errstr)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3, "TFFramesWatchdog (%s): Frame %s became unreachable. Cause: %s",
      this->robot_frame_.c_str(), frame.c_str(), errstr->c_str());

    // if we couldn't get TF for this reachable frame, mark it unreachable
    this->markUnreachable(frame);
    return {};
  }

  try {
    return this->tf_buffer_->lookupTransform(
      this->robot_frame_, frame, time, cras::remainingTime(time, timeout, clock_));
  } catch (tf2::LookupException&) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3, "TFFramesWatchdog (%s): Frame %s is not reachable. Cause: %s",
      this->robot_frame_.c_str(), frame.c_str(), errstr->c_str());

    // if we couldn't get TF for this reachable frame, mark it unreachable
    this->markUnreachable(frame);
    return {};
  }
}

void TFFramesWatchdog::setMonitoredFrames(std::set<std::string> monitored_frames) {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  this->monitored_frames_ = std::move(monitored_frames);

  // if some monitored frames disappeared, delete them also from reachable_frames_
  for (auto& frame : this->reachable_frames_) {
    if (this->monitored_frames_.find(frame) == this->monitored_frames_.end()) {
      this->reachable_frames_.erase(frame);
    }
  }
}

void TFFramesWatchdog::addMonitoredFrame(const std::string& monitored_frame) {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  this->addMonitoredFrameNoLock(monitored_frame);
}

void TFFramesWatchdog::addMonitoredFrameNoLock(const std::string& monitored_frame) {
  this->monitored_frames_.insert(monitored_frame);
}

bool TFFramesWatchdog::isReachable(const std::string& frame) const {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  return this->isReachableNoLock(frame);
}

bool TFFramesWatchdog::isReachableNoLock(const std::string& frame) const {
  return this->reachable_frames_.find(frame) != this->reachable_frames_.end();
}

void TFFramesWatchdog::markReachable(const std::string& frame) {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  this->reachable_frames_.insert(frame);
}

void TFFramesWatchdog::markUnreachable(const std::string& frame) {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  this->reachable_frames_.erase(frame);
}

bool TFFramesWatchdog::areAllFramesReachable() const {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  return this->reachable_frames_.size() == this->monitored_frames_.size();
}

bool TFFramesWatchdog::isMonitored(const std::string& frame) const {
  std::lock_guard<std::mutex> guard(this->frames_mutex_);
  return this->isMonitoredNoLock(frame);
}

bool TFFramesWatchdog::isMonitoredNoLock(const std::string& frame) const {
  return this->monitored_frames_.find(frame) != this->monitored_frames_.end();
}

TFFramesWatchdog::~TFFramesWatchdog() {
  this->stop();
}

}  // namespace robot_body_filter
