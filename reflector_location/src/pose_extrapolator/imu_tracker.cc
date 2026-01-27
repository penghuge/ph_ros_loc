/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pose_extrapolator/imu_tracker.h"

#include <cmath>
#include <limits>

namespace cartographer {
namespace mapping {

ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const ros::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(ros::Time::MIN),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

void ImuTracker::Advance(const ros::Time time) {
  if(time_ > time)
    return;

  double delta_t = ros::Duration(time - time_).toSec();
//  LOG_INFO("delta_t = %f, imu_angular_velocity_ = (%f, %f, %f)", delta_t,
//           imu_angular_velocity_.x(),
//           imu_angular_velocity_.y(),
//           imu_angular_velocity_.z());
  const Eigen::Quaterniond rotation = AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
//  LOG_INFO("penghu*(%f, %f, %f)", transform::Rigid3d::Rotation(rotation).translation().x(),
//           transform::Rigid3d::Rotation(rotation).translation().y(),
//           transform::GetYaw(transform::Rigid3d::Rotation(rotation)));
  orientation_ = (orientation_ * rotation).normalized();
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  time_ = time;
}

void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t =
      last_linear_acceleration_time_ > ros::Time::MIN
          ? ros::Duration(time_ - last_linear_acceleration_time_).toSec()
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;
  //imu_gravity_time_constant_为观测重力的时间长度
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  // CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  // CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
