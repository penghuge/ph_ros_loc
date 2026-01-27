/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "pose_extrapolator/imu_tracker.h"
#include "pose_extrapolator/pose_extrapolator_interface.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
//保持一定时间的姿势来估计线速度和角速度。利用速度来推断运动。
//使用IMU和/或里程计数据，如果可以改善预测效果。
class PoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit PoseExtrapolator(ros::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      ros::Duration pose_queue_duration, double imu_gravity_time_constant,
      const ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  ros::Time GetLastPoseTime() const override;
  ros::Time GetLastExtrapolatedTime() const override;

  void AddPose(ros::Time time, const geometry_msgs::Pose& pose) override;
  void AddImuData(const ImuData& imu_data) override;
  void AddOdometryData(const OdometryData& odometry_data) override;
  geometry_msgs::Pose ExtrapolatePose(ros::Time time) override;

  // ExtrapolationResult ExtrapolatePosesWithGravity(
  //     const std::vector<ros::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(ros::Time time) override;

  template <typename T>
  Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
      const Eigen::Quaternion<T>& quaternion) {
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
    // We choose the quaternion with positive 'w', i.e., the one with a smaller
    // angle that represents this orientation.
    if (normalized_quaternion.w() < 0.) {
      // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
      normalized_quaternion.w() = -1. * normalized_quaternion.w();
      normalized_quaternion.x() = -1. * normalized_quaternion.x();
      normalized_quaternion.y() = -1. * normalized_quaternion.y();
      normalized_quaternion.z() = -1. * normalized_quaternion.z();
    }
    // We convert the normalized_quaternion into a vector along the rotation axis
    // with length of the rotation angle.
    const T angle =
        2. * atan2(normalized_quaternion.vec().norm(), normalized_quaternion.w());
    constexpr double kCutoffAngle = 1e-7;  // We linearize below this angle.
    const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / 2.);
    return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                  scale * normalized_quaternion.y(),
                                  scale * normalized_quaternion.z());
  }

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(ros::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(ros::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(ros::Time time);

  const ros::Duration pose_queue_duration_;

  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;

  std::deque<OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
