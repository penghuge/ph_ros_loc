/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_

#include <memory>
#include <tuple>

#include "Eigen/Geometry"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

namespace cartographer {
namespace mapping {

struct TimedPose {
  ros::Time time;
  geometry_msgs::Pose pose;
};

struct OdometryData {
  ros::Time time;
  geometry_msgs::Pose pose;
};

struct ImuData {
  ros::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};


class PoseExtrapolatorInterface {
 public:
  PoseExtrapolatorInterface(const PoseExtrapolatorInterface&) = delete;
  PoseExtrapolatorInterface& operator=(const PoseExtrapolatorInterface&) =
      delete;
  virtual ~PoseExtrapolatorInterface() {}

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  virtual ros::Time GetLastPoseTime() const = 0;
  virtual ros::Time GetLastExtrapolatedTime() const = 0;

  virtual void AddPose(ros::Time time, const geometry_msgs::Pose& pose) = 0;
  virtual void AddImuData(const ImuData& imu_data) = 0;
  virtual void AddOdometryData(const OdometryData& odometry_data) = 0;
  virtual geometry_msgs::Pose ExtrapolatePose(ros::Time time) = 0;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  virtual Eigen::Quaterniond EstimateGravityOrientation(ros::Time time) = 0;

 protected:
  PoseExtrapolatorInterface() {}
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_
