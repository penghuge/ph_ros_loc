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

#ifndef CARTO_MAPPING_BRIDGE_MSG_CONVERSION_H
#define CARTO_MAPPING_BRIDGE_MSG_CONVERSION_H

#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/common/format_transform.h"

namespace CartoMapppingBridge {

SlamCommon::SlamPointCloud2Data ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud);

//geometry_msgs::Transform ToGeometryMsgTransform(
//    const ::cartographer::transform::Rigid3d& rigid3d);

SlamCommon::Pose6DVecQua ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

//geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const SlamCommon::SlamLaserScanData& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const SlamCommon::SlamMultiEchoLaserScanData& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const SlamCommon::SlamPointCloud2Data& msg);

::cartographer::sensor::LandmarkData ToLandmarkData(
    const SlamCommon::SlamLandmarkData& landmark_list);

//::cartographer::transform::Rigid3d ToRigid3d(
//    const geometry_msgs::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(const SlamCommon::Pose6DVecQua& pose);

//Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);

//Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);

// Converts from WGS84 (latitude, longitude, altitude) to ECEF.
Eigen::Vector3d LatLongAltToEcef(double latitude, double longitude,
                                 double altitude);

// Returns a transform that takes ECEF coordinates from nearby points to a local
// frame that has z pointing upwards.
cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(double latitude,
                                                              double longitude);

// Points to an occupancy grid message at a specific resolution from painted
// submap slices obtained via ::cartographer::io::PaintSubmapSlices(...).
void CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const SlamCommon::Time& time, SlamCommon::OccupancyGrid *occupancy_grid);

SlamCommon::Time ToSlamTime(const cartographer::common::Time &t);
cartographer::common::Time FromSlamTime(const SlamCommon::Time &t);

}  // namespace CartoMapppingBridge

#endif  // CARTO_MAPPING_BRIDGE_MSG_CONVERSION_H
