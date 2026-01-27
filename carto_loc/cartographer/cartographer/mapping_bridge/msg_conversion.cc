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

#include "msg_conversion.h"

#include <cmath>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"
//#include "pcl/point_cloud.h"
//#include "pcl/point_types.h"
#include "absl/memory/memory.h"
/*
namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT {
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  float unused_padding[2];
};

}  // namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, time, time))*/

namespace CartoMapppingBridge {
namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::cartographer::sensor::LandmarkData;
using ::cartographer::sensor::LandmarkObservation;
using ::cartographer::sensor::PointCloudWithIntensities;
using ::cartographer::transform::Rigid3d;

SlamCommon::SlamPointCloud2Data PreparePointCloud2Message(const int64_t timestamp,
                                                          const std::string& frame_id,
                                                          const int num_points)
{
    SlamCommon::SlamPointCloud2Data msg;
    msg.header.time = SlamCommon::FromUniversal(timestamp);
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = num_points;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = SlamCommon::SlamPointFieldData::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = SlamCommon::SlamPointFieldData::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = SlamCommon::SlamPointFieldData::FLOAT32;
    msg.fields[2].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step = 16 * msg.width;
    msg.is_dense = true;
    msg.data.resize(16 * num_points);
    return msg;
}

// For sensor_msgs::LaserScan.
bool HasEcho(double) { return true; }

float GetFirstEcho(double range) { return range; }

// For sensor_msgs::MultiEchoLaserScan.
bool HasEcho(const SlamCommon::SlamLaserEchoData& echo)
{
    return !echo.echoes.empty();
}

float GetFirstEcho(const SlamCommon::SlamLaserEchoData& echo)
{
    return echo.echoes[0];
}

// For sensor_msgs::LaserScan and sensor_msgs::MultiEchoLaserScan.
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg)
{
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f)
  {
    CHECK_GT(msg.angle_max, msg.angle_min);
  }
  else
  {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }

  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
//  LOG_INFO("laser size = %d", msg.ranges.size());
  for (size_t i = 0; i < msg.ranges.size(); ++i)
  {
    const auto& echoes = msg.ranges[i];
    //如果有至少一层激光返回值
    if (HasEcho(echoes))
    {
      //取第一次的激光返回值
      const float first_echo = GetFirstEcho(echoes);
      //确保激光测距距离在量程范围内
      if (msg.range_min <= first_echo && first_echo <= msg.range_max)
      {
        //本帧激光束绕z轴的转角
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        //将激光数据由极坐标系转换到笛卡尔坐标系
        //以第一帧激光束为时间0，每帧激光束累加一个时间增量，如果激光在移动，可计算三维点的插值位置
        const cartographer::sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            (float)(i * msg.time_increment)};
//        LOG(INFO) << "laser " << i << ", time=" << (i * msg.time_increment);
        point_cloud.points.push_back(point);
        if (msg.intensities.size() > 0)
        {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        }
        else
        {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }

  ::cartographer::common::Time timestamp = ::cartographer::common::FromUniversal(
              SlamCommon::ToUniversal(msg.header.time));
  if (!point_cloud.points.empty())
  {
    //激光扫描周期duration
    const double duration = point_cloud.points.back().time;
    //将激光的时间戳累加上扫描周期， 什么作用？
    timestamp += cartographer::common::FromSeconds(duration);
    //将每个激光点的时间戳去掉周期，这样除了激光点时间戳从[激光时间戳-duration, 激光时间戳]
    //激光时间戳为接收到激光数据的时间
    for (auto& point : point_cloud.points)
    {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

//bool PointCloud2HasField(const SlamCommon::SlamPointCloud2Data& pc2,
//                         const std::string& field_name)
//{
//    for (const auto& field : pc2.fields)
//    {
//        if (field.name == field_name)
//        {
//            return true;
//        }
//    }
//    return false;
//}

}  // namespace

SlamCommon::SlamPointCloud2Data ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud)
{
    return PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());

//  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
//  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
//  for (const cartographer::sensor::TimedRangefinderPoint& point : point_cloud) {
//    stream.next(point.position.x());
//    stream.next(point.position.y());
//    stream.next(point.position.z());
//    stream.next(kPointCloudComponentFourMagic);
//  }
//  return msg;
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const SlamCommon::SlamLaserScanData& msg)
{
    return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const SlamCommon::SlamMultiEchoLaserScanData& msg)
{
    return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const SlamCommon::SlamPointCloud2Data& msg)
{
/*    PointCloudWithIntensities point_cloud;
    // We check for intensity field here to avoid run-time warnings if we pass in
    // a PointCloud2 without intensity.
    if (PointCloud2HasField(msg, "intensity"))
    {
        if (PointCloud2HasField(msg, "time"))
        {
            pcl::PointCloud<PointXYZIT> pcl_point_cloud;
            //TODO:待解析
//            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud)
            {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
                point_cloud.intensities.push_back(point.intensity);
            }
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
            //TODO:待解析
//            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud)
            {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
                point_cloud.intensities.push_back(point.intensity);
            }
        }
    }
    else
    {
        // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
        if (PointCloud2HasField(msg, "time"))
        {
            pcl::PointCloud<PointXYZT> pcl_point_cloud;
            //TODO:待解析
//            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud)
            {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
                point_cloud.intensities.push_back(1.0f);
            }
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
            //TODO:待解析
//            pcl::fromROSMsg(msg, pcl_point_cloud);
            point_cloud.points.reserve(pcl_point_cloud.size());
            point_cloud.intensities.reserve(pcl_point_cloud.size());
            for (const auto& point : pcl_point_cloud)
            {
                point_cloud.points.push_back(
                    {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
                point_cloud.intensities.push_back(1.0f);
            }
        }
    }

    ::cartographer::common::Time timestamp = ::cartographer::common::FromUniversal(
                SlamCommon::ToUniversal(msg.header.time));
    if (!point_cloud.points.empty())
    {
        const double duration = point_cloud.points.back().time;
        timestamp += cartographer::common::FromSeconds(duration);
        for (auto& point : point_cloud.points)
        {
            point.time -= duration;
            CHECK_LE(point.time, 0.f)
              << "Encountered a point with a larger stamp than "
                 "the last point in the cloud.";
        }
    }
    return std::make_tuple(point_cloud, timestamp);*/

    PointCloudWithIntensities point_cloud;
    ::cartographer::common::Time timestamp = FromSlamTime(msg.header.time);
    return std::make_tuple(point_cloud, timestamp);
}

LandmarkData ToLandmarkData(const SlamCommon::SlamLandmarkData& landmark_list)
{
  LandmarkData landmark_data;
  landmark_data.time = FromSlamTime(landmark_list.header.time);
  for (const SlamCommon::SlamLandmarkObservationData& entry : landmark_list.landmarks)
  {
    landmark_data.landmark_observations.push_back(
        {entry.id, Rigid3d(entry.landmark_to_tracking_transform.translation,
                           entry.landmark_to_tracking_transform.rotation),
         entry.translation_weight, entry.rotation_weight});
  }
  return landmark_data;
}

//Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
//  return Rigid3d(ToEigen(transform.transform.translation),
//                 ToEigen(transform.transform.rotation));
//}

Rigid3d ToRigid3d(const SlamCommon::Pose6DVecQua& pose)
{
    return Rigid3d(pose.translation, pose.rotation);
}

//Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
//  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
//}

//Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
//  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
//                            quaternion.z);
//}

//geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
//  geometry_msgs::Transform transform;
//  transform.translation.x = rigid3d.translation().x();
//  transform.translation.y = rigid3d.translation().y();
//  transform.translation.z = rigid3d.translation().z();
//  transform.rotation.w = rigid3d.rotation().w();
//  transform.rotation.x = rigid3d.rotation().x();
//  transform.rotation.y = rigid3d.rotation().y();
//  transform.rotation.z = rigid3d.rotation().z();
//  return transform;
//}

SlamCommon::Pose6DVecQua ToGeometryMsgPose(const Rigid3d& rigid3d)
{
  SlamCommon::Pose6DVecQua pose;
  pose.translation = rigid3d.translation();
  pose.rotation = rigid3d.rotation();
  return pose;
}

//geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d)
//{
//  geometry_msgs::Point point;
//  point.x = vector3d.x();
//  point.y = vector3d.y();
//  point.z = vector3d.z();
//  return point;
//}

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude)
{
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(cartographer::common::DegToRad(latitude));
  const double cos_phi = std::cos(cartographer::common::DegToRad(latitude));
  const double sin_lambda = std::sin(cartographer::common::DegToRad(longitude));
  const double cos_lambda = std::cos(cartographer::common::DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude)
{
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(cartographer::common::DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(cartographer::common::DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  return cartographer::transform::Rigid3d(rotation * -translation, rotation);
}

void CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const SlamCommon::Time& time, SlamCommon::OccupancyGrid *occupancy_grid)
{
  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height = cairo_image_surface_get_height(painted_slices.surface.get());

  occupancy_grid->header.time = time;
  occupancy_grid->header.frame_id = frame_id;
  occupancy_grid->map_load_time = time;
  occupancy_grid->resolution = resolution;
  occupancy_grid->width = width;
  occupancy_grid->height = height;
  occupancy_grid->origin.x = -painted_slices.origin.x() * resolution;
  occupancy_grid->origin.y = (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid->origin.theta = 0.;

  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->cells.clear();
  occupancy_grid->cells.reserve(width * height);
  for (int y = height - 1; y >= 0; --y)
  {
    for (int x = 0; x < width; ++x)
    {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid->cells.push_back((char)value);
    }
  }
}

SlamCommon::Time ToSlamTime(const cartographer::common::Time &t)
{
    return SlamCommon::FromUniversal(cartographer::common::ToUniversal(t));
}

cartographer::common::Time FromSlamTime(const SlamCommon::Time &t)
{
    return cartographer::common::FromUniversal(SlamCommon::ToUniversal(t));
}

}  // namespace CartoMapppingBridge
