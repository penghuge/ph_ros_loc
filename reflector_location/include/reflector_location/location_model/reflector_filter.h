
#ifndef REF_LOC_NODE_INCLUDE_REFLECTOR_FILTER_H
#define REF_LOC_NODE_INCLUDE_REFLECTOR_FILTER_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "reflector_location/location_option.h"

#include <deque>
#include <mutex>

#include "geometry.h"
#include "pose_extrapolator/pose_extrapolator.h"

namespace location {

class ReflectorFilter {
 public:
  ReflectorFilter(location::LocationOption option): option_(option){
    tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10.0));
    // debug_pub_ = nh_.advertise<sensor_msgs::PointCloud>("reflector_debug_points", 2);
  }
  void SetPoseExtrapolatorPtr(std::shared_ptr<cartographer::mapping::PoseExtrapolator> ptr) {
    pose_extrapolator_ptr_ = ptr;
  }

  void AddLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan) {
    CalculateReflectorPoint(scan);
  }

  ReflectorScan ToScanData(const sensor_msgs::LaserScan::ConstPtr& submsg) {
    ros::Time start_time = ros::Time::now();
    ReflectorScan scan_data;
    scan_data.reserve(submsg->ranges.size());

    // sensor_msgs::PointCloud debug_cloud;
    // debug_cloud.header.stamp = submsg->header.stamp;
    // debug_cloud.header.frame_id = submsg->header.frame_id;

    std::vector<geometry_msgs::Pose> range_data_poses;
    range_data_poses.reserve(submsg->ranges.size());

    ros::Time time;
    for (int i = 0; i < submsg->ranges.size(); i++) {
      time = submsg->header.stamp + ros::Duration(submsg->time_increment * i);
      if(time < pose_extrapolator_ptr_->GetLastExtrapolatedTime()){
        time = pose_extrapolator_ptr_->GetLastExtrapolatedTime();
      }
      range_data_poses.push_back(pose_extrapolator_ptr_->ExtrapolatePose(time));
    }

    for (int i = 0; i < submsg->ranges.size(); i++) {
      // auto angle = i * 2 * M_PI / (submsg->ranges.size());
      float angle = submsg->angle_min + i * submsg->angle_increment;
      float range = submsg->ranges[i];

      const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
      Eigen::Vector3f position = rotation * (range * Eigen::Vector3f::UnitX());

      if (std::isnan(position.x()) || std::isnan(position.y()) || std::isinf(position.x()) ||
          std::isinf(position.y()) || range < submsg->range_min || range > submsg->range_max) {
        continue;
      }

      //laser to agv center
      Eigen::Vector2d laser_xy(position.x(), position.y());
      Eigen::Vector2d transform0(option_.ref_option.install_offset.x, option_.ref_option.install_offset.y);
      Eigen::Rotation2D<double> rotation0(option_.ref_option.install_offset.theta);
      Eigen::Vector2d agv_center_xy = rotation0.toRotationMatrix() * laser_xy + transform0;
      // ROS_INFO("agv_center_xy: (%f, %f)", agv_center_xy.x(), agv_center_xy.y());

      // agv center to odom
      Eigen::Vector2d transform1(range_data_poses[i].position.x, range_data_poses[i].position.y);
      Eigen::Rotation2D<double> rotation1(tf::getYaw(range_data_poses[i].orientation));
      Eigen::Vector2d odom_xy = rotation1.toRotationMatrix() * agv_center_xy + transform1;
      // ROS_INFO("ref scan time: %f, transform: (%f, %f, %f)", time.toSec(), transform_x, transform_y, transform_theta);

      //odom to agv center
      Eigen::Vector2d transform2(range_data_poses.back().position.x, range_data_poses.back().position.y);
      Eigen::Rotation2D<double> rotation2(-tf::getYaw(range_data_poses.back().orientation));
      agv_center_xy = rotation2.toRotationMatrix() * (odom_xy - transform2);
      
      // agv center to laser
      rotation0 = Eigen::Rotation2D<double>(-option_.ref_option.install_offset.theta);
      laser_xy = rotation0.toRotationMatrix() * (agv_center_xy - transform0);

      // geometry_msgs::Point32 pt;
      // pt.x = laser_xy.x();
      // pt.y = laser_xy.y();
      // pt.z = 0.0;
      // debug_cloud.points.push_back(pt);

      float local_range = std::sqrt(laser_xy.x() * laser_xy.x() + laser_xy.y() * laser_xy.y());
      float local_angle = std::atan2(laser_xy.y(), laser_xy.x());

      scan_data.push_back(RawScanData(local_angle, local_range, submsg->intensities[i]));
    }

    // debug_pub_.publish(debug_cloud);
    ros::Time end_time = ros::Time::now();
    ROS_DEBUG_THROTTLE(1.0, "[topic @ scan] ToScanData time: %f ms", (end_time - start_time).toSec() * 1000.0);

    return scan_data;
  }

  double IntensityThreshold(double x) {
    double intensity =
        1190.83493649874 - 50.1087784482629 * x - 4.44315513082476 * x * x +
        0.438928139528978 * x * x * x - 0.0094094245092296 * x * x * x * x;
    return intensity;
  }

  ReflectorScan Filter(const ReflectorScan& data,
                       std::function<bool(const RawScanData&)> filter_method) {
    ReflectorScan queue;
    std::copy_if(data.begin(), data.end(), std::back_inserter(queue),
                 filter_method);
    return queue;
  }

  ReflectorScan FilterReflectorScan(ReflectorScan scan_data) {
    return Filter(scan_data, [&](const RawScanData& scan_data) {
      auto is_ref1 =
          scan_data.intensity >= (0.8 * IntensityThreshold(scan_data.range)) &&
          (scan_data.intensity != 0.0) && (scan_data.range < 20.0);
      // test code
      auto is_ref2 =
          scan_data.intensity > (option_.ref_option.ref_intensity_thrd / std::sqrt(scan_data.range)) &&
          scan_data.range < 30.0;
      // if (is_ref1 || is_ref2)
      //   ROS_WARN_STREAM("data:intensity: "
      //                   << scan_data.intensity
      //                   << " f1: " << 0.8 *
      //                   IntensityThreshold(scan_data.range)
      //                   << " f2: " << 1100 / std::sqrt(scan_data.range)
      //                   << " l: " << scan_data.range);
      return is_ref2;
    });
  }

  std::vector<ReflectorScan> SplitReflector(ReflectorScan ref_scan) {
    std::vector<ReflectorScan> reflector_list;
    ReflectorScan reflector(ref_scan);
    ReflectorScan ref_data;
    try {
      while (!reflector.empty()) {
        if (ref_data.empty()) {
          ref_data.push_back(reflector.front());
          reflector.erase(reflector.begin());
        }
        Point p1(ref_data.back().range * std::cos(ref_data.back().angle),
                 ref_data.back().range * std::sin(ref_data.back().angle));
        Point p2(reflector.front().range * std::cos(reflector.front().angle),
                 reflector.front().range * std::sin(reflector.front().angle));
        // ROS_DEBUG_STREAM("p1.DistanceTo(p2): " << p1.DistanceTo(p2));
        if (p1.DistanceTo(p2) < 0.1) {
#if 0
          ROS_ERROR_STREAM("reflector.size: " << reflector.size());
#endif
          if (reflector.size() == 0) {
            reflector_list.push_back(ref_data);
            // ref_data.clear();
            break;
          }
          ref_data.push_back(reflector.front());
          reflector.erase(reflector.begin());
        } else {
          reflector_list.push_back(ref_data);
          ref_data.clear();
        }
      }
    } catch (std::exception ex) {
      ROS_ERROR_STREAM(ex.what());
    }
#if 0
    int cnt = 0;
    for (auto ref : reflector_list) {
      cnt++;
      ROS_WARN_STREAM(cnt << "---------------------------------------------");
      for (auto data : ref) {
        ROS_INFO_STREAM("[" << cnt << "]" << data.angle << " " << data.range
                            << " " << data.intensity);
      }
    }
#endif
    return reflector_list;
  }

  PointSet ExtractReflectorPoint(std::vector<ReflectorScan> scan_list) {
    ReflectorScan reflector;
    double ref_radius = 0.0375;
/*
    for (auto scan : scan_list) {
      std::vector<double> tmp_dist;
      double angle = 0.;
      for (auto data : scan) {
        angle += data.angle;
      }
      double local_angle = angle / scan.size();

      for (int i = 0; i < scan.size(); i++) {
        double angle_to_center = scan[i].angle - local_angle;
        double local_num =
            scan[i].range * sin(abs(angle_to_center)) / ref_radius;

        local_num = local_num > 1. ? 1. : (local_num < -1. ? -1. : local_num);

        double distance_to_ref = scan[i].range * cos(angle_to_center) +
                                 ref_radius * cos(asin(local_num));
        tmp_dist.push_back(distance_to_ref);
      }
      double distance = 0;
      for (int i = 0; i < tmp_dist.size(); i++) {
        distance += tmp_dist[i];
      }
      double local_disatnce = distance / tmp_dist.size();
      tmp_dist.clear();
      reflector.push_back(RawScanData(local_angle - M_PI, local_disatnce, 0.0));
    }

*/
  /*------强度极大值----------*/
  for (auto scan : scan_list) {
      if(scan.size() < 2) continue;
    /*------todo test----------*/
      auto data = std::max_element(scan.begin(), scan.end(),
                                  [](const RawScanData &a, const RawScanData &b) {
                                    return a.intensity < b.intensity;
                                  });
//      LOG_ERROR("max intensity element: %f,%f,%f",data->angle,data->range,data->intensity);

      // reflector.push_back(*data);
      reflector.push_back(RawScanData(data->angle,data->range + 0.038));
  }

    auto pts = reflector.ToPointSet().PointsFilter(0.08);
#if 0
    for (int i = 0; i < pts.size(); i++) {
      ROS_DEBUG("Scan Point[%d]: {x: %.8f , y: %.8f}", i, pts[i].x(),
                pts[i].y());
    }
#endif
    return pts;
  }

  void CalculateReflectorPoint(const sensor_msgs::LaserScan::ConstPtr& submsg) {
    auto reflector_scan = FilterReflectorScan(ToScanData(submsg));
    auto reflector_points =
        ExtractReflectorPoint(SplitReflector(reflector_scan));

    std::unique_lock<std::mutex> lock(mutex_);
    if (reflector_points_deque_.size() > 2) reflector_points_deque_.pop_front();
    reflector_points_deque_.push_back(std::move(reflector_points));

    // ROS_INFO_STREAM(
    //     "reflector_points_deque_:" << reflector_points_deque_.size());
  }

  std::shared_ptr<PointSet> GetRflectorPoints() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (reflector_points_deque_.empty()) return nullptr;
    while (reflector_points_deque_.size() > 1) {
      reflector_points_deque_.pop_front();
    }
    return std::make_shared<PointSet>(reflector_points_deque_.back());
  }

 private:
  std::mutex mutex_;
  std::deque<PointSet> reflector_points_deque_;
  location::LocationOption option_;
  std::shared_ptr<tf::TransformListener> tf_;
  // ros::Publisher debug_pub_;
  ros::NodeHandle nh_;
  std::shared_ptr<cartographer::mapping::PoseExtrapolator> pose_extrapolator_ptr_ = nullptr;
};

}  // namespace location

#endif