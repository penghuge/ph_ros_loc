

#ifndef REF_LOC_NODE_INCLUDE_LOCATION_OPTION_H
#define REF_LOC_NODE_INCLUDE_LOCATION_OPTION_H
#include <ros/package.h>
#include <ros/ros.h>
#include <time.h>
#include <pwd.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/ExtendedKalmanFilter1.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <string>

#include "measure_model/PoseMeasurementModel.hpp"
#include "measure_model/PositionMeasurementModel.hpp"
#include "measure_model/system_model.h"
// #include "cartographer_ros_msgs/StartTrajectory.h"
// #include "cartographer_ros_msgs/StatusCode.h"
#include "reflector_location/geometry.h"

namespace {
constexpr double PI = 3.1415926;
constexpr char PackageName[] = "reflector_location";
}  // namespace

namespace location {
using Ptree = boost::property_tree::ptree;

typedef double T;

typedef State<T> State_;
typedef Control<T> Control_;
typedef SystemModel<T> SystemModel_;

typedef PositionMeasurement<T> PositionMeasurement_;
typedef PositionMeasurementModel<T> PositionMeasurementModel_;

typedef PoseMeasurement<T> PoseMeasurement_;
typedef PoseMeasurementModel<T> PoseMeasurementModel_;

constexpr double DistanceThresholdBetweenRefs1 = 0.3;
constexpr double DistanceThresholdBetweenRefs2 = 1.0;
constexpr double LoopClusureDistanceThreshold = 0.01;
constexpr char MapFrameName[] = "map";
constexpr char TrackingFrame[] = "ref_base_link";
constexpr char LaserScanName[] = "scan";
constexpr char OdomTopicName[] = "odom";
constexpr char MappingOderServiceName[] = "mapping_order";
constexpr char MapTransferServiceName[] = "trans";
constexpr int TopicReciveCacheSize = 2;

enum class RunModelType : int {
  PURE_LOCATION = 0,
  CREATE_NEW_REF_MAP = 1,
  APPEND_EXIST_REF_MAP = 2
};

enum class LocationMode : uint32_t {
  STATIC_LOCATION = 0,
  DYNAMIC_LOACTION = 1
};

// 0 - pure localization
// 1 - build new ref map simultaneously with carto mapping 
// 2 - build new ref map use ref pose
// 3 - build new ref map use carto pose
// 4 - append existing ref map use ref pose
// 5 - append existing ref map use carto pose
// 6 - append existing ref map use carto pose
// 99 - finish mapping, only used by service name "/mapping_order"
enum class MappingOrder : uint32_t {
  NO_MAPPING = 0,
  CREATE_NEW_REF_MAP_WITH_CARTO = 1,
  CREATE_NEW_REF_MAP_USE_REF_POSE = 2,
  CREATE_NEW_REF_MAP_USE_CARTO_POSE = 3,
  APPEND_EXIST_REF_MAP_WITH_CARTO = 4,
  APPEND_EXIST_REF_MAP_USE_REF_POSE = 5,
  APPEND_EXIST_REF_MAP_USE_CARTO_POSE = 6,
  FINISH_MAPPING = 99
};

enum class UsingPoseOrder : uint32_t {
  USE_REF_POSE = 0,
  USE_CARTO_POSE = 1
};

struct PoseBase {
  double x;
  double y;
  double theta;
};

struct MeasurementPose {
  ros::Time time_stamp;
  PoseMeasurement_ measurement;
  PoseBase noise;
};

struct Node_option {
  bool use_odom;
  bool use_imu;
  bool use_location_map;
  bool use_location_config;
  int mapping_mode;
};

struct RefMapOption {
  double map_scan_resolution;    // 地图扫描距离间隔分辨率 m
  double map_scan_length;        // 地图扫描确认反光柱距离 m
  double same_reflector_scope;   // 相同反光柱范围 m
  double diff_reflector_border;  // 不同反光柱边界 m
};

struct ReflectorOption {
  PoseBase sys_model_nosie;
  PoseBase measure_nosie;
  PoseBase install_offset;
  RefMapOption ref_map;

  int ref_intensity_thrd;
  int hit_ref_laser_min_num;
  double match_sys_diff_dist;
  int dynamic_match_min_ref_num;
  double diff_ref_dist_thrd;
  bool flag_use_file_pose;
};

struct LocationOption {
  Node_option node_option;
  PoseBase sys_model_nosie;
  ReflectorOption ref_option;
  PoseBase odom_option;
};

static std::string TimeNow() {
  time_t timep;
  struct timeval tv;
  time(&timep);
  gettimeofday(&tv, NULL);
  char tmp[64];
  strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
  char tmp_ns[20];
  uint64_t ns = tv.tv_usec;
  int i = snprintf(tmp_ns, sizeof(ns), "%.6ld", ns);
  std::string s1(tmp);
  std::string s(s1 + ":" + tmp_ns);
  return s;
}

static bool TimeOut(ros::Time time, double duration_thresh){
  ros::Duration duration = ros::Time::now() - time;
  if (duration.toSec() > duration_thresh){
    return true;
  }
  
  return false;
}

static std::string GetUserName() {
  uid_t userid;
  struct passwd* pwd;
  userid = getuid();
  pwd = getpwuid(userid);
  return pwd->pw_name;
}

struct TimedCartoPose {
  ros::Time time;
  Pose pose;
};

}  // namespace location
#endif  // REF_LOC_NODE_INCLUDE_LOCATION_OPTION_H