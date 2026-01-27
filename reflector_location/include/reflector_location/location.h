#ifndef REF_LOC_NODE_INCLUDE_LOCATION_H
#define REF_LOC_NODE_INCLUDE_LOCATION_H
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <mutex>
#include <thread>
#include "reflector_location/location_model/reflector_filter.h"
#include "reflector_location/location_model/reflector_location/reflector_location.h"
#include "reflector_location/location_option.h"

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include "pose_extrapolator/pose_extrapolator.h"

namespace location {

class Location {
 public:
  Location();

  bool Init(ros::NodeHandle* nh, location::LocationOption option);

  bool UpdateOption(location::LocationOption option);

  bool LoadLocationMap();

  void AddLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan);

  void AddOdomMsg(const nav_msgs::Odometry::ConstPtr& msg);

  void AddMoveFeedbackMsg(double v, double w);

  void AddMappingOrder(const std_msgs::Int32& msg);

  void HandleTransRefMapPoint(const geometry_msgs::Vector3& msg);

  void HandleCartoPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void HandleCartoBestScore(const std_msgs::Float32MultiArray::ConstPtr& msg);

  std::shared_ptr<Pose> GetPose();
  void SetPoseExtrapolatorPtr(std::shared_ptr<cartographer::mapping::PoseExtrapolator> ptr) {
    pose_extrapolator_ptr_ = ptr;
    reflector_scan_filter_->SetPoseExtrapolatorPtr(ptr);
  }


 private:
  location::LocationOption options;
  volatile location::RunModelType run_model;
  std::shared_ptr<ReflectorLocation> relector_loc_ptr = nullptr;
  Pose current_pose_;
  double best_score_;
  double v_;
  double w_;
  std::shared_ptr<ReflectorFilter> reflector_scan_filter_ = nullptr;
  std::vector<ros::Subscriber> subscribers;
  std::shared_ptr<cartographer::mapping::PoseExtrapolator> pose_extrapolator_ptr_ = nullptr;
};

}  // namespace location

#endif  // REF_LOC_NODE_INCLUDE_LOCATION_H
