

#ifndef REF_LOC_NODE_INCLUDE_reflector_location_NODE_H
#define REF_LOC_NODE_INCLUDE_reflector_location_NODE_H

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <common_msgs/SetInt.h>
#include <common_msgs/SetPose.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include "location.h"
#include "location_option.h"
#include "location_param.h"
#include <atomic>
#include "pose_extrapolator/pose_extrapolator.h"
#include "geometry_msgs/Pose.h"

namespace location {
class RefLocNode {
 public:
  RefLocNode(std::shared_ptr<Location> location);
  bool Init(ros::NodeHandle* nh);
  bool RefreshConfig(location::LocationOption options);
  bool LoadLocationMap() { return location_ptr->LoadLocationMap(); }

 private:
  void HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan);
  void HandleOdomMessage(const nav_msgs::Odometry::ConstPtr& msg);
  bool HandleMappingOrderRequest(common_msgs::SetInt::Request& req, common_msgs::SetInt::Response& res);
  bool HandleTransRefMapPointRequest(common_msgs::SetPose::Request& req, common_msgs::SetPose::Response& res);
  void SetInitSuccessFlag(bool flag) { init_success_.store(flag); } 
  bool GetInitSuccessFlag() { return init_success_.load(); }

  std::shared_ptr<Location> location_ptr;
  std::vector<ros::Subscriber> subscribers;
  std::vector<ros::ServiceServer> service_servers_;
  location::LocationOption options;
  std::shared_ptr<tf::TransformBroadcaster> tb_;
  ros::Publisher location_pose_pub;
  double v_;
  double w_;
  std::vector<ros::WallTimer> wall_timer_;
  std::shared_ptr<tf::TransformListener> tf_;
  std::atomic<bool> init_success_;
  std::shared_ptr<cartographer::mapping::PoseExtrapolator> pose_extrapolator_ptr_ = nullptr;
  // ros::Publisher debug_scan_pub_;
  sensor_msgs::PointCloud debug_scan_cloud_;

  void PublishRefPose(Pose current_pose);
  void SendTransform(Pose pose, std::string header_frame_id, std::string child_frame_id);
  void BroadcastRefTransform(Pose current_pose, const sensor_msgs::LaserScan::ConstPtr& scan);
  Pose LaserPoseToRobotPose(Pose& laser_pose);
  void HandleTwistOdomMessage(const geometry_msgs::Twist::ConstPtr& msg);

  void SetPoseExtrapolatorPtr(std::shared_ptr<cartographer::mapping::PoseExtrapolator> ptr) {
    location_ptr->SetPoseExtrapolatorPtr(ptr);
  }
};

}  // namespace location

#endif  // REF_LOC_NODE_INCLUDE_reflector_location_NODE_H
