#include "location.h"



#define ENABLE_CARTOGRAPHER 0

namespace location {
Location::Location()
    : run_model(location::RunModelType::PURE_LOCATION),
      current_pose_(Pose(0., 0., 0.)) {
  v_ = 0.0;
  w_ = 0.0;
  relector_loc_ptr = std::make_shared<ReflectorLocation>();
}

bool Location::Init(ros::NodeHandle* nh, location::LocationOption option) {
  options = option;
  relector_loc_ptr->Init(options);
  current_pose_ << 0., 0., 0.;
  best_score_ = 0.0;
  reflector_scan_filter_ = std::make_shared<ReflectorFilter>(options);

  subscribers.emplace_back(nh->subscribe<geometry_msgs::PoseStamped>("tracked_pose", 2, boost::bind(&Location::HandleCartoPoseMsg, this, _1)));
  subscribers.emplace_back(nh->subscribe<std_msgs::Float32MultiArray>("best_score_submapid", 2, boost::bind(&Location::HandleCartoBestScore, this , _1)));

  return true;
}

bool Location::UpdateOption(location::LocationOption option) {
  options = option;
  return true;
}

bool Location::LoadLocationMap() {
  PointSet pts;

  if (!relector_loc_ptr->LoadMap()) return false;
  pts = relector_loc_ptr->MapInfo();

  return true;
}

void Location::AddLaserScanMessage(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  if (reflector_scan_filter_ != nullptr){
    reflector_scan_filter_->AddLaserScanMessage(scan);
    auto scan_reflector_points = reflector_scan_filter_->GetRflectorPoints();

    double laser_theta = options.ref_option.install_offset.theta;
    PointSet scan_points;

    if(scan_reflector_points != nullptr){
      PointSet ref_pts = *scan_reflector_points;
      for (std::size_t i = 0; i < ref_pts.size(); i++) { 
        double x = ref_pts[i].x() * std::cos(laser_theta) - ref_pts[i].y() * std::sin(laser_theta);
        double y = ref_pts[i].x() * std::sin(laser_theta) + ref_pts[i].y() * std::cos(laser_theta);
        scan_points.push_back(Point(x, y));
      }
    }
    
    relector_loc_ptr->SetScanReflectorPoints(scan_points);
  }
}

void Location::AddOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
  v_ = msg->twist.twist.linear.x;
  w_ = msg->twist.twist.angular.z;
  ROS_DEBUG_THROTTLE(1.0, "[topic @ odom] v: (%f, %f)", v_ , w_);
  relector_loc_ptr->AddOdomData(v_, w_);
}
void Location::AddMoveFeedbackMsg(double v, double w) {
  v_ = v;
  w_ = w;
   ROS_DEBUG_THROTTLE(1.0, "[topic @ odom] v: (%f, %f)", v_ , w_);

  relector_loc_ptr->AddOdomData(v_, w_);
}

void Location::AddMappingOrder(const std_msgs::Int32& msg) {
  relector_loc_ptr->HandleMappingOrder(msg);
}

void Location::HandleTransRefMapPoint(const geometry_msgs::Vector3& msg) {
  relector_loc_ptr->HandleTransRefMapPoint(msg);
}

void Location::HandleCartoPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  double theta = std::atan2(msg->pose.orientation.z,msg->pose.orientation.w) * 2;
  current_pose_ << msg->pose.position.x, msg->pose.position.y, theta;
  // ROS_INFO("carto pose:(%f, %f, %f)", current_pose_.x(), current_pose_.y(), current_pose_.theta());

  // pose_extrapolator_ptr_->AddPose(msg->header.stamp, msg->pose);
}

void Location::HandleCartoBestScore(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (msg->data.size() > 0){
    best_score_ = msg->data[0];
  }
  
  // ROS_INFO("best score: %f", best_score_);
}

std::shared_ptr<Pose> Location::GetPose() {
  std::shared_ptr<location::Pose> pose_ref = relector_loc_ptr->GetPose(current_pose_, best_score_);
  return pose_ref;
}

}  // namespace location