

#include "node/reflector_location_node.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

namespace location {

RefLocNode::RefLocNode(std::shared_ptr<Location> location)
    : location_ptr(location), init_success_(false) {
  tb_ = std::make_shared<tf::TransformBroadcaster>();
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10.0));
  pose_extrapolator_ptr_ = std::make_shared<cartographer::mapping::PoseExtrapolator>(ros::Duration(0.001), 9.81);
}

bool RefLocNode::Init(ros::NodeHandle* nh) {
  nh->param("use_imu", options.node_option.use_imu, false);
  nh->param("use_odom", options.node_option.use_odom, true);
  nh->param("use_location_map", options.node_option.use_location_map, true);
  nh->param("use_location_config", options.node_option.use_location_config, true);

  nh->param("sys_model_nosie_x", options.sys_model_nosie.x, 0.05);
  nh->param("sys_model_nosie_y", options.sys_model_nosie.y, 0.05);
  nh->param("sys_model_nosie_theta", options.sys_model_nosie.theta, 0.0005);

  nh->param("reflector_option_sys_model_nosie_x", options.ref_option.sys_model_nosie.x, 0.005);
  nh->param("reflector_option_sys_model_nosie_y", options.ref_option.sys_model_nosie.y, 0.005);
  nh->param("reflector_option_sys_model_nosie_theta", options.ref_option.sys_model_nosie.theta, 0.0005);
  nh->param("reflector_option_measure_nosie_x", options.ref_option.measure_nosie.x, 0.0135);
  nh->param("reflector_option_measure_nosie_y", options.ref_option.measure_nosie.y, 0.0135);
  nh->param("reflector_option_measure_nosie_theta", options.ref_option.hit_ref_laser_min_num, 2);
  nh->param("hit_ref_laser_min_num", options.ref_option.measure_nosie.theta, 0.01);
  nh->param("ref_intensity_thrd", options.ref_option.ref_intensity_thrd, 1200);
  nh->param("match_sys_diff_dist", options.ref_option.match_sys_diff_dist, 20.0);
  nh->param("dynamic_match_min_ref_num", options.ref_option.dynamic_match_min_ref_num, 3);
  nh->param("diff_ref_dist_thrd", options.ref_option.diff_ref_dist_thrd, 0.6);
  
  nh->param("flag_use_file_pose", options.ref_option.flag_use_file_pose, true);

  nh->param("odom_option_measure_nosie_x", options.odom_option.x, 0.01);
  nh->param("odom_option_measure_nosie_y", options.odom_option.y, 0.01);
  nh->param("odom_option_measure_nosie_theta", options.odom_option.theta, 0.01);

  nh->param("mapping_mode", options.node_option.mapping_mode, 0);
  
  location_pose_pub = nh->advertise<nav_msgs::Odometry>("reflector_location", 2);
  // debug_scan_pub_ = nh->advertise<sensor_msgs::PointCloud>("test_motion_distortion", 2);

  ROS_INFO("==============Registe subscribers callback function==============");
  subscribers.emplace_back(nh->subscribe<sensor_msgs::LaserScan>(
      LaserScanName, TopicReciveCacheSize,
      boost::bind(&RefLocNode::HandleLaserScanMessage, this, _1)));
  ROS_INFO("Registe topic [%s] callback function: [HandleLaserScanMessage]", LaserScanName);

  subscribers.emplace_back(nh->subscribe<nav_msgs::Odometry>(
    OdomTopicName, TopicReciveCacheSize,
      boost::bind(&RefLocNode::HandleOdomMessage, this, _1)));
  ROS_INFO("Registe topic [%s] callback function: [HandleOdomMessage]", OdomTopicName);

  service_servers_.emplace_back(nh->advertiseService(MappingOderServiceName, &RefLocNode::HandleMappingOrderRequest, this));
  ROS_INFO("Service [%s] waiting for call!", MappingOderServiceName);

  service_servers_.emplace_back(nh->advertiseService(MapTransferServiceName, &RefLocNode::HandleTransRefMapPointRequest, this));
  ROS_INFO("Service [%s] waiting for call!", MapTransferServiceName);
  
  tf::StampedTransform tmp_transform;
  tf_->waitForTransform(std::string("base_link"), std::string("base_scan"), ros::Time(0), ros::Duration(30.0));//block until transfor is possible or 30s time out
  tf_->lookupTransform(std::string("base_link"), std::string("base_scan"), ros::Time(0), tmp_transform);
  options.ref_option.install_offset.x = tmp_transform.getOrigin().x();
  options.ref_option.install_offset.y = tmp_transform.getOrigin().y();
  options.ref_option.install_offset.theta = tf::getYaw(tmp_transform.getRotation());

  if (!location_ptr->Init(nh, options)){
    return false;
  }
  SetPoseExtrapolatorPtr(pose_extrapolator_ptr_);

  if (options.node_option.use_location_map) {
    if (!LoadLocationMap()) return false;
  }
  SetInitSuccessFlag(true);

  return true;
}

void RefLocNode::HandleLaserScanMessage(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  if(GetInitSuccessFlag()){
    ros::Time start_time = ros::Time::now();
    location_ptr->AddLaserScanMessage(scan);
    ros::Time end_time = ros::Time::now();
    ROS_DEBUG_THROTTLE(1.0, "[topic @ scan] process scan time: %f ms", (end_time - start_time).toSec() * 1000.0);

    auto pose = location_ptr->GetPose();
    if (pose != nullptr) {
      ROS_INFO_THROTTLE(1.0, "robot pose: (%f, %f, %f)", pose->x(), pose->y(), angles::to_degrees(pose->theta()));
      PublishRefPose(*pose);
      BroadcastRefTransform(*pose, scan);

      // geometry_msgs::Pose laser_pose;
      // laser_pose.position.x = pose->x();
      // laser_pose.position.y = pose->y();
      // laser_pose.position.z = 0.0;
      // laser_pose.orientation = tf::createQuaternionMsgFromYaw(pose->theta());
      // pose_extrapolator_ptr_->AddPose(scan->header.stamp, laser_pose);

      /* //test 
      debug_scan_cloud_.header.stamp = scan->header.stamp;
      debug_scan_cloud_.header.frame_id = "map";
      debug_scan_cloud_.points.clear();

      double laser_x = options.ref_option.install_offset.x;
      double laser_y = options.ref_option.install_offset.y;
      double laser_theta = options.ref_option.install_offset.theta;

      tf::StampedTransform tmp_transform;
      tf_->waitForTransform(std::string("map"), std::string("base_footprint"), scan->header.stamp, ros::Duration(30.0));//block until transfor is possible or 30s time out
      tf_->lookupTransform(std::string("map"), std::string("base_footprint"), scan->header.stamp, tmp_transform);
      double map_to_agv_center_x = tmp_transform.getOrigin().x();
      double map_to_agv_center_y = tmp_transform.getOrigin().y();
      double map_to_agv_center_theta = tf::getYaw(tmp_transform.getRotation());

      for(int i=0; i<scan->ranges.size(); i++){
        float angle = scan->angle_min + i * scan->angle_increment;
        float range = scan->ranges[i];
  
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        Eigen::Vector3f position = rotation * (range * Eigen::Vector3f::UnitX());
  
        double agv_center_x = position.x() * std::cos(laser_theta) - position.y() * std::sin(laser_theta) + laser_x;
        double agv_center_y = position.x() * std::sin(laser_theta) + position.y() * std::cos(laser_theta) + laser_y;

        double map_x = agv_center_x * std::cos(map_to_agv_center_theta) - agv_center_y * std::sin(map_to_agv_center_theta) + map_to_agv_center_x;
        double map_y = agv_center_x * std::sin(map_to_agv_center_theta) + agv_center_y * std::cos(map_to_agv_center_theta) + map_to_agv_center_y;
  
        double ref_x = (map_x - pose->x()) * std::cos(-pose->theta()) - (map_y - pose->y()) * std::sin(-pose->theta());
        double ref_y = (map_x - pose->x()) * std::sin(-pose->theta()) + (map_y - pose->y()) * std::cos(-pose->theta());
        
        geometry_msgs::Point32 pt;
        pt.x = map_x;
        pt.y = map_y;
        pt.z = 0.0;
        debug_scan_cloud_.points.push_back(pt);
      }
      debug_scan_pub_.publish(debug_scan_cloud_);
      */
    }
  }
}

void RefLocNode::HandleOdomMessage(
    const nav_msgs::Odometry::ConstPtr& msg) {
  if(GetInitSuccessFlag()){
    v_ = msg->twist.twist.linear.x;
    w_ = msg->twist.twist.angular.z;
    location_ptr->AddOdomMsg(msg);

    cartographer::mapping::OdometryData odom_data;
    odom_data.time = msg->header.stamp;
    odom_data.pose = msg->pose.pose;
    pose_extrapolator_ptr_->AddOdometryData(odom_data);

    pose_extrapolator_ptr_->AddPose(odom_data.time, odom_data.pose);
  }
}

void RefLocNode::HandleTwistOdomMessage(
    const geometry_msgs::Twist::ConstPtr& msg) {
  if(GetInitSuccessFlag()){
    v_ = msg->linear.x;
    w_ = msg->angular.x;
    location_ptr->AddMoveFeedbackMsg(v_, w_);
  }
}

bool RefLocNode::HandleMappingOrderRequest(common_msgs::SetInt::Request& req, common_msgs::SetInt::Response& res)
{
  std_msgs::Int32 msg;
  msg.data = req.data;

  res.result = "Recieved: " + std::to_string(req.data);

  location_ptr->AddMappingOrder(msg);

  return true;
}


bool RefLocNode::HandleTransRefMapPointRequest(common_msgs::SetPose::Request& req, common_msgs::SetPose::Response& res)
{
  geometry_msgs::Vector3 msg;
  msg.x = req.x;
  msg.y = req.y;
  msg.z = req.theta;

  res.result = "Recieved: " + std::to_string(req.x) + ", "  + std::to_string(req.y) + ", " + std::to_string(req.theta);

  location_ptr->HandleTransRefMapPoint(msg);

  return true;
}

bool RefLocNode::RefreshConfig(location::LocationOption options) {
  location_ptr->UpdateOption(options);
  ROS_INFO("load config");
  return true;
}

void RefLocNode::BroadcastRefTransform(
    Pose current_pose, const sensor_msgs::LaserScan::ConstPtr& scan) {
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = std::string(MapFrameName);
  tf.header.seq = scan->header.seq;
  tf.header.stamp = ros::Time::now();
  tf.child_frame_id = std::string(TrackingFrame);
  tf.transform.translation.x = current_pose.x();
  tf.transform.translation.y = current_pose.y();
  tf.transform.rotation = tf::createQuaternionMsgFromYaw(current_pose.theta());
  tb_->sendTransform(tf);
}

void RefLocNode::SendTransform(Pose pose, std::string header_frame_id,
                                      std::string child_frame_id) {
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = header_frame_id;
  tf.child_frame_id = child_frame_id;
  tf.header.stamp = ros::Time::now();
  tf.transform.translation.x = pose.x();
  tf.transform.translation.y = pose.y();
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = tf::createQuaternionMsgFromYaw(pose.theta());
  tb_->sendTransform(tf);
}

void RefLocNode::PublishRefPose(Pose current_pose) {
  static uint64_t seq = 0;
  seq++;
  nav_msgs::Odometry msg;
  msg.header.frame_id = std::string("map");
  msg.header.seq = seq;
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = std::string("base_link");
  msg.pose.pose.position.x = current_pose.x();
  msg.pose.pose.position.y = current_pose.y();
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(current_pose.theta());
  msg.twist.twist.linear.x = v_;
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.linear.z = w_;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = 0.0;
  location_pose_pub.publish(msg);
}

Pose RefLocNode::LaserPoseToRobotPose(Pose& laser_pose) {
  Eigen::Vector2d laser_coordinate(laser_pose.x(), laser_pose.y());
  Eigen::Rotation2D<double> laser_rotation(
      laser_pose.theta() - options.ref_option.install_offset.theta);
  Eigen::Vector2d laser_installation(options.ref_option.install_offset.x,
                                     options.ref_option.install_offset.y);
  Eigen::Vector2d robot_pose =
      laser_coordinate -
      laser_rotation.toRotationMatrix() * laser_installation +
      laser_installation;
  return Pose(robot_pose.x(), robot_pose.y(),
              laser_pose.theta() - options.ref_option.install_offset.theta);
}

}  // namespace location