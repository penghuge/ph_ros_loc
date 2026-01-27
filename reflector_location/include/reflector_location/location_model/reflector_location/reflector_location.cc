

#include "reflector_location/location_model/reflector_location/reflector_location.h"

namespace location {

ReflectorLocation::ReflectorLocation()
    : location_mode(LocationMode::STATIC_LOCATION),
      odom_init(false),
      bosch_init_(false),
      bosch_sync_(false) {
  reflector_points.clear();
  scan_reflector_points_.clear();
  control_ << 0., 0., 0.;
  run_model = RunModelType::PURE_LOCATION;
  mapping_order = MappingOrder::NO_MAPPING;

  static_count_ = 0;
  
  std_msgs::ColorRGBA red, green, blue, r_g, r_b, g_b;
  red.r = 1.0f; red.g = 0.0f; red.b = 0.0f; red.a = 1.0f;
  green.r = 0.0f; green.g = 1.0f; green.b = 0.0f; green.a = 1.0f;
  blue.r = 0.0f; blue.g = 0.0f; blue.b = 1.0f; blue.a = 1.0f;
  r_g.r = 1.0f; r_g.g = 1.0f; r_g.b = 0.0f; r_g.a = 1.0f;
  r_b.r = 1.0f; r_b.g = 0.0f; r_b.b = 1.0f; r_b.a = 1.0f;
  g_b.r = 0.0f; g_b.g = 1.0f; g_b.b = 1.0f; g_b.a = 1.0f;
  landmark_color_vec_ = {red, green, blue, r_g, r_b, g_b};
  
  all_loop_closure_reflector_pub_ = nh_.advertise<visualization_msgs::Marker>("all_loop_closure_map_reflectors", 2);
  all_reflector_pub_ = nh_.advertise<visualization_msgs::Marker>("all_map_reflectors", 2);
  text_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("text_marker_array", 2);
  matched_reflector_pub_ = nh_.advertise<visualization_msgs::Marker>("matched_reflectors", 2);
  matched_reflector_as_landmark_pub_ = nh_.advertise<cartographer_ros_msgs::LandmarkList>("landmark", 2);

  acquire_file_pose_client_ = nh_.serviceClient<std_srvs::Trigger>(AcquireFilePoseServiceName);
  acquire_ref_in_submap_client_ = nh_.serviceClient<common_msgs::SetPose>(RefInSubmapQueryServiceName);
  subscriblers_vec_.emplace_back(nh_.subscribe<cartographer_ros_msgs::SubmapList>("/submap_list", 2, &ReflectorLocation::HandleSubmapList, this));
  service_servers_vec_.emplace_back(nh_.advertiseService(SwitchMap, &ReflectorLocation::HandleSwitchMap, this));
}

bool ReflectorLocation::Init(LocationOption opt) {
  option = opt.ref_option;

  sys_model_.SetNoise(option.sys_model_nosie.x, option.sys_model_nosie.y,
                      option.sys_model_nosie.theta);
  position_measurement_model_.SetNoise(option.measure_nosie.x,
                                       option.measure_nosie.y);
  laser_install_offset << option.install_offset.x, option.install_offset.y,
      option.install_offset.theta;
  position_measurement_model_.SetTransform(laser_install_offset);

  filter_pose_deque_.clear();
  static_err_count_ = 0;
  switch_map_index_ = 0;
  flag_use_file_pose_ = option.flag_use_file_pose;

  std_msgs::Int32 msg;
  msg.data = opt.node_option.mapping_mode;
  HandleMappingOrder(msg);

  ros::service::waitForService(AcquireFilePoseServiceName);

  landmark_match_timer_ = nh_.createTimer(ros::Duration(0.2), &ReflectorLocation::HandleMatchLandmark, this);

  ROS_INFO("sys_model noise: (%f, %f, %f), measurement_model noise: (%f, %f), install_offset: (%f, %f, %f), flag_use_file_pose: %d", 
    option.sys_model_nosie.x, option.sys_model_nosie.y, option.sys_model_nosie.theta,
    option.measure_nosie.x, option.measure_nosie.y, option.install_offset.x, option.install_offset.y,
    option.install_offset.theta, flag_use_file_pose_);

  return true;
}

bool ReflectorLocation::UpdateOption(LocationOption opt) {
  option = opt.ref_option;
  return true;
}

bool ReflectorLocation::LoadMap() {
  if (run_model == RunModelType::PURE_LOCATION) {
    reflector_map.LoadMapFromFile("");
    GeneralRefZone();
    // reflector_map.SaveNewMapToFile(""); // test code--ok
    ROS_DEBUG("LoadMap: reflector_map size: %d", reflector_map.GetMap().size());
  }
  return true;
}

void ReflectorLocation::GeneralRefZone() {
    pts_zone.clear();
    PointSet pts_zone1;

    for (auto guide : reflector_map.GetMap()) {
        Point check_pt(guide.second.x(), guide.second.y());
        pts_zone1.push_back(Point(check_pt.x(),check_pt.y()));
    }
    pts_zone.push_back(pts_zone1);
    ROS_DEBUG_STREAM("pts_zone1 size: " << pts_zone1.size());
}

PointSet ReflectorLocation::MapInfo() { return reflector_map.ToPointSet(); }

void ReflectorLocation::AddOdomData(double v, double w) {
  control_.v() = v;
  control_.w() = w;

  ros::Duration time = ros::Time::now() - delta_time;
  delta_time = ros::Time::now();
  control_.time() = time.toSec();
  predict_pose_ = ekf_.predict(sys_model_, control_);
}

bool ReflectorLocation::HandleSwitchMap(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  if(request.data) {
    switch_map_index_ = 1;
  } else {
    switch_map_index_ = 0;
  }
  response.success = true;
  response.message = "switch_map_index_ set to " + std::to_string(switch_map_index_);

  return true;
}

void ReflectorLocation::HandleMappingOrder(const std_msgs::Int32& msg) {
  mapping_order = static_cast<MappingOrder>(msg.data);

  // mode selection: 
  // 0 - pure localization
  // 1 - build new ref map simultaneously with carto mapping 
  // 2 - build new ref map use ref pose
  // 3 - build new ref map use carto pose
  // 4 - append existing ref map simultaneously with carto mapping
  // 5 - append existing ref map use ref pose
  // 6 - append existing ref map use carto pose
  // 99 - finish mapping, only used by service name "/mapping_order"
  switch (mapping_order)
  {
  case MappingOrder::NO_MAPPING:
    run_model = RunModelType::PURE_LOCATION;
    ROS_INFO("HandleMappingOrder: NO_MAPPING");
    
    break;

  case MappingOrder::CREATE_NEW_REF_MAP_WITH_CARTO:
    run_model = RunModelType::CREATE_NEW_REF_MAP;
    using_pose_order = UsingPoseOrder::USE_CARTO_POSE;
    location_mode = LocationMode::STATIC_LOCATION;

    reflector_map.ClearMap();
    tmp_map.ClearMap();
    reflector_map_modified_by_loop_closure_.ClearMap();

    tmp_map.set_use_median_filter(true);
    reflector_map.set_use_median_filter(false);
    reflector_map_modified_by_loop_closure_.set_use_median_filter(false);
    ROS_INFO("HandleMappingOrder: CREATE_NEW_REF_MAP_WITH_CARTO");

    break;

  case MappingOrder::CREATE_NEW_REF_MAP_USE_REF_POSE:
    run_model = RunModelType::CREATE_NEW_REF_MAP;
    using_pose_order = UsingPoseOrder::USE_REF_POSE;
    location_mode = LocationMode::STATIC_LOCATION;

    reflector_map.ClearMap();
    tmp_map.ClearMap();
    reflector_map_modified_by_loop_closure_.ClearMap();

    tmp_map.set_use_median_filter(true);
    reflector_map.set_use_median_filter(false);
    reflector_map_modified_by_loop_closure_.set_use_median_filter(false);
    ROS_INFO("HandleMappingOrder: CREATE_NEW_REF_MAP_USE_REF_POSE");

    break;

  case MappingOrder::CREATE_NEW_REF_MAP_USE_CARTO_POSE:
    run_model = RunModelType::CREATE_NEW_REF_MAP;
    using_pose_order = UsingPoseOrder::USE_CARTO_POSE;
    location_mode = LocationMode::STATIC_LOCATION;

    reflector_map.ClearMap();
    tmp_map.ClearMap();
    reflector_map_modified_by_loop_closure_.ClearMap();

    tmp_map.set_use_median_filter(true);
    reflector_map.set_use_median_filter(false);
    reflector_map_modified_by_loop_closure_.set_use_median_filter(false);
    ROS_INFO("HandleMappingOrder: CREATE_NEW_REF_MAP_USE_CARTO_POSE");

    break;

  case MappingOrder::APPEND_EXIST_REF_MAP_WITH_CARTO:
    run_model = RunModelType::APPEND_EXIST_REF_MAP;
    using_pose_order = UsingPoseOrder::USE_CARTO_POSE;
    location_mode = LocationMode::STATIC_LOCATION;

    reflector_map.ClearMap();
    tmp_map.ClearMap();
    // tmp_map = reflector_map;
    reflector_map_modified_by_loop_closure_.ClearMap();
    // reflector_map_modified_by_loop_closure_ = reflector_map;

    tmp_map.set_use_median_filter(true);
    reflector_map.set_use_median_filter(false);
    reflector_map_modified_by_loop_closure_.set_use_median_filter(false);
    ROS_INFO("HandleMappingOrder: APPEND_EXIST_REF_MAP_WITH_CARTO, reflector_map size: %d", reflector_map.GetMap().size());

    break;

  case MappingOrder::APPEND_EXIST_REF_MAP_USE_REF_POSE:
    run_model = RunModelType::APPEND_EXIST_REF_MAP;
    using_pose_order = UsingPoseOrder::USE_REF_POSE;
    location_mode = LocationMode::STATIC_LOCATION;

    reflector_map.ClearMap();
    tmp_map.ClearMap();
    // tmp_map = reflector_map;
    reflector_map_modified_by_loop_closure_.ClearMap();
    // reflector_map_modified_by_loop_closure_ = reflector_map;

    tmp_map.set_use_median_filter(true);
    reflector_map.set_use_median_filter(false);
    reflector_map_modified_by_loop_closure_.set_use_median_filter(false);
    ROS_INFO("HandleMappingOrder: APPEND_EXIST_REF_MAP_USE_REF_POSE, reflector_map size: %d", reflector_map.GetMap().size());

    break;

  case MappingOrder::APPEND_EXIST_REF_MAP_USE_CARTO_POSE:
    run_model = RunModelType::APPEND_EXIST_REF_MAP;
    using_pose_order = UsingPoseOrder::USE_CARTO_POSE;
    location_mode = LocationMode::STATIC_LOCATION;

    reflector_map.ClearMap();
    tmp_map.ClearMap();
    // tmp_map = reflector_map;
    reflector_map_modified_by_loop_closure_.ClearMap();
    // reflector_map_modified_by_loop_closure_ = reflector_map;

    tmp_map.set_use_median_filter(true);
    reflector_map.set_use_median_filter(false);
    reflector_map_modified_by_loop_closure_.set_use_median_filter(false);
    ROS_INFO("HandleMappingOrder: APPEND_EXIST_REF_MAP_USE_CARTO_POSE, reflector_map size: %d", reflector_map.GetMap().size());

    break;

  case MappingOrder::FINISH_MAPPING:
    SaveNewMapToFile();
    reflector_map.ClearMap();
    tmp_map.ClearMap();
    reflector_map.LoadMapFromFile("");

    run_model = RunModelType::PURE_LOCATION;
    using_pose_order = UsingPoseOrder::USE_REF_POSE;
    location_mode = LocationMode::DYNAMIC_LOACTION;
    ROS_INFO("HandleMappingOrder: FINISH_MAPPING");

    break;
  
  default:
    break;
  }

  // if (mapping_order == MappingOrder::NO_MAPPING) {
  //   run_model = RunModelType::PURE_LOCATION;
  //   ROS_INFO("Get Mapping Order: RunModelType = PURE_LOCATION");
  // } else if (mapping_order == MappingOrder::START_MAPPING) {
  //   ROS_INFO("Get Mapping Order: START_MAPPING,CREATE_NEW_REF_MAP");
    
  //   location_mode = LocationMode::STATIC_LOCATION;
  //   reflector_map.ClearMap();
  //   tmp_map.ClearMap();
  // } else if (mapping_order == MappingOrder::APPEND_EXIST_REF_MAP) {
  //   ROS_INFO("Get Mapping Order: APPEND_EXIST_REF_MAP");
  //   run_model = RunModelType::APPEND_EXIST_REF_MAP;
  // } else if (mapping_order == MappingOrder::FINISH_MAPPING) {
  //   ROS_INFO("Get Mapping Order: FINISH_MAPPING SAVE MAP TO FILE");
  //   SaveNewMapToFile();
  //   reflector_map.ClearMap();
  //   tmp_map.ClearMap();
  //   reflector_map.LoadMapFromFile("");
  //   run_model = RunModelType::PURE_LOCATION;
  // }
  ROS_INFO("HandleMappingOrder: mapping_order: %d, run_model: %d, using_pose_order: %d, location_mode:%d", 
    mapping_order, run_model, using_pose_order, location_mode);
}

void ReflectorLocation::HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg){
  ROS_INFO_THROTTLE(1.0, "HandleSubmapList: submap_list size: %d", msg->submap.size());
  if (msg->submap.empty()) return;

  if (using_pose_order == UsingPoseOrder::USE_CARTO_POSE) {
    std::unique_lock<std::mutex> lock(submap_origins_update_mutex_);
    for (auto& submap : msg->submap) {
      int submap_index = submap.submap_index;
      int trajectory_id = submap.trajectory_id;
      int combined_index = trajectory_id * 10000 + submap_index;
      double x = submap.pose.position.x;
      double y = submap.pose.position.y;
      double th = tf::getYaw(submap.pose.orientation);
      submap_origins_[combined_index] = {x, y, th};
    }
  }
}

Point ReflectorLocation::TransformRefMapPointWithPose(Point pt, Pose& robot_pose) {
  Point offset_pt;
  offset_pt.setX( pt.x() * cos(robot_pose.theta()) + 
                  pt.y() * sin(robot_pose.theta()) + robot_pose.x());
  offset_pt.setY( pt.y() * cos(robot_pose.theta()) - 
                  pt.x() * sin(robot_pose.theta()) + robot_pose.y());

  return Point(offset_pt.x(),offset_pt.y());
}

void ReflectorLocation::HandleTransRefMapPoint(
    const geometry_msgs::Vector3& msg) {
  ROS_INFO_STREAM("-----HandleTransRefMapPoint-----");
  double theta = angles::from_degrees(msg.z);
  Pose robot_pose(msg.x, msg.y, theta);
  reflector_map.ClearOldTransMap();

  int i = 0;
  for (auto guide : reflector_map.GetMap()) {
    Point cur_pt(guide.second.x(), guide.second.y());
    Point ref_pt = TransformRefMapPointWithPose(cur_pt,robot_pose);
    i++;
    
    reflector_map.Add_Trans_Map(ref_pt.x(), ref_pt.y(), i, guide.second.count());
  }
  reflector_map.SaveTransMapToFile("");
}

std::vector<Pose> ReflectorLocation::MatchReflector(PointSet m, PointSet n) {
  // m:scan reflector point  || n: reflector map point
  find_pts.clear();
  z_.clear();
  std::vector<Pose> pose_list;
  if (m.size() < 2 || n.size() < 2) return pose_list;
  int k = m.size();
  ros::Time cal_time = ros::Time::now();
  while (k > 0) {
    for (auto scan_pt : m.choose(k)) {
      if (Match(std::move(scan_pt), std::move(n))) {
        for (auto pts : find_pts) {
#if 0
          ROS_WARN_STREAM("Reflectors: " << pts.size());
          for (auto pt : pts) {
            ROS_DEBUG_STREAM("Matched Ref is: (" << pt.x() << "," << pt.y()
                                                 << ")");
          }
          pts.GeneralEdgeMatrix();
          // ROS_DEBUG_STREAM("distance matrix:" << std::endl << pts.Dist_m());
#endif
          pose_list.push_back(CalcPose(pts));
          ROS_WARN_STREAM("1 pose_list.size:" << pose_list.size());
        }
      }
      find_pts.clear();
      z_.clear();
      ros::Duration duration = ros::Time::now() - cal_time;
      if(duration.toSec() > 0.55) {   // 计算超时25ms判定计算不出结果
          pose_list.clear();
          return pose_list;
      }
    }

    ros::Duration duration = ros::Time::now() - cal_time;
    if(duration.toSec() > 0.50) {   // 计算超时25ms判定计算不出结果
        pose_list.clear();
        return pose_list;
    }

    if (pose_list.empty()) {
      ROS_WARN("k--");
      k--;
    } else {
      return pose_list;
    }
  }
  ROS_INFO_STREAM("2 pose_list.size:" << pose_list.size());
  for (auto pose : pose_list) {
    ROS_INFO_STREAM("pose_list: (" << pose.x() << "," << pose.y() << ","
                                   << pose.theta() << ")");
  }
  return pose_list;
}

bool ReflectorLocation::MatchLandmark(PointSet&& map_point_set, PointSet&& current_point_set) {
  find_pts_landmark_.clear();
  z_landmark_.clear();
  if (current_point_set.size() < 2 || map_point_set.size() < 2) return false;

   current_point_set.GeneralEdgeMatrix();
   z_landmark_ = current_point_set.GetPointsWithMaxEdge();
   map_point_set.GeneralEdgeMatrix();
   find_pts_landmark_ = map_point_set.GetCandidateWithEdge(current_point_set.MaxDistance());
 
   ROS_DEBUG("find_pts_landmark size: %d, max_distance: %f", find_pts_landmark_.size(), current_point_set.MaxDistance());
   if (find_pts_landmark_.empty()) return false;
 
   ros::Time cal_time = ros::Time::now();
   while (find_pts_landmark_.size() > 1) {
     if (z_landmark_.size() == current_point_set.size()){
      return true;
     }
     if (z_landmark_.size() ==2 && current_point_set.size()== 1) return false;
     ROS_DEBUG_STREAM("z_.size():" << z_landmark_.size() << ", m.size():" << current_point_set.size());
     ros::Duration duration = ros::Time::now() - cal_time;
     if(duration.toSec() > 0.195) {   // 计算超时12ms判定计算不出结果
        ROS_DEBUG("find_pts_landmark: timeout: %f > 0.195 s", duration.toSec());
        return false;
     }
 
     for (auto mi : current_point_set) {
       if (!z_landmark_.ContainPoint(mi)) {
          z_landmark_.push_back(mi);
          break;
       }
     }
     if (!AppendCandidatesLandmark(map_point_set)) return false;
   }
 
   if (1 == find_pts_landmark_.size()) {
     for (auto mi : current_point_set) {
       if (!z_landmark_.ContainPoint(mi)) {
          z_landmark_.push_back(mi);
          if (!AppendCandidatesLandmark(map_point_set)) return false;
       }
     }
   }
   
   return true;
}

void ReflectorLocation::HandleMatchLandmark(const ::ros::TimerEvent& timer_event) {
  if (mapping_order == MappingOrder::CREATE_NEW_REF_MAP_WITH_CARTO || mapping_order == MappingOrder::APPEND_EXIST_REF_MAP_WITH_CARTO) {
    std::unique_lock<std::mutex> lock(landmark_pts_mutex_);
    std::vector<int> ids = loop_closure_pts_for_match_landmark_.first;
    PointSet map_pts = loop_closure_pts_for_match_landmark_.second;
    lock.unlock();
  
    PointSet current_pts = GetScanReflectorPoints();
    PointSet roi_map_trim_current_pts;
    PointSet roi_map_pts;
    double r_roi = 30.0;
    for(auto mp : map_pts) {
      bool is_pt_in_roi = std::hypot(mp.x() - current_carto_pose_.x(), mp.y() - current_carto_pose_.y()) < r_roi ? true : false;
      if(is_pt_in_roi){
        bool found = false;
        roi_map_pts.push_back(mp);

        for(auto cp : current_pts) {
          Eigen::Vector2d laser_xy(cp.x(), cp.y());
          Eigen::Vector2d laser_to_base_footprint_translation(option.install_offset.x, option.install_offset.y);
          Eigen::Rotation2D<double> laser_to_base_footprint_rotation(option.install_offset.theta);
          Eigen::Vector2d base_footprint_xy = laser_to_base_footprint_rotation.toRotationMatrix() * laser_xy + laser_to_base_footprint_translation;
  
          Eigen::Vector2d base_footprint_to_map_translation(current_carto_pose_.x(), current_carto_pose_.y());
          Eigen::Rotation2D<double> base_footprint_to_map_rotation(current_carto_pose_.theta());
          Eigen::Vector2d map_xy = base_footprint_to_map_rotation.toRotationMatrix() * base_footprint_xy + base_footprint_to_map_translation;
  
          if(std::fabs(mp.x() - map_xy.x()) < DistanceThresholdBetweenRefs1 && std::fabs(mp.y() - map_xy.y()) < DistanceThresholdBetweenRefs1) {
            found = true;
            break;
          }
        }
        if(!found) {
          roi_map_trim_current_pts.push_back(mp);
        }
      }
    }

    PointSet using_pts = roi_map_pts;
    if(switch_map_index_ == 1) {
      using_pts = roi_map_trim_current_pts;
    }
    ROS_INFO("MatchLandmarkProcess: map_pts size: %d, current_pts size: %d, roi_map size: %d", 
      map_pts.size(), current_pts.size(), using_pts.size());
    if(MatchLandmark(std::move(using_pts), std::move(current_pts))) {
      ROS_INFO("MatchLandmarkProcess: find_pts_landmark size: %d", find_pts_landmark_.size());

      current_landmark_list_.header.stamp = ros::Time::now();
      current_landmark_list_.header.frame_id = "base_footprint";
      current_landmark_list_.landmarks.clear();

      PointSet using_landmark;
      if(find_pts_landmark_.size() == 1) {
        using_landmark = find_pts_landmark_.front();
        ROS_INFO("MatchLandmarkProcess: find_pts_landmark front size: %d", find_pts_landmark_.front().size());
      }else if(find_pts_landmark_.size() > 1) {
        auto max_ele = std::max_element(find_pts_landmark_.begin(), find_pts_landmark_.end(), [](const PointSet& a, const PointSet& b) {
          return a.size() < b.size();
        });
        using_landmark = *max_ele;
        ROS_INFO("MatchLandmarkProcess: find_pts_landmark max_ele size: %d", max_ele->size());
      }else {
        ROS_ERROR("MatchLandmarkProcess: find_pts_landmark size is 0: %d", find_pts_landmark_.size());
      }

      for(auto pt : using_landmark) {
        Eigen::Vector2d map_xy(pt.x(), pt.y());
        Eigen::Vector2d map_to_base_footprint_translation(predict_pose_.x(), predict_pose_.y());
        Eigen::Rotation2D<double> map_to_base_footprint_rotation(-predict_pose_.theta());
        Eigen::Vector2d base_footprint_xy = map_to_base_footprint_rotation.toRotationMatrix() * (map_xy - map_to_base_footprint_translation);

        int tmp_id = 0;
        for(int i=0; i<map_pts.size(); i++) {
          if(std::fabs(pt.x() - map_pts[i].x()) < 0.05 && std::fabs(pt.y() - map_pts[i].y()) < 0.05) {
            // ROS_INFO("MatchLandmarkProcess: find matched landmark id: %d", mp.second.id());
            tmp_id = ids[i];
            break;
          }
        }

        cartographer_ros_msgs::LandmarkEntry landmark;
        landmark.id = std::to_string(tmp_id);
        landmark.tracking_from_landmark_transform.position.x = base_footprint_xy.x();
        landmark.tracking_from_landmark_transform.position.y = base_footprint_xy.y();
        landmark.tracking_from_landmark_transform.position.z = 0.0;
        landmark.tracking_from_landmark_transform.orientation.w = 1.0;
        landmark.tracking_from_landmark_transform.orientation.x = 0.0;
        landmark.tracking_from_landmark_transform.orientation.y = 0.0;
        landmark.tracking_from_landmark_transform.orientation.z = 0.0;
        landmark.translation_weight = 1e2;
        landmark.rotation_weight = 0.0;
        current_landmark_list_.landmarks.push_back(landmark);
        ROS_DEBUG("landmark: id: %s, map pos(%f, %f), transform(%f, %f, %f),base_footprint pos:(%f, %f)", 
          landmark.id.c_str(), map_xy.x(), map_xy.y(), predict_pose_.x(), predict_pose_.y(), predict_pose_.theta(),
          landmark.tracking_from_landmark_transform.position.x, landmark.tracking_from_landmark_transform.position.y);
      }

      if(current_landmark_list_.landmarks.size() > 3) matched_reflector_as_landmark_pub_.publish(current_landmark_list_);
    }
  }
}

bool ReflectorLocation::Match(PointSet&& m, PointSet&& n) {
  // scan data
  m.GeneralEdgeMatrix();
  // find the max edge end point
  z_ = m.GetPointsWithMaxEdge();
  // reflector map
  n.GeneralEdgeMatrix();
  // find matched largest edge points
  // ROS_INFO_STREAM("m.MaxDistance()" << m.MaxDistance());
  find_pts = n.GetCandidateWithEdge(m.MaxDistance());

#if 0
  ROS_WARN_STREAM("1 find_pts size:" << find_pts.size());
  for (auto f : find_pts) {
    ROS_INFO_STREAM("-------------------------------------------");
    ROS_INFO_STREAM("find_pts1: " << f[0].x() << "," << f[0].y());
    ROS_INFO_STREAM("find_pts2: " << f[1].x() << "," << f[1].y());
    ROS_INFO_STREAM("-------------------------------------------");
  }
#endif

  // ROS_ERROR_STREAM("find_pts.size()" << find_pts.size());
  if (find_pts.empty()) return false;

  ros::Time cal_time = ros::Time::now();
  while (find_pts.size() > 1) {
    if (z_.size() == m.size()) return true;
    if (z_.size() ==2 && m.size()== 1) return false;
    ROS_DEBUG_STREAM("z_.size():" << z_.size() << " m.size():" << m.size());
    ros::Duration duration = ros::Time::now() - cal_time;
    if(duration.toSec() > 0.50) {   // 计算超时12ms判定计算不出结果
        return false;
    }

    for (auto mi : m) {
      if (!z_.ContainPoint(mi)) {
        z_.push_back(mi);
        break;
      }
    }
    if (!AppendCandidates(n)) return false;
  }

  if (1 == find_pts.size()) {
    for (auto mi : m) {
      if (!z_.ContainPoint(mi)) {
        z_.push_back(mi);
        if (!AppendCandidates(n)) return false;
      }
    }
  }

  return true;
}

bool ReflectorLocation::AppendCandidates(const PointSet& pts) {
  std::vector<PointSet> tmpCandidateList;
  for (auto candidate : find_pts) {
    for (auto pt : pts) {
      // 找与地图中反光板距离差不多一样的候选点
      if (z_.AddpointCorrespond(candidate, pt)) {
        PointSet tmpCandidate = {};
        tmpCandidate.assign(candidate.begin(), candidate.end());
        tmpCandidate.push_back(pt);
        tmpCandidateList.push_back(tmpCandidate);
      }
    }
  }

  if (!tmpCandidateList.empty()) {
    find_pts.clear();
    find_pts.assign(tmpCandidateList.begin(), tmpCandidateList.end());
    return true;
  }
  return false;
}

bool ReflectorLocation::AppendCandidatesLandmark(const PointSet& pts) {
  std::vector<PointSet> tmpCandidateList;
  for (auto candidate : find_pts_landmark_) {
    for (auto pt : pts) {
      // 找与地图中反光板距离差不多一样的候选点
      if(!candidate.ContainPoint(pt)){
        if (z_landmark_.AddpointCorrespond(candidate, pt)) {
          PointSet tmpCandidate = {};
          tmpCandidate.assign(candidate.begin(), candidate.end());
          tmpCandidate.push_back(pt);
          tmpCandidateList.push_back(tmpCandidate);
        }
      }
    }
  }

  if (!tmpCandidateList.empty()) {
    find_pts_landmark_.clear();
    find_pts_landmark_.assign(tmpCandidateList.begin(), tmpCandidateList.end());
    return true;
  }
  return false;
}

// 最小二乘求解
Pose ReflectorLocation::CalcPose(PointSet& pts) {
  Eigen::MatrixXd A(2 * z_.size(), 4);
  Eigen::MatrixXd b(2 * z_.size(), 1);

  for (uint32_t i = 0; i < z_.size(); i++) {
    A(2 * i, 0) = z_[i].x();
    A(2 * i, 1) = -z_[i].y();
    A(2 * i, 2) = 1;
    A(2 * i, 3) = 0;
    b(2 * i, 0) = pts[i].x();

    A(2 * i + 1, 0) = z_[i].y();
    A(2 * i + 1, 1) = z_[i].x();
    A(2 * i + 1, 2) = 0;
    A(2 * i + 1, 3) = 1;
    b(2 * i + 1, 0) = pts[i].y();
  }
  // Eigen::MatrixXd H = A.jacobiSvd(Eigen::DecompositionOptions::ComputeThinU |
  //                                 Eigen::DecompositionOptions::ComputeThinV)
  //                         .solve(b);
  Eigen::MatrixXd H = (A.transpose() * A).inverse() * A.transpose() * b;
  double x = H(2);
  double y = H(3);
  double theta = atan2(H(1), H(0));
#if 0
  ROS_INFO_STREAM("A:" << std::endl << A);
  ROS_INFO_STREAM("b:" << std::endl << b);
#endif
#if 1
  ROS_DEBUG_STREAM("Calculate Pose: (" << x << "," << y << "," << theta << ")");
#endif
  return Pose(x, y, theta);
}


bool ReflectorLocation::ReadInitialPoseFromTxt(Pose &init_pose){
  std::stringstream str_stream;
  std::ifstream infile;
  std::string str_tempt;

  std::string pose_path = "record_pose_0.txt";

  infile.open(pose_path.c_str());

  if (!infile.is_open()) {
      ROS_WARN("Open %s failed!", pose_path.c_str());
      return false;
  }

  while (infile.good() && !infile.eof()) {
      getline(infile, str_tempt, '\n');
      if (str_tempt.empty()) {
          continue;
      }
      str_stream.clear();
      str_stream.str(str_tempt);

      int init_time = 0;
      double init_x = 0.0, init_y = 0.0, init_th = 0.0;
      str_stream >> init_time >> init_x >> init_y >> init_th;
      init_pose = Pose(init_x, init_y, init_th);

      if (str_stream.fail()) {
          ROS_WARN("String stream from reading %s failed! Continue.", pose_path.c_str());
          continue;
      }
  }

  infile.close();
  str_stream.clear();
  str_stream.str("");
  ROS_INFO("Read %s successfully!", pose_path.c_str());
  return true;
}

void ReflectorLocation::AcquireFilePoseFunction(Pose& file_pose){
  std_srvs::Trigger srv;
  srv.request = {};
  acquire_file_pose_client_.call(srv);
  while (!srv.response.success){
    acquire_file_pose_client_.call(srv);
    ROS_ERROR("Ref call %s failed, get file_pose: %s", AcquireFilePoseServiceName, srv.response.message.c_str());
    usleep(10000);
  }

  if(srv.response.success){
    ROS_INFO("Ref call %s success, get file_pose: %s", AcquireFilePoseServiceName, srv.response.message.c_str());

    std::stringstream ss(srv.response.message);
    double file_pose_x, file_pose_y, file_pose_theta;
    ss >> file_pose_x >> file_pose_y >> file_pose_theta;

    file_pose = Pose(file_pose_x, file_pose_y, file_pose_theta);
  }
}

std::vector<int> ReflectorLocation::AcquireSubmapIdOfRefPose(double x, double y, int ref_id) {
  common_msgs::SetPose srv;
  srv.response.result = "none";
  srv.request.x = x;
  srv.request.y = y;
  srv.request.theta = double(ref_id);

  std::vector<int> submap_id_vec;
  int count = 0;
  while (srv.response.result == "none"){
    acquire_ref_in_submap_client_.call(srv);
    if (srv.response.result != "none"){
      std::stringstream ss(srv.response.result);
      ROS_INFO("srv.response.result: %s", srv.response.result.c_str());
  
      int submap_id = -1;
      int return_ref_id = -1;
      ss >> return_ref_id;

      if(return_ref_id != -1 && return_ref_id != ref_id){
        ROS_ERROR("Ref(%f, %f) call %s failed: return_ref_id %d != ref_id %d, continue call ...", x, y, RefInSubmapQueryServiceName, return_ref_id, ref_id);
        usleep(300000);
        continue;
      }
      while (ss >> submap_id)
      {
        submap_id_vec.push_back(submap_id);
      }
      break;
    }
    if (count > 9) {
      ROS_ERROR("Ref(%f, %f) call %s failed: count %d > 9, get submap_id: %s", x, y, RefInSubmapQueryServiceName, count, srv.response.result.c_str());
      break;
    }
    count++;

    usleep(300000);
  }
  
  return submap_id_vec;
}

void ReflectorLocation::AdjustMapFromLoopClosure(){
  int ref_id = 0;
  static int ref_count = 0;
  std::map<uint32_t, Reflector> reflector_map_get_map = reflector_map.GetMap();
  if(!reflector_map_get_map.empty()) {
    ref_count %= int(tmp_map.GetMap().size());
    ref_id = ref_count+1;// ref index start from 1
    ++ref_count;
    
    if(processed_corresponding_submap_info_when_ref_first_created_.find(ref_id) != processed_corresponding_submap_info_when_ref_first_created_.end()) {
      int submap_id = processed_corresponding_submap_info_when_ref_first_created_[ref_id].first;
      // ROS_INFO("ref_id: %d, submap_id: %d", ref_id, submap_id);
      std::map<int, std::vector<double>> submap_origins_tmp = get_submap_origins();
      if(submap_origins_tmp.find(submap_id) != submap_origins_tmp.end()){
        ROS_INFO("submap_origins_[submap_id: %d].size(): %d, submap_origin: (%f, %f, %f), corr origin size: %d", submap_id, submap_origins_tmp[submap_id].size(), 
                  submap_origins_tmp[submap_id].at(0), submap_origins_tmp[submap_id].at(1), submap_origins_tmp[submap_id].at(2),
                  processed_corresponding_submap_info_when_ref_first_created_[ref_id].second.size());
        
        if(reflector_map_get_map.find(ref_id) != reflector_map_get_map.end()) {
          Reflector one_ref = reflector_map_get_map[ref_id];
          // ROS_INFO("reflector_map_get_map: %d, ref_id: %d, corr size: %d", reflector_map_get_map.size(), ref_id, processed_corresponding_submap_info_when_ref_first_created_.size());
          double x = one_ref.x();
          double y = one_ref.y();
    
          if(submap_origins_tmp[submap_id].size() == 3 && 
              processed_corresponding_submap_info_when_ref_first_created_[ref_id].second.size() == 3){
            double raw_submap_origin_x = processed_corresponding_submap_info_when_ref_first_created_[ref_id].second.at(0);
            double raw_submap_origin_y = processed_corresponding_submap_info_when_ref_first_created_[ref_id].second.at(1);
            double raw_submap_origin_th = processed_corresponding_submap_info_when_ref_first_created_[ref_id].second.at(2);
    
            double submap_origin_x = submap_origins_tmp[submap_id].at(0);
            double submap_origin_y = submap_origins_tmp[submap_id].at(1);
            double submap_origin_th = submap_origins_tmp[submap_id].at(2);
    
            double x_in_submap = (x-raw_submap_origin_x) * std::cos(-raw_submap_origin_th) - (y-raw_submap_origin_y) * std::sin(-raw_submap_origin_th);
            double y_in_submap = (x-raw_submap_origin_x) * std::sin(-raw_submap_origin_th) + (y-raw_submap_origin_y) * std::cos(-raw_submap_origin_th);
    
            double x_in_global = x_in_submap * std::cos(submap_origin_th) - y_in_submap * std::sin(submap_origin_th) + submap_origin_x;
            double y_in_global = x_in_submap * std::sin(submap_origin_th) + y_in_submap * std::cos(submap_origin_th) + submap_origin_y;
            ROS_INFO("raw ref xy: (%f, %f), local xy: (%f, %f), global xy: (%f, %f)", x, y, x_in_submap, y_in_submap, x_in_global, y_in_global);
            bool new_ref = true;
            if((x_in_global == 0.0 && y_in_global == 0.0)/* || one_ref.count() < 30*/){
              new_ref = false;
            }
            
            auto reflector_map_modified_by_loop_closure_map = reflector_map_modified_by_loop_closure_.GetMap();
            if(reflector_map_modified_by_loop_closure_map.find(ref_id) != reflector_map_modified_by_loop_closure_map.end()){
              x = reflector_map_modified_by_loop_closure_map[ref_id].x();
              y = reflector_map_modified_by_loop_closure_map[ref_id].y();
              ROS_INFO("reflector_map_modified_by_loop_closure_map exist: ref_id: %d, xy: (%f, %f)", ref_id, x, y);
            }
            
            double ref_pose_diff_between_loop_closure = std::hypot(x_in_global - x, y_in_global - y);
            bool exist_loop_closure = ref_pose_diff_between_loop_closure > LoopClusureDistanceThreshold ? true : false;
  
            reflector_map_modified_by_loop_closure_map = reflector_map_modified_by_loop_closure_.GetMap();
            if (exist_loop_closure) {
              if(reflector_map_modified_by_loop_closure_map.find(ref_id) != reflector_map_modified_by_loop_closure_map.end()){
                reflector_map_modified_by_loop_closure_.Modify(ref_id, x_in_global, y_in_global);
                ROS_INFO("reflector_map_modified_by_loop_closure_.Modify(): ref_pose_diff_between_loop_closure: %f", ref_pose_diff_between_loop_closure);
              }else {
                if(new_ref) {
                  reflector_map_modified_by_loop_closure_.Add(x_in_global, y_in_global, ref_id);//loop closure happens before ref is added to map
                }
              }
            }else {
              if(new_ref) {
                reflector_map_modified_by_loop_closure_.Add(x_in_global, y_in_global, ref_id);
              }
            }
  
            reflector_map_modified_by_loop_closure_map = reflector_map_modified_by_loop_closure_.GetMap();
            for (auto ref : reflector_map_modified_by_loop_closure_map) {
              for(auto tmp_ref : reflector_map.GetMap()) {
                if(ref.second.id() == tmp_ref.second.id()){
                  double tmp_diff = std::hypot(ref.second.x() - tmp_ref.second.x(), ref.second.y() - tmp_ref.second.y());
                  bool exist_loop_closure2 = tmp_diff > LoopClusureDistanceThreshold ? true : false;
                  if(exist_loop_closure2) {
                    reflector_map.Modify(ref.second.id(), ref.second.x(), ref.second.y());
                    tmp_map.Modify(ref.second.id(), ref.second.x(), ref.second.y());
                    int tmp_submap_id = processed_corresponding_submap_info_when_ref_first_created_[ref.second.id()].first;
                    processed_corresponding_submap_info_when_ref_first_created_[ref.second.id()].second = submap_origins_tmp[tmp_submap_id];
                    corresponding_submap_info_when_ref_first_created_[ref.second.id()].second = submap_origins_tmp[tmp_submap_id];
                    ROS_INFO("reflector_map.Modify(): ref_id: %d, tmp_diff: %f, old xy: (%f, %f), new xy: (%f, %f)", 
                              ref.second.id(), tmp_diff, tmp_ref.second.x(), tmp_ref.second.y(), ref.second.x(), ref.second.y());
                    break;
                  }
                }
              }
            }
  
            //remove repeated ref after loop closure
            if(reflector_map.GetMap().find(ref_id) != reflector_map.GetMap().end()){
              for(auto ref : reflector_map.GetMap()) {
                if(ref.second.id() != ref_id) {
                  double tmp_diff = std::hypot(reflector_map.GetMap()[ref_id].x() - ref.second.x(), reflector_map.GetMap()[ref_id].y() - ref.second.y());
                  if(tmp_diff < DistanceThresholdBetweenRefs2) {
                    reflector_map.DeleteReflector(ref.second.id());
                    reflector_map_modified_by_loop_closure_.DeleteReflector(ref.second.id());
                    if(processed_corresponding_submap_info_when_ref_first_created_.count(ref.second.id()) != 0){
                      processed_corresponding_submap_info_when_ref_first_created_.erase(ref.second.id());
                    }
                    
                    ROS_INFO("reflector_map.Remove(): ref_id: %d, remove repeated ref_id: %d, (%f, %f),tmp_diff: %f", 
                                    ref_id, ref.second.id(), ref.second.x(), ref.second.y(), tmp_diff);
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

std::shared_ptr<Pose> ReflectorLocation::GetPose(Pose start_pose, double best_score) {
  current_carto_pose_ = start_pose;
  ROS_INFO_THROTTLE(1.0, "1.start_pose: %f, %f, %f", start_pose.x(), start_pose.y(), angles::to_degrees(start_pose.theta()));
  
  if(run_model == RunModelType::PURE_LOCATION) {
    if(TimeOut(check_map_file_changed_time_, 1.0)){
      check_map_file_changed_time_ = ros::Time::now();
      if(reflector_map.CheckMapFileChanged()){
        // reflector_map.LoadMapFromFile("");
        LoadMap();
      }
      ROS_DEBUG("check_map_file_changed_time in once!");
    }
  }
  if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE) {
    mapping_using_pose_ = start_pose;
  }else if(using_pose_order == UsingPoseOrder::USE_REF_POSE) {
    mapping_using_pose_ = Pose(predict_pose_.x(), predict_pose_.y(), predict_pose_.theta());
  }
  
  reflector_points.clear();
  if(scan_reflector_points_.size() > 0){
    reflector_points.assign(scan_reflector_points_.begin(), scan_reflector_points_.end());
  }
  
  PublshAllReflectorDisplayInfo();
  if(run_model == RunModelType::CREATE_NEW_REF_MAP || run_model == RunModelType::APPEND_EXIST_REF_MAP){
    if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE){
      PublshAllLoopClosureReflectorDisplayInfo();
    }
  }

  ROS_DEBUG_THROTTLE(1.0, "predict_pose: (%f, %f, %f), mapping_order: %d, run_model: %d, using_pose_order: %d, location_mode:%d, matched_reflectors.size: %d", 
      predict_pose_.x(), predict_pose_.y(), angles::to_degrees(predict_pose_.theta()), mapping_order, run_model, using_pose_order, 
      location_mode, matched_reflectors_.size());
  if (location_mode == LocationMode::STATIC_LOCATION) {
    ROS_INFO("-------------------LocationMode::STATIC_LOCATION-------------------------");
    delta_time = ros::Time::now();
    filter_pose_deque_.clear();

    static int cnt = 0;
    cnt++;

    // 建图：计数 稳定帧
    if (run_model == RunModelType::CREATE_NEW_REF_MAP || run_model == RunModelType::APPEND_EXIST_REF_MAP) {
      ROS_INFO("RunModelType::CREATE_NEW_REF_MAP RunModelType::APPEND_EXIST_REF_MAP -----------------");

      reflector_map.ClearMap();
      tmp_map.ClearMap();
      reflector_map_modified_by_loop_closure_.ClearMap();

      if(cnt > 10) {
        Pose laser_pose = RobotPoseToLaserPose(start_pose);
        ROS_WARN("1.laser_pose: (%f, %f, %f)", laser_pose.x(), laser_pose.y(), angles::to_degrees(laser_pose.theta()));
        std::map<int, std::vector<double>> submap_origins_tmp = get_submap_origins();
        if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE){
          if (submap_origins_tmp.empty()) {
            ROS_WARN("wait for submap_origins_ ...");
            return std::make_shared<Pose>(start_pose);
          }
        }
        
        ReflectorstoMapPoints(laser_pose, reflector_points, true);      
        
        if(tmp_map.GetMap().size() < 3){
          ROS_WARN("tmp_map.GetMap().size < 3, not enough");

          return std::make_shared<Pose>(start_pose);
        }

        reflector_map.AddMap(tmp_map);
        
        if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE) {
          for (auto ref : tmp_map.GetMap()) {
            if(!submap_origins_tmp.empty()) {
              int submap_id = submap_origins_tmp.rbegin()->first;
              corresponding_submap_info_when_ref_first_created_[ref.second.id()] = {submap_id, submap_origins_tmp[submap_id]};
              ROS_INFO("record submap %d origin4 (%f, %f, %f), when add a new ref %d", submap_id, 
                submap_origins_tmp[submap_id][0], submap_origins_tmp[submap_id][1], submap_origins_tmp[submap_id][2], ref.second.id());

              processed_corresponding_submap_info_when_ref_first_created_[ref.second.id()] = corresponding_submap_info_when_ref_first_created_[ref.second.id()];
              ROS_INFO("move tmp3 ref_col %d to all ref_col %d", ref.second.id(), ref.second.id());
            }else {
              break;
            }
          }
        }
        
        if ((using_pose_order == UsingPoseOrder::USE_CARTO_POSE && !submap_origins_tmp.empty()) || using_pose_order == UsingPoseOrder::USE_REF_POSE) {
          SaveNewMapToFile();
          // run_model = RunModelType::APPEND_EXIST_REF_MAP;
          location_mode = LocationMode::DYNAMIC_LOACTION;
  
          State_ init_state;
          init_state << start_pose.x(), start_pose.y(), start_pose.theta();
          ekf_.init(init_state);
          cnt = 0;
        }

        return std::make_shared<Pose>(start_pose);
      }
      return std::make_shared<Pose>(start_pose);
    }

    ROS_INFO("Start static match..., cnt: %d", cnt);

    if(flag_use_file_pose_){
      Pose file_pose(0.0, 0.0, 0.0);

      AcquireFilePoseFunction(file_pose);
      
      if(cnt > 10){
        location_mode = LocationMode::DYNAMIC_LOACTION;
        last_ref_locate_pose_ = file_pose;
        cnt = 0;
        flag_use_file_pose_ = false;
      }
      
      State_ init_state;

      init_state << file_pose.x(), file_pose.y(), file_pose.theta();
      ROS_WARN("ReadInitialPoseFromTxt file pose1(%f, %f, %f)", file_pose.x(), file_pose.y(), angles::to_degrees(file_pose.theta()));
      ekf_.init(init_state);

      return std::make_shared<Pose>(file_pose);
    }

    static_count_ = static_count_ % int(pts_zone.size());
    // auto poses = MatchReflector(reflector_points, reflector_map.ToPointSet());
    auto poses = MatchReflector(reflector_points,  pts_zone.at(static_count_));
    if (poses.empty()) {
      ROS_ERROR("match ERROR reflector map...");
      static_count_++;
      static_err_count_++;
      if(static_err_count_ > 100 && best_score > 0.4){
        State_ init_state;
        init_state << start_pose.x(), start_pose.y(), start_pose.theta();
        ekf_.init(init_state);
        static_err_count_ = 0;

        if(cnt > 10){
          location_mode = LocationMode::DYNAMIC_LOACTION;
          last_ref_locate_pose_ = start_pose;
          cnt = 0;
        }

        ROS_ERROR("use carto init ref pose 01 (%f, %f, %f).....", start_pose.x(), start_pose.y(), 
                                                                  angles::to_degrees(start_pose.theta()));
        return std::make_shared<Pose>(start_pose);
      }
    } else if (poses.size() == 1) {
      auto laser_pose = poses.back();
      ROS_INFO("poses.size() == 1: laser_pose: (%f, %f, %f)", laser_pose.x(), laser_pose.y(), angles::to_degrees(laser_pose.theta()));
      Pose robot_pose = LaserPoseToRobotPose(laser_pose);
      ROS_INFO("poses.size() == 1: robot_pose: (%f, %f, %f)", robot_pose.x(), robot_pose.y(), angles::to_degrees(robot_pose.theta()));

      if(cnt > 10){
        location_mode = LocationMode::DYNAMIC_LOACTION;
        last_ref_locate_pose_ = robot_pose;
        cnt = 0;
      }

      float diff_dist = std::hypot(robot_pose.x()- start_pose.x(), robot_pose.y()-start_pose.y());
      if(start_pose.x() != 0.0 && start_pose.y() != 0.0 && diff_dist > option.match_sys_diff_dist && best_score > 0.4) {
        location_mode = LocationMode::STATIC_LOCATION;
        static_count_++;
        static_err_count_++;

        State_ init_state;
        init_state << start_pose.x(), start_pose.y(), start_pose.theta();
        ekf_.init(init_state);

        ROS_ERROR("STATIC Match ERROR----diff_dist > %f", option.match_sys_diff_dist);
        last_ref_locate_pose_ = start_pose;

        return std::make_shared<Pose>(start_pose);
      }
      State_ init_state;
      init_state << robot_pose.x(), robot_pose.y(), robot_pose.theta();
      ekf_.init(init_state);

      return std::make_shared<Pose>(robot_pose);
    } else {
      if(poses.size() == 2) {
        Pose pose1 = LaserPoseToRobotPose(poses.at(0));
        Pose pose2 = LaserPoseToRobotPose(poses.at(1));

        float dist = std::hypot(pose1.x() - pose2.x(), pose1.y() - pose2.y());
        float diff_theta = fabs(pose1.theta() - pose2.theta());
        if(dist < 0.15f && diff_theta < 0.1) {
            State_ init_state;
            Pose pose_avg = Pose((pose1.x() + pose2.x())/2.0f, (pose1.y() + pose2.y())/2.0f, pose1.theta());
            init_state << pose_avg.x(), pose_avg.y(), pose_avg.theta();
            ekf_.init(init_state);
            
            ROS_WARN("STATIC get 2 pose, use avg pose(%f, %f, %f)", pose_avg.x(), pose_avg.y(), angles::to_degrees(pose_avg.theta()));
            if(cnt > 10){
              location_mode = LocationMode::DYNAMIC_LOACTION;
              cnt = 0;
              last_ref_locate_pose_ = pose_avg;
            }
            return std::make_shared<Pose>(pose_avg);
        }else{
          if(flag_use_file_pose_){
            Pose file_pose(0.0, 0.0, 0.0);

            AcquireFilePoseFunction(file_pose);

            if(cnt > 10){
              location_mode = LocationMode::DYNAMIC_LOACTION;
              last_ref_locate_pose_ = file_pose;
              cnt = 0;
            }
            State_ init_state;
            init_state << file_pose.x(), file_pose.y(), file_pose.theta();
            ekf_.init(init_state);

            ROS_WARN("ReadInitialPoseFromTxt file pose2(%f, %f, %f)", file_pose.x(), file_pose.y(), angles::to_degrees(file_pose.theta()));
            return std::make_shared<Pose>(file_pose);
          }
        }
      }else{
        if(best_score > 0.4) {
          State_ init_state;
          init_state << start_pose.x(), start_pose.y(), start_pose.theta();
          ekf_.init(init_state);
          static_err_count_ = 0;
          if(cnt > 10){
            location_mode = LocationMode::DYNAMIC_LOACTION;
            cnt = 0;
            last_ref_locate_pose_ = start_pose;
          }

          ROS_ERROR("use carto init ref pose 02 (%f, %f, %f).....", start_pose.x(), start_pose.y(), 
                                                                    angles::to_degrees(start_pose.theta()));
          return std::make_shared<Pose>(start_pose);
        }
      }

      static_count_++;
      static_err_count_++;
    }
  } else {
    ROS_DEBUG_THROTTLE(1.0, "---------------------LocationMode::DYNAMIC_LOACTION-----------------------");
    // 建图： 添加未匹配反光板
    if ((run_model == RunModelType::CREATE_NEW_REF_MAP || run_model == RunModelType::APPEND_EXIST_REF_MAP) && switch_map_index_ == 0) {
      //lyy: 回环会使得子图原点发生变化，从而使得反光柱位置和子图特征对应不上了，需要调整反光柱位置
      // mode selection: 
      // 0 - pure localization
      // 1 - build new ref map simultaneously with carto mapping 
      // 2 - build new ref map use ref pose
      // 3 - build new ref map use carto pose
      // 4 - append existing ref map simultaneously with carto mapping
      // 5 - append existing ref map use ref pose
      // 6 - append existing ref map use carto pose
      // 99 - finish mapping, only used by service name "/mapping_order"
      if (using_pose_order == UsingPoseOrder::USE_CARTO_POSE){
        //mode 1 3 4 6
        ReflectorstoMapPoints(RobotPoseToLaserPose(mapping_using_pose_), reflector_points, false);
        UpdateMap(mapping_using_pose_);

        //使用carto pose建图，包括有反、无反同时建图和分别建图，都用到了子图原点，需要刷新相关变量，最终地图保存在reflector_map_modified_by_loop_closure_中，写入到xml文件里
        AdjustMapFromLoopClosure();
      }

      //mode 2
      if(run_model == RunModelType::CREATE_NEW_REF_MAP && using_pose_order == UsingPoseOrder::USE_REF_POSE) {
        ReflectorstoMapPoints(RobotPoseToLaserPose(mapping_using_pose_), reflector_points, false);
        UpdateMap(mapping_using_pose_);
      }

      std::unique_lock<std::mutex> lock(landmark_pts_mutex_);
      loop_closure_pts_for_match_landmark_.first = reflector_map_modified_by_loop_closure_.ToIds();
      loop_closure_pts_for_match_landmark_.second = reflector_map_modified_by_loop_closure_.ToPointSet();
      lock.unlock();
    }

    matched_reflectors_ = DynamicMatch(predict_pose_, std::move(reflector_points));

    static int warn_cnt = 0;
    if (matched_reflectors_.size() < option.dynamic_match_min_ref_num) {
      if(run_model != RunModelType::CREATE_NEW_REF_MAP && run_model != RunModelType::APPEND_EXIST_REF_MAP){
        warn_cnt++;
        ROS_ERROR("1.dynamic failed ...");
        ROS_ERROR("matched_reflectors.size: %d < %d", matched_reflectors_.size(), option.dynamic_match_min_ref_num);

        State_ init_state;
        init_state << predict_pose_.x(), predict_pose_.y(), predict_pose_.theta();

        if (warn_cnt < 3) {
          location_mode = LocationMode::DYNAMIC_LOACTION;
          ekf_.init(init_state);
        } else {
          location_mode = LocationMode::STATIC_LOCATION;
          ROS_ERROR("2.dynamic failed ..., warn_cnt: %d", warn_cnt);
          filter_pose_deque_.clear();
        }
        predict_pose_ = init_state;

        return std::make_shared<Pose>(Pose(predict_pose_.x(), predict_pose_.y(), predict_pose_.theta()));
      }
    }else{
      warn_cnt = 0;
    }

    // update every matched rf  get the pose
    predict_pose_.theta() = angles::normalize_angle(predict_pose_.theta());
    for (auto rf : matched_reflectors_) {
      ReflectorMeasurement<double> rf_measurement;
      rf_measurement << rf.x(), rf.y(), 0., 0.;
      position_measurement_model_.SetLandmark(rf_measurement);
      PositionMeasurement_ pm;
      pm << rf.mx(), rf.my();
      predict_pose_ = ekf_.update(position_measurement_model_, pm);
      predict_pose_.theta() = angles::normalize_angle(predict_pose_.theta());
      ROS_DEBUG_THROTTLE(1.0, "[update]: (%f, %f, %f)", predict_pose_.x(), predict_pose_.y(), angles::to_degrees(predict_pose_.theta()));
    }

    float carto_diff = std::hypot(predict_pose_.x() - start_pose.x(), predict_pose_.y() - start_pose.y());
    float last_ref_diff_dist = std::hypot(predict_pose_.x()- last_ref_locate_pose_.x(), predict_pose_.y()-last_ref_locate_pose_.y());
    float last_ref_diff_angle = angles::normalize_angle(predict_pose_.theta() - last_ref_locate_pose_.theta());

    if(best_score > 0.4 && matched_reflectors_.size() < 4 && (
                carto_diff > 0.5 || last_ref_diff_dist > 0.5f || last_ref_diff_angle > angles::from_degrees(3.0) )){
      ref_locate_pose_ = Pose(start_pose.x(),start_pose.y(),start_pose.theta());
      last_ref_locate_pose_ = ref_locate_pose_;
      State_ init_state;
      init_state << start_pose.x(), start_pose.y(), start_pose.theta();
      ekf_.init(init_state);
      location_mode = LocationMode::DYNAMIC_LOACTION;
      ROS_ERROR("use carto init ref pose 04....., carto_diff: %f, matched_reflectors.size(): %d, last_ref_diff_dist: %f, last_ref_diff_angle: %f, using (%f, %f, %f)",
                carto_diff, matched_reflectors_.size(), last_ref_diff_dist, last_ref_diff_angle, start_pose.x(), start_pose.y(), angles::to_degrees(start_pose.theta()));
      filter_pose_deque_.clear();//lyy

      predict_pose_ = init_state;
    }

    filter_pose_deque_.push_back(Pose(predict_pose_.x(),predict_pose_.y(),predict_pose_.theta()));
    int filter_size = 3;
    if(fabs(control_.v()) < 0.1 && fabs(control_.w()) < 0.06) {
        filter_size = 5;
    }
    if(control_.v() == 0 && control_.w() == 0) {
        filter_size = 10;
    }


    while(filter_pose_deque_.size() > filter_size){
        filter_pose_deque_.pop_front();
    }

    double sum_x = 0.0, sum_y = 0.0;
    Eigen::Quaterniond avg_quat(Eigen::AngleAxisd(filter_pose_deque_.front().theta(), Eigen::Vector3d::UnitZ()));
    int size = filter_pose_deque_.size();
    for(int i=0;i<size; i++) {
      sum_x += filter_pose_deque_.at(i).x();
      sum_y += filter_pose_deque_.at(i).y();

      Eigen::Quaterniond tmp_quat(Eigen::AngleAxisd(filter_pose_deque_.at(i).theta(), Eigen::Vector3d::UnitZ()));
      avg_quat = avg_quat.slerp(1.0/size, tmp_quat);
    }
    double avg_theta = avg_quat.normalized().toRotationMatrix().eulerAngles(0, 1, 2)[2];
    if(size > 0){
      ref_locate_pose_ = Pose(sum_x/size, sum_y/size, avg_theta);
    }else{
      ref_locate_pose_ = Pose(predict_pose_.x(),predict_pose_.y(),predict_pose_.theta());
    }
    
    last_ref_locate_pose_ = ref_locate_pose_;

    static_err_count_ = 0;

    ROS_DEBUG_THROTTLE(1.0, "---update_pose:(%f, %f, %f)", ref_locate_pose_.x(), ref_locate_pose_.y(), angles::to_degrees(ref_locate_pose_.theta()));

    return std::make_shared<Pose>(ref_locate_pose_);
  }
  ROS_DEBUG("GetPose: reach last return");
  return std::make_shared<Pose>(mapping_using_pose_);
}

MatchedReflectorList ReflectorLocation::DynamicMatch(
    const State_& predict_pose, PointSet&& reflector_scan_data) {
  MatchedReflectorList matched_rf;
  std::vector<std::pair<double, MatchedReflector>> mf_vect;

  if(reflector_scan_data.empty()) return matched_rf;

  auto ref = reflector_scan_data.begin();
  PointSet convert_to_map_point;
  for (; ref != reflector_scan_data.end(); ref++) {
    ReflectorMeasurement<double> rm;
    rm << ref->x(), ref->y(), 0., 0.;
    position_measurement_model_.SetLandmark(rm);

    // 将扫描的反光板位置根据当前预测pose转换为地图中的反光柱位置
    PositionMeasurement_ pm = position_measurement_model_.h(predict_pose);
    convert_to_map_point.push_back(Point(pm.x(), pm.y()));

    // 在反光柱地图中遍历查找扫描到的反光柱 找到放到matched_rf中
    for (auto guide : reflector_map.GetMap()) {
      auto dist = std::hypot(pm.x() - guide.second.x(), pm.y() - guide.second.y());

      if (dist < option.diff_ref_dist_thrd) {
        MatchedReflector mf(rm.x(), rm.y(), guide.second.x(), guide.second.y());
        mf_vect.push_back(std::make_pair(dist, mf));
        ROS_DEBUG_THROTTLE(1.0, "-----match new reflector, diff_ref_dist_thrd: %f, diff dist: %f, raw point(%f, %f), predict pm(%f, %f), map pm(%f, %f)",
                 option.diff_ref_dist_thrd, dist, ref->x(), ref->y(), pm.x(), pm.y(), guide.second.x(), guide.second.y());
        break;
      }
    }
  }

  std::sort(mf_vect.begin(), mf_vect.end(), [](const std::pair<double, MatchedReflector>& a, const std::pair<double, MatchedReflector>& b) {
    return a.first < b.first;
  });

  double sum_dist = 0.;
  int count = 0;
  for (const auto& pair : mf_vect) {
    sum_dist += pair.first;
    count++;
  }

  if(count == 0) return matched_rf;

  double aver_dist = sum_dist / count;
  ROS_DEBUG_THROTTLE(1.0, "new scan ref aver_dist: %f", aver_dist);

  std::vector<float> rate_vect;
  for(int i=0;i<mf_vect.size();i++) {
      float diff_rate = mf_vect.at(i).first / aver_dist;
      rate_vect.push_back(diff_rate);
      ROS_DEBUG_THROTTLE(1.0, "match ref dist: %f, [%d], rate: %f", mf_vect.at(i).first, i, diff_rate);
  }

  double rate_thresh = 3.0;
  for(int i=0;i<mf_vect.size();i++) {
    if(rate_vect.at(i) > rate_thresh && mf_vect.at(i).first > 0.1f*rate_thresh) continue;
    matched_rf.push_back(mf_vect.at(i).second);
  }

  if (matched_rf.size() < 3) {
    ROS_INFO("---------------------------------------------");
    for (auto rf : matched_rf) {
      ROS_INFO("matched: (%f, %f)", rf.x(), rf.y());
    }
    ROS_INFO("---------------------------------------------");
    for (auto point : reflector_scan_data) {
      ROS_INFO("scaned point:(%f, %f)", point.x(), point.y());
    }
    ROS_INFO("---------------------------------------------");
    for (auto pt : convert_to_map_point) {
      ROS_INFO("to map point:(%f, %f)", pt.x(), pt.y());
    }
  }

  //mode 5
  if (ref == reflector_scan_data.end() && run_model == RunModelType::APPEND_EXIST_REF_MAP) {
    // Pose pose(predict_pose.x(), predict_pose.y(), predict_pose.theta());
    if(using_pose_order == UsingPoseOrder::USE_REF_POSE && switch_map_index_ == 0) {
      auto pt_set = reflector_scan_data;
      // ReflectorstoMapPoints(RobotPoseToLaserPose(mapping_using_pose_), pt_set, false, true);
      ReflectorstoMapPoints(RobotPoseToLaserPose(mapping_using_pose_), pt_set, false);
      UpdateMap(mapping_using_pose_);
    }
  }
  PublshMatchedReflectorDisplayInfo(convert_to_map_point);

  return matched_rf;
}

void ReflectorLocation::ReflectorstoMapPoints(Pose pose, PointSet ref,
                                              bool first_scan, bool add_map) {
  static Pose last_count_pose(pose);
  double dist = std::hypot(last_count_pose.x() - pose.x(), last_count_pose.y() - pose.y());

  if (!first_scan && dist < 0.05) return;

  last_count_pose = pose;
  ROS_WARN_STREAM("==============ReflectorstoMapPoints2==============");
  for (std::size_t i = 0; i < ref.size(); i++) {
    double theta = pose.theta();
    // reflector坐标系转换 车体坐标系=>世界坐标
    double x = ref[i].x() * std::cos(theta) - ref[i].y() * std::sin(theta) + pose.x();
    double y = ref[i].x() * std::sin(theta) + ref[i].y() * std::cos(theta) + pose.y();

    bool new_ref = true;
    for (auto tmp : reflector_map.GetMap()) {
      // 相同反光柱更新
      if (std::hypot(tmp.second.x() - x, tmp.second.y() - y) < DistanceThresholdBetweenRefs2) {
        // tmp_map.Update(tmp.second.id());
        tmp_map.Add(x, y, tmp.second.id());
        new_ref = false;
      }
      // 近距离抛弃
      if (std::hypot(tmp.second.x() - x, tmp.second.y() - y) < DistanceThresholdBetweenRefs2) {
        new_ref = false;
        break;
      }
    }
    
    // 不同反光柱-添加
    if (new_ref) {
      int id = tmp_map.Add(x, y);
      if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE) {
        std::map<int, std::vector<double>> submap_origins_tmp = get_submap_origins();
        if(!submap_origins_tmp.empty()) {
          int submap_id = submap_origins_tmp.rbegin()->first;
          corresponding_submap_info_when_ref_first_created_[id] = {submap_id, submap_origins_tmp[submap_id]};
          ROS_INFO("record submap %d origin3 (%f, %f, %f), when add a new ref %d", submap_id, 
            submap_origins_tmp[submap_id][0], submap_origins_tmp[submap_id][1], submap_origins_tmp[submap_id][2], id);
        }
      }

      if (add_map) {
        reflector_map.Add(x, y, id);
      }
    }
  }
#if 0
  ROS_INFO_STREAM(
      "[ReflectorstoMapPoint2]: reflector_map size: " << reflector_map.GetMap().size());
  for (auto tmp : reflector_map.GetMap()) {
    ROS_DEBUG_STREAM("[ReflectorstoMapPoint2] reflector_map: ("
                     << tmp.second.x() << "," << tmp.second.y()
                     << "),count:" << tmp.second.count());
  }
#endif
}

void ReflectorLocation::ReflectorstoMapPoints(Pose pose, PointSet ref,
                                              bool first_scan) {
  static Pose last_count_pose(pose);
  double dist = std::hypot(last_count_pose.x() - pose.x(), last_count_pose.y() - pose.y());

  if (!first_scan && dist < 0.05) return;

  last_count_pose = pose;
  ROS_WARN_STREAM("==============ReflectorstoMapPoints==============");

  for (std::size_t i = 0; i < ref.size(); i++) {
    double theta = pose.theta();
    // reflector坐标系转换 车体坐标系=>世界坐标
    double x = ref[i].x() * std::cos(theta) - ref[i].y() * std::sin(theta) + pose.x();
    double y = ref[i].x() * std::sin(theta) + ref[i].y() * std::cos(theta) + pose.y();

    if (x == 0.0 && y == 0.0) {
      continue;
    }
    
    std::map<int, std::vector<double>> submap_origins_tmp = get_submap_origins();
    if (tmp_map.GetMap().empty()) {
      int id = tmp_map.Add(x, y);
      if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE){
        if(!submap_origins_tmp.empty()) {
          int submap_id = submap_origins_tmp.rbegin()->first;
          corresponding_submap_info_when_ref_first_created_[id] = {submap_id, submap_origins_tmp[submap_id]};
          ROS_INFO("record submap %d origin1 (%f, %f, %f), when add a new ref %d", submap_id, 
            submap_origins_tmp[submap_id][0], submap_origins_tmp[submap_id][1], submap_origins_tmp[submap_id][2], id);
        }
      }
    } else {
      bool new_ref = true;
      for (auto tmp : tmp_map.GetMap()) {
        // 相同反光柱更新
        if (std::hypot(tmp.second.x() - x, tmp.second.y() - y) < DistanceThresholdBetweenRefs2) {
          tmp_map.Add(x, y, tmp.second.id());
          new_ref = false;
          // ROS_DEBUG_STREAM("update common reflector..");
        }
        // 近距离抛弃
        if (std::hypot(tmp.second.x() - x, tmp.second.y() - y) < DistanceThresholdBetweenRefs2) {
          new_ref = false;
          break;
        }
      }
      // 不同反光柱-添加
      if (new_ref) {
        int id = tmp_map.Add(x, y);
        if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE) {
          if(!submap_origins_tmp.empty()){
            int submap_id = submap_origins_tmp.rbegin()->first;
            corresponding_submap_info_when_ref_first_created_[id] = {submap_id, submap_origins_tmp[submap_id]};
            ROS_INFO("record submap %d origin2 (%f, %f, %f), when add a new ref %d", submap_id, 
              submap_origins_tmp[submap_id][0], submap_origins_tmp[submap_id][1], submap_origins_tmp[submap_id][2], id);
          }
        }
      }
    }
  }

#if 0
  ROS_INFO_STREAM(
      "[ReflectorstoMapPoint]: tmp_map size: " << tmp_map.GetMap().size());
  for (auto tmp : tmp_map.GetMap()) {
    ROS_DEBUG_STREAM("[ReflectorstoMapPoint] tmp: ("
                     << tmp.second.x() << "," << tmp.second.y()
                     << "),count:" << tmp.second.count());
  }
#endif
}

void ReflectorLocation::UpdateMap(Pose pose) {
  // 间断更新要使用的地图
  static ros::Time update_time = ros::Time::now();
  if (ros::Time::now() - update_time < ros::Duration(1.0)) return;
  ROS_DEBUG_STREAM("==============UpdateMap==============");
  update_time = ros::Time::now();

  for (int i = 1; i <= tmp_map.GetMap().size(); i++) {
    if(tmp_map.GetMap().count(i) == 0) continue;

    auto ref = tmp_map.GetMap().at(i);
    // ROS_INFO_STREAM("ref: " << ref.x() << "," << ref.y() << "," << ref.id()
    //                         << " count: " << ref.count());
    if (ref.count() >= RefInitPoseStableCountThreshold) {
      bool new_ref = true;
      for (auto map : reflector_map.GetMap()) {
        // 相同反光柱更新
        if (std::hypot(ref.x() - map.second.x(), ref.y() - map.second.y()) < DistanceThresholdBetweenRefs1) {
          reflector_map.Add(ref.x(), ref.y(), map.second.id());
          new_ref = false;
          // ROS_DEBUG_STREAM("update common reflector..");
        }

        // 近距离抛弃
        if (std::hypot(ref.x() - map.second.x(), ref.y() - map.second.y()) < DistanceThresholdBetweenRefs2) {
          new_ref = false;
          break;
        }
      }
      if(ref.x() == 0.0 && ref.y() == 0.0) {
        new_ref = false;
      }

      if (new_ref) {
        int id = reflector_map.Add(ref.x(), ref.y(), ref.id());
        if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE){
          processed_corresponding_submap_info_when_ref_first_created_[id] = corresponding_submap_info_when_ref_first_created_[ref.id()];
        }
        ROS_INFO("move tmp2 ref_col %d to all ref_col %d", ref.id(), id);
        ROS_INFO_STREAM("add new ref: " << ref.x() << "," << ref.y() << ","
                                        << ref.id());
      }
    }
  }

  if (reflector_map.GetMap().empty()) {
    ROS_ERROR_STREAM("reflector_map error");
  }
  // for (auto ref : reflector_map.GetMap()) {
  //   ROS_DEBUG_STREAM("reflector_map: [" << ref.second.id() << "]:("
  //                                       << ref.second.x() << ","
  //                                       << ref.second.y() << ")");
  // }
}

void ReflectorLocation::SaveNewMapToFile() {
  if (run_model == RunModelType::CREATE_NEW_REF_MAP) {
    // ReflectorMap save_map;
    // for (auto ref : reflector_map.GetMap()) {
    //   if (ref.second.id() == 0) continue;
    //   save_map.Add(ref.second.x(), ref.second.y());
    // }
    // save_map.SaveNewMapToFile("");
    if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE){
      reflector_map_modified_by_loop_closure_.SaveNewMapToFile("");
    }
    if(using_pose_order == UsingPoseOrder::USE_REF_POSE){
      reflector_map.SaveNewMapToFile("");
    }
  } else if (run_model == RunModelType::APPEND_EXIST_REF_MAP) {
    // tmp_map.AppendMapToFile("");
    if(using_pose_order == UsingPoseOrder::USE_CARTO_POSE){
      reflector_map_modified_by_loop_closure_.AppendMapToFile("");
    }
    if(using_pose_order == UsingPoseOrder::USE_REF_POSE){
      reflector_map.AppendMapToFile("");
    }
  } else {
    ROS_ERROR("SaveNewMapToFile: run_model error");
  }
}

// 激光pose转换为AGV pose
Pose ReflectorLocation::LaserPoseToRobotPose(Pose& laser_pose) {
  Eigen::Vector2d laser_coordinate(laser_pose.x(), laser_pose.y());
  Eigen::Rotation2D<double> laser_rotation(laser_pose.theta() - option.install_offset.theta);
  Eigen::Vector2d laser_installation(option.install_offset.x,
                                     option.install_offset.y);
  Eigen::Vector2d robot_pose =
      laser_coordinate - laser_rotation.toRotationMatrix() * laser_installation;

  return Pose(robot_pose.x(), robot_pose.y(), laser_pose.theta() - option.install_offset.theta);
}

// AGV pose 转换为激光pose
Pose ReflectorLocation::RobotPoseToLaserPose(Pose& robot_pose) {
  Eigen::Vector2d robot_coordinate(robot_pose.x(), robot_pose.y());
  Eigen::Rotation2D<double> rotation(robot_pose.theta());
  Eigen::Vector2d laser_installation(option.install_offset.x,
                                     option.install_offset.y);
  // ROS_DEBUG("option.install_offset: (%f, %f, %f)", option.install_offset.x, option.install_offset.y, angles::to_degrees(option.install_offset.theta));

  Eigen::Vector2d laser_pose =
      robot_coordinate + rotation.toRotationMatrix() * laser_installation ;
  return Pose(laser_pose[0], laser_pose[1], robot_pose.theta() + option.install_offset.theta);
}

void ReflectorLocation::PublshAllLoopClosureReflectorDisplayInfo() {
  visualization_msgs::Marker points;
  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  points.ns = "Reflector";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 1;

  // 设置marker类型
  points.type = visualization_msgs::Marker::POINTS;
  
  // 设置宽高
  points.scale.x = 0.6;
  points.scale.y = 0.6;
  // 设置颜色
  points.color.b = 0.0f;
  points.color.r = 1.0f;
  points.color.a = 1.0;

  if (reflector_map_modified_by_loop_closure_.GetMap().size() < 1) return;

  for (auto& mark : reflector_map_modified_by_loop_closure_.GetMap()) {
    geometry_msgs::Point p;
    p.x = mark.second.x();
    p.y = mark.second.y();
    p.z = 0.0;
    points.points.emplace_back(p);
  }

  all_loop_closure_reflector_pub_.publish(points);
}

void ReflectorLocation::PublshAllReflectorDisplayInfo() {
  visualization_msgs::Marker points;
  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  points.ns = "Reflector";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;

  // 设置marker类型
  points.type = visualization_msgs::Marker::POINTS;
  
  // 设置宽高
  points.scale.x = 0.5;
  points.scale.y = 0.5;
  // 设置颜色
  points.color.b = 1.0f;
  points.color.r = 1.0f;
  points.color.a = 1.0;

  visualization_msgs::MarkerArray text_maker_array;
  visualization_msgs::Marker text_maker;
  text_maker.header = points.header;
  text_maker.ns = "ReflectorText";
  text_maker.action = points.action;
  text_maker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_maker.scale.x = 0.5;
  text_maker.scale.y = 0.5;
  text_maker.scale.z = 0.5;

  if (reflector_map.GetMap().size() < 1) return;

  static int color_index = 0;
  for (auto& mark : reflector_map.GetMap()) {
    geometry_msgs::Point p;
    p.x = mark.second.x();
    p.y = mark.second.y();
    p.z = 0.0;
    points.points.emplace_back(p);

    color_index %= int(landmark_color_vec_.size());
    text_maker.color = landmark_color_vec_[color_index];
    color_index++;

    std::string text_suffix = "";
    bool found = false;
    for(int i=0; i<find_pts_landmark_.size() && (!found); i++) {
      for(int j=0; j<find_pts_landmark_[i].size() && (!found); j++) {
        if(std::hypot(p.x - find_pts_landmark_[i][j].x(), p.y - find_pts_landmark_[i][j].y()) < 0.1) {
          text_suffix = ", arr: " + std::to_string(i+1) + " - " + std::to_string(j+1);
          found = true;
          break;
        }
      }
    }

    double offset_y = 0.5;
    std::string show_text;
    show_text += "ID: " + std::to_string(mark.second.id()) + text_suffix;
    text_maker.pose.position.x = p.x;
    text_maker.pose.position.y = p.y + offset_y;
    text_maker.pose.position.z = p.z;
    text_maker.pose.orientation.w = 1.0;
    text_maker.id = mark.second.id();
    text_maker.text = show_text;
    text_maker_array.markers.emplace_back(text_maker);
  }
  all_reflector_pub_.publish(points);
  text_marker_array_pub_.publish(text_maker_array);
}

void ReflectorLocation::PublshMatchedReflectorDisplayInfo(
    PointSet landmarks_tmp) {
  visualization_msgs::Marker points;
  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  points.ns = "Reflector";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 2;
  // 设置marker类型
  points.type = visualization_msgs::Marker::POINTS;
  // 设置宽高
  points.scale.x = 0.4;
  points.scale.y = 0.4;
  // 设置颜色
  points.color.b = 1.0f;
  points.color.a = 1.0;
  if (landmarks_tmp.size() < 1) return;
  for (auto& mark : landmarks_tmp) {
    geometry_msgs::Point p;
    p.x = mark.x();
    p.y = mark.y();
    p.z = p.z;
    points.points.emplace_back(p);
  }
  matched_reflector_pub_.publish(points);
}

}  // namespace location