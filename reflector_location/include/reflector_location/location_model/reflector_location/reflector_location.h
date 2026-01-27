
#ifndef REF_LOC_NODE_INCLUDE_REFLECTOR_LOCATION_H
#define REF_LOC_NODE_INCLUDE_REFLECTOR_LOCATION_H

#include <angles/angles.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <deque>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>

#include "geometry.h"
#include "location_option.h"
#include "reflector_map.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include <tf/transform_datatypes.h>
#include "cartographer_ros_msgs/SubmapList.h"
#include <common_msgs/SetPose.h>
#include "cartographer_ros_msgs/LandmarkList.h"
#include "std_msgs/ColorRGBA.h"

namespace location {
  constexpr char AcquireFilePoseServiceName[] = "acquire_file_pose";
  constexpr char RefInSubmapQueryServiceName[] = "ref_in_submap_query";
  constexpr char SwitchMap[] = "switch_map";

class ReflectorLocation {
 public:
  ReflectorLocation();

  bool Init(LocationOption opt);

  bool UpdateOption(LocationOption opt);

  bool LoadMap();

  void GeneralRefZone();

  void AddOdomData(double v, double w);

  void HandleMappingOrder(const std_msgs::Int32& msg);

  void HandleTransRefMapPoint(const geometry_msgs::Vector3& msg);

  Point TransformRefMapPointWithPose(Point pt, Pose& robot_pose);

  std::shared_ptr<Pose> GetPose(Pose start_pose, double best_score);

  PointSet MapInfo();

  bool SetScanReflectorPoints(PointSet points) {
    std::unique_lock<std::mutex> lock(scan_reflector_points_mutex_);
    scan_reflector_points_.clear();
    if(points.size() > 0) scan_reflector_points_.assign(points.begin(), points.end());

    if (scan_reflector_points_.size() < 3) return false;

    return true;
  }

  PointSet GetScanReflectorPoints() {
    std::unique_lock<std::mutex> lock(scan_reflector_points_mutex_);
    
    return scan_reflector_points_;
  }

  PointSet GetActivePoints() { return reflector_map.ToPointSet(); }
  bool ReadInitialPoseFromTxt(Pose &init_pose);
  void AdjustMapFromLoopClosure();

 private:
  PointSet reflector_points;
  PointSet scan_reflector_points_;
  ReflectorOption option;
  bool bosch_init_;
  bool bosch_sync_;

  int static_count_;
  std::vector<PointSet> pts_zone;

  ReflectorMap reflector_map_modified_by_loop_closure_;
  ReflectorMap tmp_map;
  ReflectorMap reflector_map;

  std::vector<PointSet> find_pts;
  PointSet z_;
  volatile LocationMode location_mode;
  Control_ control_;
  State_ state_;
  SystemModel_ sys_model_;
  PositionMeasurementModel_ position_measurement_model_;
  Kalman::ExtendedKalmanFilter<State_> ekf_;

  ros::Time delta_time;
  State_ laser_install_offset;
  bool odom_init;
  RunModelType run_model;
  MappingOrder mapping_order;
  UsingPoseOrder using_pose_order;
  ros::Publisher all_loop_closure_reflector_pub_;
  ros::Publisher all_reflector_pub_;
  ros::Publisher text_marker_array_pub_;
  ros::Publisher matched_reflector_pub_;
  ros::Publisher matched_reflector_as_landmark_pub_;

  std::vector<ros::Subscriber> subscriblers_vec_;

  location::State_ predict_pose_;
  MatchedReflectorList matched_reflectors_;

  void SaveNewMapToFile();
  MatchedReflectorList DynamicMatch(const State_& predict_pose,
                                    PointSet&& reflector_scan_data);
  bool Match(PointSet&& m, PointSet&& n);
  std::vector<Pose> MatchReflector(PointSet m, PointSet n);
  bool MatchLandmark(PointSet&& map_pts, PointSet&& pts);
  // bool HandleMatchLandmark(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  void HandleMatchLandmark(const ::ros::TimerEvent& timer_event);

  Pose CalcPose(PointSet& f);
  bool AppendCandidates(const PointSet& n);
  bool AppendCandidatesLandmark(const PointSet& pts);
  void ReflectorstoMapPoints(Pose pose, PointSet ref, bool first_scan);
  void ReflectorstoMapPoints(Pose pose, PointSet ref, bool first_scan, bool add_map);
  void ReflectorstoMapPoints(Pose pose, PointSet ref);
  void UpdateMap(Pose pose);
  Pose LaserPoseToRobotPose(Pose& laser_pose);
  Pose RobotPoseToLaserPose(Pose& robot_pose);
  void PublshAllReflectorDisplayInfo();
  void PublshAllLoopClosureReflectorDisplayInfo();
  void PublshMatchedReflectorDisplayInfo(PointSet landmarks_tmp);
  void AcquireFilePoseFunction(Pose& file_pose);
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  bool HandleSwitchMap(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  std::vector<int> AcquireSubmapIdOfRefPose(double x, double y, int ref_id);
  bool TimeOut(ros::Time time, double duration_thresh){
    ros::Duration dur = ros::Duration(ros::Time::now() - time);
    if(dur.toSec() > duration_thresh){
      return true;
    }

    return false;
  }

  std::map<int, std::vector<double>> get_submap_origins() {
    std::unique_lock<std::mutex> lock(submap_origins_update_mutex_);
    return submap_origins_;
  }

  ros::NodeHandle nh_;
  ros::Time check_map_file_changed_time_;
  std::deque<Pose> filter_pose_deque_;
  int static_err_count_;
  Pose ref_locate_pose_;
  Pose last_ref_locate_pose_;
  Pose mapping_using_pose_;
  bool flag_use_file_pose_;
  std::vector<ros::ServiceServer> service_servers_vec_;
  ros::ServiceClient acquire_file_pose_client_;
  ros::ServiceClient acquire_ref_in_submap_client_;
  std::map<int, std::vector<double>> submap_origins_;
  std::map<int, std::pair<int, std::vector<double>>> corresponding_submap_info_when_ref_first_created_;
  std::map<int, std::pair<int, std::vector<double>>> processed_corresponding_submap_info_when_ref_first_created_;
  cartographer_ros_msgs::LandmarkList current_landmark_list_;
  std::vector<std_msgs::ColorRGBA> landmark_color_vec_;
  std::mutex scan_reflector_points_mutex_;
  std::mutex landmark_pts_mutex_;
  std::mutex submap_origins_update_mutex_;
  std::pair<std::vector<int>, PointSet> loop_closure_pts_for_match_landmark_;
  ros::Timer landmark_match_timer_;
  std::vector<PointSet> find_pts_landmark_;
  PointSet z_landmark_;
  Pose current_carto_pose_;
  int switch_map_index_;
};

}  // namespace location

#endif  // REF_LOC_NODE_INCLUDE_REFLECTOR_LOCATION_H