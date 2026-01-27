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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

#include "std_msgs/String.h"
#include "cartographer/common/file_interface.h"
#include "cartographer/carto_localization/initial_pose_locating/global_locator_2d.h"
#include <angles/angles.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cartographer/mapping_bridge/node_options.h>
#include <cartographer/mapping_bridge/trajectory_options.h>
#include <tf/transform_listener.h>
#include <common_msgs/SetPose.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_srvs/SetBool.h"

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const cartographer_ros::NodeOptions& node_options,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       tf2_ros::Buffer* tf_buffer, bool collect_metrics, SlamCommon::CFileInterface* file_interface);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();
  // Finishes a single given trajectory. Returns false if the trajectory did not
  // exist or was already finished.
  bool FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Returns unique SensorIds for multiple input bag files based on
  // their TrajectoryOptions.
  // 'SensorId::id' is the expected ROS topic name.
  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleAssignedSubmapId(const int trajectory_id,
                                const std::string& sensor_id,
                                const std_msgs::String::ConstPtr& msg);//lyy
  void HandleInitialpose(const int trajectory_id, const std::string& sensor_id,
                      const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  // Serializes the complete Node state.
  void SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  // Loads a serialized SLAM state from a .pbstream file.
  void LoadState(const std::string& state_filename, bool load_frozen_state);

  ::ros::NodeHandle* node_handle();

  //lyy
  void AddSubmapIdAndFlag(int submap_id, bool assigned_flag) {
    submap_id_ = submap_id;
    assigned_flag_ = assigned_flag;
  }

  void SetConfigurationDirectoryAndName(std::string dir, std::string name){
    configuration_directory_ = dir;
    configuration_basename_ = name;
    std::tie(node_options_global_locator_, trajectory_options_global_locator_) = 
                            CartoMapppingBridge::LoadOptions(configuration_directory_, configuration_basename_);
  }

  void ReadCartoLocYamls();

 private:
  struct Subscriber {
    ::ros::Subscriber subscriber;

    // ::ros::Subscriber::getTopic() does not necessarily return the same
    // std::string
    // it was given in its constructor. Since we rely on the topic name as the
    // unique identifier of a subscriber, we remember it ourselves.
    std::string topic;
  };

  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleTrajectoryQuery(
      ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
      ::cartographer_ros_msgs::TrajectoryQuery::Response& response);
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);
  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);
  bool HandleGetTrajectoryStates(
      ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
      ::cartographer_ros_msgs::GetTrajectoryStates::Response& response);
  bool HandleReadMetrics(
      cartographer_ros_msgs::ReadMetrics::Request& request,
      cartographer_ros_msgs::ReadMetrics::Response& response);
  bool HandleSetCartoInitpose(common_msgs::SetPose::Request& request, common_msgs::SetPose::Response& response);
  bool HandleUsingPredictPose(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);
  bool HandleRefInSubmapQuery(common_msgs::SetPose::Request& request, common_msgs::SetPose::Response& response);

  // Returns the set of SensorIds expected for a trajectory.
  // 'SensorId::id' is the expected ROS topic name.
  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  int AddTrajectory(const TrajectoryOptions& options);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event);
  void PublishBestScoreSubmapId(const ::ros::TimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&);

  // Helper function for service handlers that need to check trajectory states.
  cartographer_ros_msgs::StatusResponse TrajectoryStateToStatus(
      int trajectory_id, const std::set<cartographer::mapping::PoseGraphInterface::TrajectoryState>& valid_states);

  const cartographer_ros::NodeOptions node_options_;
  CartoMapppingBridge::NodeOptions node_options_global_locator_;
  CartoMapppingBridge::TrajectoryOptions trajectory_options_global_locator_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer* tf_buffer_;

  absl::Mutex mutex_;
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher landmark_poses_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  ::ros::Publisher tracked_pose_publisher_;
  ::ros::Publisher best_data_publisher_;
  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<::ros::ServiceServer> service_servers_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  // These are keyed with 'trajectory_id'.
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::map<int, ::ros::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;

  // The timer for publishing local trajectory data (i.e. pose transforms and
  // range data point clouds) is a regular timer which is not triggered when
  // simulation time is standing still. This prevents overflowing the transform
  // listener buffer by publishing the same transforms over and over again.
  ::ros::Timer publish_local_trajectory_data_timer_;
  ::ros::Timer publish_best_score_;

  int submap_id_;
  bool assigned_flag_;
  SlamCommon::CFileInterface* file_interface_;
  cartographer::sensor::PointCloud init_pointcloud_;
  int init_trajectory_id_;// localization traj ID default is 1
  std::string configuration_directory_;
  std::string configuration_basename_;

  double initial_scan_match_linear_window_;
  double initial_scan_match_min_score_threshold_;
  double initial_scan_match_confidence_score_threshold_;
  double initial_scan_match_min_time_;
  bool initial_use_ceres_to_refine_;

  double scan_match_min_score_threshold_;
  double scan_match_confidence_score_threshold_;
  bool use_ceres_to_refine_;
  double  scan_match_linear_window_;
  double  scan_match_angle_window_;
  double  scan_match_min_time_;
  bool is_appy_;
  bool is_one_scan_;

  bool yaw_con_;
  bool precison_;
  int num_submap_;

  int match_mode_;
  std::unique_ptr<Localization::CartoLocalization::GlobalLocator> global_locator_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
