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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/2d/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

//penghu*
#include "cartographer/mapping/internal/2d/scan_matching/global_scan_matcher_2d_local.h"
#include "cartographer/common/config_file_operator.h"
#include "cartographer/common/file_interface.h"
//penghu 24/4/19

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping_bridge/msg_conversion.h"
#include "cartographer/carto_mapping_interface/carto_mapping_data.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
//连接局部SLAM堆栈(即位姿估算器、扫描匹配等)而不需要闭环。
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalTrajectoryBuilder2D {
 public:
  //penghu*
    struct TestResult
    {
        global_scan_matcher_2d_local::Pose pose;
        sensor::PointCloud point_data;
        double use_time = 0.;
        bool flag = false;
    };
  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data; ////节点数据
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps; //插入的submap的向量
  };
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
     // 如果被MotionFilter滤掉了，那么insertion_result返回空指针
    //为避免每个子图插入太多扫描帧（scan）数据，一旦扫描匹配器生成了新的pose，
    //计算这次pose与last_pose 的变化量， 调用Motion Filter，
    //如果这次的变化不够重要（或者说变化很小），则扫描将被删除。
    //当pose的变化大于某个距离，角度或时间阈值时，才会将scan插入到当前子图中
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit LocalTrajectoryBuilder2D(
      SlamCommon::CFileInterface *file_interface,
      SlamCommon::CConfigFileOperator *config_operator,
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  void AddSubmapIdAndFlag(int submap_id, bool assigned_flag) {
      submap_id_ = submap_id;
      assigned_flag_ = assigned_flag;
  }

  float GetBestCandidateScore(){
      return m_init_global_scan_match->GetBestCandidateScore();
  }

  int GetMatchedSubmapId(){
    return m_init_global_scan_match->GetMatchedSubmapId();
  }

  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid3d& pose,
                                const common::Time time);
  bool SetSeedPose(SlamCommon::Pose3D seedpose);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

  static bool is_seed_;
  static SlamCommon::Pose3D seed_pose_;

  //penghu 24/4/21
  bool expand(SlamCommon::Pose3D &yun_pose);
  bool handdata( std::vector<std::uint16_t> & yuntai_data, sensor::RangeData &rangedata);
  std::vector<std::shared_ptr<const Submap2D>> ExpandSubmap(const sensor::RangeData& range_data_in_local);

  //penghu 24/4/11
  struct Data {
      proto::SubmapsOptions2D options;
      std::vector<std::shared_ptr<Submap2D>> submaps;
  };

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment,
      const absl::optional<common::Duration>& sensor_duration);
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  std::unique_ptr<InsertionResult> InsertSingleMapIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time /*time*/, const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  SlamCommon::CFileInterface *m_file_interface = nullptr;
  SlamCommon::CConfigFileOperator *m_config_operator = nullptr;
  const proto::LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;

  MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  std::unique_ptr<PoseExtrapolator> extrapolator_;
  //penghu*
  global_scan_matcher_2d_local *m_init_global_scan_match;
  SlamCommon::Time m_start_mach_time;

  bool m_use_ceres_to_refine = true;
  double m_initial_scan_match_min_score_threshold = 0.4;
  double m_initial_scan_match_confidence_score_threshold = 0.6;
  std::string m_state_filename = "";
  std::string m_map_name = "";
  double m_initial_scan_match_linear_window = 0.1;
  double m_initial_scan_match_angle_window = 0.1;
  double m_initial_scan_match_min_time = 10.;//unit: s
  SlamCommon::Pose3D m_best_pose;
  int m_match_model_ = 1;
  transform::Rigid3d m_init_pose;
  bool m_first_scan = true;
  mutable absl::Mutex mutex_;

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  absl::optional<double> last_thread_cpu_time_seconds_;
  absl::optional<common::Time> last_sensor_time_;

  RangeDataCollator range_data_collator_;

  SlamCommon::OccupancyGrid *m_occupancy_grid = nullptr;
  SlamCommon::ConnectStateEnum connect_status_ = SlamCommon::CONNSTATE_WAIT_FOR_CONNECT;

  bool m_stop_recv = true;
  std::vector<std::uint16_t> distance_data_; // distance_data的大小为1200
  bool start_ = false;
  SlamCommon::Pose3D init_pose_ = {0., 0., 0.};
  SlamCommon::Pose3D yuntai_pose_ = {0., 0., 0.};
  std::vector<std::shared_ptr<const Submap2D>> expand_result_;
  std::shared_ptr<const Submap> submap_;
  Data expand_option_;
  bool appy_ = false;
  bool one_scan_ = false;

  int submap_id_ = 0;//lyy
  bool assigned_flag_ = false;//lyy
  int first_scan_filter_count_ = 0;
  int start_expand_count_ = 0;//ph
  bool start_expand_ = true;//ph
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
