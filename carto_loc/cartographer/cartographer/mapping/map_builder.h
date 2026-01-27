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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"
//penghu*
#include "cartographer/common/config_file_operator.h"
#include "cartographer/common/file_interface.h"
#include "cartographer/logging/log.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary *const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder : public MapBuilderInterface {
 public:
  //penghu* add config_operator
  explicit MapBuilder(SlamCommon::CFileInterface *file_interface,
                      SlamCommon::CConfigFileOperator *config_operator,
                      const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;

  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();
  }

  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }

  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }

  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

  bool Setinitpose(SlamCommon::Pose3D pose) const override
  {
      // LOG_INFO("411 SET INIT POSE");
      auto* traj_ = MapBuilder::Gettraj();
      traj_->SetSeedPose(pose);
      return true;
  }

  //penghu 24/4/12
  static ::cartographer::mapping::LocalTrajectoryBuilder2D* Gettraj();

 private:
  SlamCommon::CFileInterface *m_file_interface = nullptr;
  SlamCommon::CConfigFileOperator *m_config_operator = nullptr;
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  //PoseGraph2D or PoseGraph3D
  std::unique_ptr<PoseGraph> pose_graph_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_; //保存了CollatedTrajectoryBuilder指针
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
  //penghu*
  int m_match_model_ = 1;

  //penghu 24/4/12
  static std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_;

  std::vector<cartographer::transform::Rigid2d> local_to_global_vec_;//lyy
  std::deque<cartographer::mapping::proto::Submap2D> submap2d_protos_;//lyy
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
