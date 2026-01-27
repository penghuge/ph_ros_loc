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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/port.h"
#include "cartographer/common/rate_timer.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace mapping {

// Collates sensor data using a sensor::CollatorInterface, then passes it on to
// a mapping::TrajectoryBuilderInterface which is common for 2D and 3D.
//使用sensor::CollatorInterface整理传感器数据，
//然后将其传递给2D和3D通用的接口mapping::TrajectoryBuilderInterface
class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface {
 public:
  using SensorId = TrajectoryBuilderInterface::SensorId;

  CollatedTrajectoryBuilder(
      const proto::TrajectoryBuilderOptions& trajectory_options,
      sensor::CollatorInterface* sensor_collator, int trajectory_id,
      const std::set<SensorId>& expected_sensor_ids,
      std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder);
  ~CollatedTrajectoryBuilder() override {}

  CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;
  CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) =
      delete;

  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, imu_data));
  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    // LOG(INFO)<<"614_odom_data x = "<<odometry_data.pose.translation().x()<<"y = "<<odometry_data.pose.translation().y()<<
    //         "yaw = "<< odometry_data.pose.rotation().z();
    AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
  }

  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override {
    if (collate_fixed_frame_) {
      AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id,
                                               fixed_frame_pose_data);
  }

  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    //是否对地标数据进行排序整理
    if (collate_landmarks_) {
      AddData(sensor::MakeDispatchable(sensor_id, landmark_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id, landmark_data);
  }

  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    AddData(std::move(local_slam_result_data));
  }

 private:
  void AddData(std::unique_ptr<sensor::Data> data);

  void HandleCollatedSensorData(const std::string& sensor_id,
                                std::unique_ptr<sensor::Data> data);
  //lyy
  void AddSubmapIdAndFlag(int submap_id, bool assigned_flag) override {
    submap_id_ = submap_id;
    assigned_flag_ = assigned_flag;
  }

  //根据目前的参数设置，基类指针sensor_collator_指向Collator
  sensor::CollatorInterface* const sensor_collator_;
  const bool collate_landmarks_;
  const bool collate_fixed_frame_;
  const int trajectory_id_;
  //全局路径生成器，该接口是2D和3D的通用接口，根据外部传入是2D还是3D确定
  //该基类指针指向GlobalTrajectoryBuilder
  std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;

  // Time at which we last logged the rates of incoming sensor data.
  std::chrono::steady_clock::time_point last_logging_time_;
  std::map<std::string, common::RateTimer<>> rate_timers_;
  int submap_id_;//lyy
  bool assigned_flag_;//lyy
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
