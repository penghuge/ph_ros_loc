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

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

//传感器数据频率打印周期，单位：s
constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      collate_landmarks_(trajectory_options.collate_landmarks()),
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
        
  submap_id_ = 0;//lyy
  assigned_flag_ = false;//lyy

  absl::flat_hash_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) {
      continue;
    }
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) {
      continue;
    }
    expected_sensor_id_strings.insert(sensor_id.id);
  }
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  // LOG(INFO)<<"614  for odom6";
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

//处理已经进行过时间排序的传感器数据的通用接口
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->GetTime());

  //达到打印时间，打印一下传感器频率
  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      // LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }
  data->AddSubmapIdAndFlag(submap_id_, assigned_flag_);//lyy

  //调用子类Dispatchable的方法，将传感器数据传入GlobalTrajectoryBuilder中进行处理
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

}  // namespace mapping
}  // namespace cartographer
