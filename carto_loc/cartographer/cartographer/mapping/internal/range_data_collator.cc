/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(gaschler): These two cases can probably be one.
  if (id_to_pending_data_.count(sensor_id) != 0) {
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    //当我们有两个相同传感器的消息时，向前移动到两个中较早的一个(不发送当前这个)。
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
    return result;
  }
  id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
  //直到所有激光传感器ID都有数据时，才开始处理
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    sensor::TimedPointCloudData& data = it->second;
    sensor::TimedPointCloud& ranges = it->second.ranges;

    //找到[current_start_, current_end_]这段时间内的激光数据
    auto overlap_begin = ranges.begin();
    //(*overlap_begin).time为负值，data.time为激光数据测量结束时间
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <=
               current_end_) {
      ++overlap_end;
    }
    // while (overlap_begin < ranges.end() &&
    //     data.time + common::FromSeconds((*overlap_begin).time) <
    //     common::Time::min()) {
    //   ++overlap_begin;
    // }
    // auto overlap_end = overlap_begin;
    // while (overlap_end < ranges.end() &&
    //     data.time + common::FromSeconds((*overlap_end).time) <=
    //     common::Time::max()) {
    //   ++overlap_end;
    // }
    //如果有激光数据早于current_start_，打印一下将丢弃多少个过早的激光点
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);
      //current_end_对应激光点的时间戳与本周期激光数据测量结束时间戳之间的差值
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it,
                                                                  origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        //将[current_start_, current_end_]这段时间内的激光点的时间化归到[-xxx, 0]
        point.point_time.time += time_correction;
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    //只留下overlap_end之后的数据，之前的数据已经处理，删除
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } else if (overlap_end == ranges.begin()) {
      ++it;
    } else {
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end())};
      ++it;
    }
  }

  //将所有激光点按时间戳，从小到大排序
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
