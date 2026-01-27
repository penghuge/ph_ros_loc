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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H
#define CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H

#include <chrono>
#include <deque>
#include <functional>

namespace cartographer {
namespace mapping {

struct WorkItem {
  //C++11，相较老版enum封装性更好，也可以指定底层数据类型
  enum class Result {
    kDoNotRunOptimization,
    kRunOptimization,//只在新增节点约束数量达到阈值，或者结束轨迹时才会执行优化
  };

  std::chrono::steady_clock::time_point time;
  std::function<Result()> task;
};

using WorkQueue = std::deque<WorkItem>;

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H
