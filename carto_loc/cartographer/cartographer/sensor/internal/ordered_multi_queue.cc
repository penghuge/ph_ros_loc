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

#include "cartographer/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}

void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
  auto& queue = it->second;
  CHECK(!queue.finished);
  queue.finished = true;
  Dispatch();
}

void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  //确保队列中存在指定的key
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  it->second.queue.Push(std::move(data));

  //将插入的数据分配处理
  Dispatch();
}

void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}

QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

void OrderedMultiQueue::Dispatch() {
  while (true) {
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    //取出所有传感器中时间戳最早的数据
    for (auto it = queues_.begin(); it != queues_.end();) {
      //查看队列是否为空
      const auto* data = it->second.queue.Peek<Data>();
      //数据队列为空，说明所有数据已经处理完成，且没有收到新数据
      if (data == nullptr) {
        //如果类型数据，队列中没有未处理数据，且已经被标记为处理完成，则将该类型数据的key从map中删除
        //一般到结束建图时才会标记为完成
        if (it->second.finished) {
          queues_.erase(it++);
          continue;
        }
        //因该key没有数据，其他数据暂时也不处理，进入等待
        CannotMakeProgress(it->first);
        return;
      }
      //找出所有key中，时间戳最小的key，准备处理对应数据
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      //确认下一帧数据比上一帧已处理数据的时间戳大
//      CHECK_LE(last_dispatched_time_, next_data->GetTime())
//          << "Non-sorted data added to queue: '" << it->first << "'";
      if(last_dispatched_time_ > next_data->GetTime())
      {
          LOG(WARNING) << "Non-sorted data added to queue: '" << it->first << "'";
      }
      ++it;
    }
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    //如果我们还没有为这个轨迹发送任何数据，那么就快进这个轨迹的所有队列，直到到达一个公共的开始时间。
    //对于某条轨迹，该操作只会执行一次
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);

    //正常情况，均是满足这个条件，调用传入的lambda表达式处理数据
    //该lambda表达式在CollatedTrajectoryBuilder构造时传入
    if (next_data->GetTime() >= common_start_time) {
      // Happy case, we are beyond the 'common_start_time' already.
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    }
    //待处理数据只剩最后一帧数据
    else if (next_queue->queue.Size() < 2) {
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      }
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.
      //检查待处理数据之后的那帧数据的时间。如果它也没超过‘common_start_time’，则删除‘next_data’，
      //否则我们只找到要从这个队列分派的第一个包。
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key;
  //当传入的数据key在数据队列中没有数据时，查看一下其他key的数据量是否有超过60个的，
  //如果有，打印一下，提示说在等待传入数据key对应的数据
  for (auto& entry : queues_) {
    if (entry.second.queue.Size() > kMaxQueueSize) {
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
      return;
    }
  }
}

common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {
  //std::map的emplace，first为key，second为成功与否的值
  //如果不存在，则插入成功，返回true，如果已经存在，则插入失败，返回false
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  //取出该轨迹最早开始时间
  common::Time& common_start_time = emplace_result.first->second;
  if (emplace_result.second) {
    for (auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id) {
        //查找所有传感器数据队列中第一个数据的时间戳的最大值，从这个时间开始，所有传感器都已经有数据
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer
