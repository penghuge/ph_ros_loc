/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/internal/trajectory_connectivity_state.h"

namespace cartographer {
namespace mapping {

void TrajectoryConnectivityState::Add(const int trajectory_id) {
  connected_components_.Add(trajectory_id);
}

void TrajectoryConnectivityState::Connect(const int trajectory_id_a,
                                          const int trajectory_id_b,
                                          const common::Time time) {
  if (TransitivelyConnected(trajectory_id_a, trajectory_id_b)) {
    // The trajectories are transitively connected, i.e. they belong to the same
    // connected component. In this case we only update the last connection time
    // of those two trajectories.
    //轨迹是传递连接的，即它们属于同一连通组件。此时，只更新这两个轨迹的最后连接时间。
    //对两条轨迹编号排序，sorted_pair.first为小编号，sorted_pair.second为大编号
    auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
    if (last_connection_time_map_[sorted_pair] < time) {
      last_connection_time_map_[sorted_pair] = time;
    }
  } else {
    // The connection between these two trajectories is about to join to
    // connected components. Here we update all bipartite trajectory pairs for
    // the two connected components with the connection time. This is to quickly
    // change to a more efficient loop closure search (by constraining the
    // search window) when connected components are joined.
    //这两条轨迹之间的连接将会连接到相连的组件上。并更新两条轨迹所连的连接组件的连接时间。
    //这是为了在连接组件时，快速转变为更高效的回环检测(通过约束搜索窗口)。
    std::vector<int> component_a =
        connected_components_.GetComponent(trajectory_id_a);
    std::vector<int> component_b =
        connected_components_.GetComponent(trajectory_id_b);
    for (const auto id_a : component_a) {
      for (const auto id_b : component_b) {
        auto id_pair = std::minmax(id_a, id_b);
        last_connection_time_map_[id_pair] = time;
      }
    }
  }
  connected_components_.Connect(trajectory_id_a, trajectory_id_b);
}

bool TrajectoryConnectivityState::TransitivelyConnected(
    const int trajectory_id_a, const int trajectory_id_b) const {
  return connected_components_.TransitivelyConnected(trajectory_id_a,
                                                     trajectory_id_b);
}

std::vector<std::vector<int>> TrajectoryConnectivityState::Components() const {
  return connected_components_.Components();
}

common::Time TrajectoryConnectivityState::LastConnectionTime(
    const int trajectory_id_a, const int trajectory_id_b) {
  const auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
  return last_connection_time_map_[sorted_pair];
}

}  // namespace mapping
}  // namespace cartographer
