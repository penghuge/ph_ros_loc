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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/2d/map_limits.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
//定义栅格地图的界限。由于性能原因，这个类必须保持内联。
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  explicit MapLimits(const proto::MapLimits& map_limits)
      : resolution_(map_limits.resolution()),
        max_(transform::ToEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  //返回单元格大小(以米为单位)。所有的单元格都是正方形的，分辨率是一条边的长度。
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  //返回极限的一角，即所有像素的位置的坐标都比该值小
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  // 返回网格在单元数量上的限制。
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  //返回单元格的索引，其中包含可能在地图之外的“点”，即，为负或太大的索引，这时如果调用Contains()将返回false。
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const
  {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    //索引值以行为主维度，左上角有Eigen::Array2i::Zero()和contains (centered_max_x, centered_max_y)。
    //我们需要翻转和旋转。
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  // Returns the center of the cell at 'cell_index'.
  Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const
  {
    return {max_.x() - resolution() * (cell_index[1] + 0.5),
            max_.y() - resolution() * (cell_index[0] + 0.5)};
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  //cell_index必须在0和最大值之间
  bool Contains(const Eigen::Array2i& cell_index) const
  {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;
  Eigen::Vector2d max_;//地图右上角
  CellLimits cell_limits_;
};

inline proto::MapLimits ToProto(const MapLimits& map_limits) {
  proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = transform::ToProto(map_limits.max());
  *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
  return result;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
