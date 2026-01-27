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

#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"

#include "cartographer/mapping/probability_values.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
//计算在“pose”处，将“点云”与“网格”匹配的代价。网格和观测点的对应关系越差(例如点落在更小的占用空间内)，代价就越高。
class OccupiedSpaceCostFunction2D {
 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const Grid2D& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    //代价函数，pose被指定大小为3，residual被指定大小为动态
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    //构造3x3变换矩阵
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(grid_);
    //要求模板类型必须具备函数GetValue()
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      //注意：对于2D点，只有x和y，第三个参数为比例因子，此处，该比例设置为1，保证平移时不变形
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));
      //旋转变换，将激光坐标点转换到全局地图中
      const Eigen::Matrix<T, 3, 1> world = transform * point;

      //将激光点在全局地图中的坐标转换到栅格地图中，并添加一个阴影，以此来减小浮点向整形转换造成的误差
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      //将残差值乘以比例因子
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const Grid2D& grid) : grid_(grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      //如果越界，返回最大的损失率
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMaxCorrespondenceCost;
      } else {
        //去掉阴影，将行和列重新转换到栅格地图中，去读取表中记录的损失值
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    //返回行的最大数 + 2倍阴影，注意行对应y
    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    //返回列的最大数 + 2倍阴影，注意列对应x
    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    //定义为常量引用，必须在构造的时候完成赋值
    const Grid2D& grid_;
  };

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const Grid2D& grid_;
};

}  // namespace

//创建2D的占有栅格代码函数
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const Grid2D& grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables 输入的维度，此处包括x/y/theta三个*/>(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
      point_cloud.size());
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
