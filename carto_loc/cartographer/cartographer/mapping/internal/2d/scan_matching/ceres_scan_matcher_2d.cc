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

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

//利用ceres进行激光匹配，以实现激光点在栅格地图中占有概率最高，与目标位置偏差最小为优化目标
void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary,
                               const double &v) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  int a,b,c = 0;
//   LOG_INFO("519_v = %f", v);
  //penghu 24/5/19
  if(v == 3)
  {
    //   LOG_INFO("519_v_A,B,C");
      a = 10; b = 20; c = 20;
  }else
  {
    //   LOG_INFO("519_v_a,b,c");
      a = options_.occupied_space_weight();
      b = options_.translation_weight();
      c = options_.rotation_weight();
  }

  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      //使用少量参数添加残差的方便方法。这是常见的情况。与其将参数块参数指定为向量，不如将它们作为指针列出。
      //添加占有栅格代价函数残差，比例因子 = 设置比例值 / 激光点数
      CHECK_GT(options_.occupied_space_weight(), 0.);
      problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
              /*a*/options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate/*待寻优参数*/);
      break;
    case GridType::TSDF:
      problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D&>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
  }
  //添加平移残差
  CHECK_GT(options_.translation_weight(), 0.);
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          /*b*/options_.translation_weight(), target_translation),
      nullptr /* loss function */, ceres_pose_estimate);
  //添加旋转残差
  CHECK_GT(options_.rotation_weight(), 0.);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          /*c*/options_.rotation_weight(), ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
