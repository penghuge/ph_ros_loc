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

#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/common_histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_2d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::cartographer::mapping::optimization::CeresPose;
using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;
using TrajectoryData =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryData;

// For fixed frame pose.
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::FixedFramePoseData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id) ||
      !it->pose.has_value()) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose.value());
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  if (prev_it->pose.has_value()) {
    return absl::make_unique<transform::Rigid3d>(
        Interpolate(transform::TimestampedTransform{prev_it->time,
                                                    prev_it->pose.value()},
                    transform::TimestampedTransform{it->time, it->pose.value()},
                    time)
            .transform);
  }
  return nullptr;
}

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
//将一个姿态转换为Ceres使用的3个优化变量格式:x和y方向上的平移，以及表示方向的旋转角度。
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
//将Ceres表示的位姿转换为transform::Rigid2d位姿。
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
//选择与地标观测时间最接近的轨迹节点，并对其进行相对变换。
//在前后相邻两个节点之间，按时间戳比例插值一个地标初始位姿
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeSpec2D& prev_node, const NodeSpec2D& next_node,
    const std::array<double, 3>& prev_node_pose,
    const std::array<double, 3>& next_node_pose) {
  //计算地标在前后相邻两个节点之间的插值比例
  const double interpolation_parameter =
      common::ToSeconds(observation.time - prev_node.time) /
      common::ToSeconds(next_node.time - prev_node.time);

  //在前后相邻两个节点之间插值一个地标
  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation =
          InterpolateNodes2D(prev_node_pose.data(), prev_node.gravity_alignment,
                             next_node_pose.data(), next_node.gravity_alignment,
                             interpolation_parameter);
  //将插值得到的地标坐标转换到tracking坐标系下
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
}

void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    const MapById<NodeId, NodeSpec2D>& node_data,
    MapById<NodeId, std::array<double, 3>>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem,
    double huber_scale) {
  for (const auto& landmark_node : landmark_nodes) {
    std::string print_string = "cost function observation id: " + landmark_node.first + ": ";
    std::string sub_print_string = "";
    //为每一个观测到的地标，加入代价函数，计算残差
    for (const auto& observation : landmark_node.second.landmark_observations) {
    // if(!landmark_node.second.landmark_observations.empty()) {
    //   const auto& observation = landmark_node.second.landmark_observations.back();
      const std::string& landmark_id = landmark_node.first;
      const auto& begin_of_trajectory =
          node_data.BeginOfTrajectory(observation.trajectory_id);
      // The landmark observation was made before the trajectory was created.
      //略过是轨迹创建之前观测到的地标数据
      if (observation.time < begin_of_trajectory->data.time) {
        continue;
      }
      // Find the trajectory nodes before and after the landmark observation.
      //求出比地标观测时间戳晚的第一帧轨迹节点。
      auto next =
          node_data.lower_bound(observation.trajectory_id, observation.time);
      // The landmark observation was made, but the next trajectory node has
      // not been added yet.
      //地标观测已经完成，但是下一个轨迹节点还没有加入。
      if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
        continue;
      }
      //如果第一个轨迹节点的时间戳就比地标观测时间晚，将next指向第二个节点
      if (next == begin_of_trajectory) {
        next = std::next(next);
      }
      auto prev = std::prev(next);

      sub_print_string += std::to_string(std::distance(begin_of_trajectory, prev)) + "->" + std::to_string(std::distance(begin_of_trajectory, next)) + " ";
      // Add parameter blocks for the landmark ID if they were not added before.
      //如果之前没有添加地标ID，则添加它们的参数块。
      std::array<double, 3>* prev_node_pose = &C_nodes->at(prev->id);
      std::array<double, 3>* next_node_pose = &C_nodes->at(next->id);
      if (!C_landmarks->count(landmark_id)) {
        //如果该地标还没有全局坐标，为其生成一个初始的全局坐标
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                : GetInitialLandmarkPose(observation, prev->data, next->data,
                                         *prev_node_pose, *next_node_pose);

        //向problem中插入starting_point的平移和旋转参数块
        C_landmarks->emplace(
            landmark_id,
            CeresPose(starting_point, nullptr /* translation_parametrization */,
                      absl::make_unique<ceres::QuaternionParameterization>(),
                      problem));

        // Set landmark constant if it is frozen.
        //将冻结的地标设置成常量参数
        if (landmark_node.second.frozen) {
          LOG(INFO) << "landmark_node.second.frozen: landmark_id: " << landmark_id;
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).translation());
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).rotation());
        }
      }

      //添加代价函数
      //优化目标：插值得到的节点位姿与优化后的地标间的相对位姿，与tracking和地标间的相对位姿，两者之差最小
      problem->AddResidualBlock( 
          LandmarkCostFunction2D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          new ceres::HuberLoss(huber_scale), prev_node_pose->data(),
          next_node_pose->data(), C_landmarks->at(landmark_id).rotation(),
          C_landmarks->at(landmark_id).translation());
    }
    LOG(INFO) << print_string + sub_print_string;
  }
}

}  // namespace

OptimizationProblem2D::OptimizationProblem2D(
    const proto::OptimizationProblemOptions& options)
    : options_(options) {}

OptimizationProblem2D::~OptimizationProblem2D() {}

void OptimizationProblem2D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
  imu_data_.Append(trajectory_id, imu_data);
}

void OptimizationProblem2D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

void OptimizationProblem2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  fixed_frame_pose_data_.Append(trajectory_id, fixed_frame_pose_data);
}

void OptimizationProblem2D::AddTrajectoryNode(const int trajectory_id,
                                              const NodeSpec2D& node_data) {
  node_data_.Append(trajectory_id, node_data);
  trajectory_data_[trajectory_id];
}

void OptimizationProblem2D::SetTrajectoryData(
    int trajectory_id, const TrajectoryData& trajectory_data) {
  trajectory_data_[trajectory_id] = trajectory_data;
}

void OptimizationProblem2D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
  trajectory_data_[node_id.trajectory_id];
}

void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
  imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
  if (node_data_.SizeOfTrajectoryOrZero(node_id.trajectory_id) == 0) {
    trajectory_data_.erase(node_id.trajectory_id);
  }
}

void OptimizationProblem2D::AddSubmap(
    const int trajectory_id, const transform::Rigid2d& global_submap_pose) {
  // LOG_INFO("AddSubmap.submap_data_.size = %d", submap_data_.size());
  submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
}

void OptimizationProblem2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  // LOG_INFO("InsertSubmap.submap_data_.size = %d", submap_data_.size());
  submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}

void OptimizationProblem2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

void OptimizationProblem2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

//全局优化求解，同时考虑多个数据源的损失函数，
//包括：全局(闭环)约束、非全局(匹配)约束、IMU加速度和旋转量、局部SLAM粗略位姿估计、里程计数据或固定坐标系位姿(如GPS系统)
void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectories_state,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapSpec.
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;
  //将所有submap的全局坐标加入problem中
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    //将具有适当大小的参数块添加到问题中。具有相同参数的重复调用将被忽略。
    //使用相同的双指针但大小不同的重复调用将导致未定义的行为。
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      //修正第一张子地图的位置或者一个冻结轨迹的所有子地图的位置。
      //在优化过程中保持指定的参数块不变。
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  //将所有节点的全局坐标加入problem中
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    //向problem中添加节点坐标
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  // Add cost functions for intra- and inter-submap constraints.
  //为子地图内部和子地图之间的约束添加代价函数。
  for (const Constraint& constraint : constraints) {
    //使用少量参数添加残差的方便方法。这是常见的情况。与其将参数块参数指定为向量，不如将它们作为指针列出。
    //对于每个约束都添加一个代价函数，优化参数为该约束下的子地图和节点的坐标
    //优化目标：节点和子地图的相对位姿，与约束中记录的相对位姿之间的偏差最小（重投影误差最小）
    problem.AddResidualBlock(
        CreateAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        //如果是不同子地图间的约束，则需要闭环，而闭环约束要有损失函数。
        //Huber Loss 是一个用于回归问题的带参损失函数,
        //优点是能增强平方误差损失函数(MSE, mean square error)对离群点的鲁棒性。
        //当预测偏差小于 δ 时，它采用平方误差；当预测偏差大于 δ 时，采用的线性误差。
        //相比于最小二乘的线性回归，HuberLoss降低了对离群点的惩罚程度，所以 HuberLoss是一种常用的鲁棒的回归损失函数。
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id).data());
  }
  // Add cost functions for landmarks.
  //为所有观测到的地标添加残差
  AddLandmarkCostFunctions(landmark_nodes, node_data_, &C_nodes, &C_landmarks,
                           &problem, options_.huber_scale());
  // Add penalties for violating odometry or changes between consecutive nodes
  // if odometry is not available.
  //增加违反里程计数据的惩罚，或者如果无法使用里程计，则在连续节点之间进行更改。
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    //不处理已冻结轨迹
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    auto prev_node_it = node_it;
    for (++node_it; node_it != trajectory_end; ++node_it) {
      const NodeId first_node_id = prev_node_it->id;
      const NodeSpec2D& first_node_data = prev_node_it->data;
      prev_node_it = node_it;
      const NodeId second_node_id = node_it->id;
      const NodeSpec2D& second_node_data = node_it->data;

      if (second_node_id.node_index != first_node_id.node_index + 1) {
        continue;
      }

      // Add a relative pose constraint based on the odometry (if available).
      //添加一个基于里程计(如果可用)的相对位姿约束。
      std::unique_ptr<transform::Rigid3d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      if (relative_odometry != nullptr) {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry, options_.odometry_translation_weight(),
                options_.odometry_rotation_weight()}),
            nullptr /* loss function */, C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }

      // Add a relative pose constraint based on consecutive local SLAM poses.
      //添加一个基于连续的局部SLAM位姿的相对位姿约束。
      const transform::Rigid3d relative_local_slam_pose =
          transform::Embed3D(first_node_data.local_pose_2d.inverse() *
                             second_node_data.local_pose_2d);
      //优化目标：前后两个节点的相对位姿，与约束中记录的相对位姿之间的偏差最小
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(
              Constraint::Pose{relative_local_slam_pose,
                               options_.local_slam_pose_translation_weight(),
                               options_.local_slam_pose_rotation_weight()}),
          nullptr /* loss function */, C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
    }
  }

  std::map<int, std::array<double, 3>> C_fixed_frames;
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (!fixed_frame_pose_data_.HasTrajectory(trajectory_id)) {
      node_it = trajectory_end;
      continue;
    }

    const TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
    bool fixed_frame_pose_initialized = false;
    for (; node_it != trajectory_end; ++node_it) {
      const NodeId node_id = node_it->id;
      const NodeSpec2D& node_data = node_it->data;

      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      if (fixed_frame_pose == nullptr) {
        continue;
      }

      const Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      if (!fixed_frame_pose_initialized) {
        transform::Rigid2d fixed_frame_pose_in_map;
        if (trajectory_data.fixed_frame_origin_in_map.has_value()) {
          fixed_frame_pose_in_map = transform::Project2D(
              trajectory_data.fixed_frame_origin_in_map.value());
        } else {
          fixed_frame_pose_in_map =
              node_data.global_pose_2d *
              transform::Project2D(constraint_pose.zbar_ij).inverse();
        }

        C_fixed_frames.emplace(trajectory_id,
                               FromPose(fixed_frame_pose_in_map));
        fixed_frame_pose_initialized = true;
      }

      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(constraint_pose),
          options_.fixed_frame_pose_use_tolerant_loss()
              ? new ceres::TolerantLoss(
                    options_.fixed_frame_pose_tolerant_loss_param_a(),
                    options_.fixed_frame_pose_tolerant_loss_param_b())
              : nullptr,
          C_fixed_frames.at(trajectory_id).data(), C_nodes.at(node_id).data());
    }
  }

  // Solve.
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);
  //打印求解结果
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // Store the result.
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }
  for (const auto& C_fixed_frame : C_fixed_frames) {
    trajectory_data_.at(C_fixed_frame.first).fixed_frame_origin_in_map =
        transform::Embed3D(ToPose(C_fixed_frame.second));
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
}

//按时间戳插值一个里程计位姿
std::unique_ptr<transform::Rigid3d> OptimizationProblem2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  const auto it = odometry_data_.lower_bound(trajectory_id, time);
  if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose);
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  return absl::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

//计算两个节点之间，里程计估计的位姿
std::unique_ptr<transform::Rigid3d>
OptimizationProblem2D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec2D& first_node_data,
    const NodeSpec2D& second_node_data) const {
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      transform::Rigid3d relative_odometry =
          transform::Rigid3d::Rotation(first_node_data.gravity_alignment) *
          first_node_odometry->inverse() * (*second_node_odometry) *
          transform::Rigid3d::Rotation(
              second_node_data.gravity_alignment.inverse());
      return absl::make_unique<transform::Rigid3d>(relative_odometry);
    }
  }
  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
