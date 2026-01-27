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

#include "cartographer/mapping/internal/2d/pose_graph_2d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

//任务队列中任务执行滞后的度量
static auto* kWorkQueueDelayMetric = metrics::Gauge::Null();
static auto* kWorkQueueSizeMetric = metrics::Gauge::Null();
static auto* kConstraintsSameTrajectoryMetric = metrics::Gauge::Null();
static auto* kConstraintsDifferentTrajectoryMetric = metrics::Gauge::Null();
static auto* kActiveSubmapsMetric = metrics::Gauge::Null();
static auto* kFrozenSubmapsMetric = metrics::Gauge::Null();
static auto* kDeletedSubmapsMetric = metrics::Gauge::Null();

PoseGraph2D::PoseGraph2D(
    SlamCommon::CFileInterface *file_interface,
    SlamCommon::CConfigFileOperator *config_operator,
    const proto::PoseGraphOptions& options,
    std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
    common::ThreadPool* thread_pool)
    : options_(options),
      m_file_interface(file_interface),
      m_config_operator(config_operator),
      optimization_problem_(std::move(optimization_problem)),
      constraint_builder_(options_.constraint_builder_options(), thread_pool),
      thread_pool_(thread_pool) {
  //该参数目前被注释了
  if (options.has_overlapping_submaps_trimmer_2d()) {
    const auto& trimmer_options = options.overlapping_submaps_trimmer_2d();
    AddTrimmer(absl::make_unique<OverlappingSubmapsTrimmer2D>(
        trimmer_options.fresh_submaps_count(),
        trimmer_options.min_covered_area(),
        trimmer_options.min_added_submaps_count()));
  }
  //penghu*
 // updatemapdata_thread_ = new std::thread(&PoseGraph2D::UpdateMapData, this);
  m_match_model_ = m_file_interface->GetPureLocalization();
}

PoseGraph2D::~PoseGraph2D() {
  WaitForAllComputations();
  absl::MutexLock locker(&work_queue_mutex_);
  CHECK(work_queue_ == nullptr);
  //penghu*
//  is_update_ = true;
//  if((updatemapdata_thread_) != nullptr){
//      (updatemapdata_thread_)->join();
//      delete (updatemapdata_thread_);
//      (updatemapdata_thread_) = nullptr;
//  }
}

//penghu*
void PoseGraph2D::UpdateMapData(){
    TrimmingHandle trimming_handle(this);
    trimmer = absl::make_unique<OverlappingSubmapsTrimmer2D>(2,2,5);
    is_update_ = false;
    while(!is_update_){
      std::this_thread::sleep_for(std::chrono::seconds(60));
      trimmer->Trim(&trimming_handle);
    }
}

std::vector<SubmapId> PoseGraph2D::InitializeGlobalSubmapPoses(
    const int trajectory_id, const common::Time time,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_->submap_data();
  //只存在一张submap的情况，用于
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      if (data_.initial_trajectory_poses.count(trajectory_id) > 0) {
        //有初始位姿时，更新当前轨迹和相对轨迹的连接关系
        data_.trajectory_connectivity_state.Connect(
            trajectory_id,
            data_.initial_trajectory_poses.at(trajectory_id).to_trajectory_id,
            time);
      }
      //向OptimizationProblem2D添加第1张submap
      optimization_problem_->AddSubmap(
          trajectory_id, transform::Project2D(
                             ComputeLocalToGlobalTransform(
                                 data_.global_submap_poses_2d, trajectory_id) *
                             insertion_submaps[0]->local_pose()));
    }
    CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
    const SubmapId submap_id{trajectory_id, 0};
    CHECK(data_.submap_data.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }

  //此时必有2张submap
  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
  const SubmapId last_submap_id = std::prev(end_it)->id;
  //新增了有submap的情况
  if (data_.submap_data.at(last_submap_id).submap ==
      insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of
    // 'insertions_submaps.front()' and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
    optimization_problem_->AddSubmap(
        trajectory_id,
        //此时submap_data中最后这张submap为insertion_submaps[0]
        //first_submap_pose * constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse()
        //该表达式可求出局部向全局的转换关系
        first_submap_pose *
            constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
            constraints::ComputeSubmapPose(*insertion_submaps[1]));
    //返回最后两张submap
    return {last_submap_id,
            SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }

  //submap数量和状态均跟上一周期一样
  CHECK(data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.back());
  const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
  CHECK(data_.submap_data.at(front_submap_id).submap ==
        insertion_submaps.front());
  //返回最后两张submap
  return {front_submap_id, last_submap_id};
}

NodeId PoseGraph2D::AppendNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
    const transform::Rigid3d& optimized_pose) {
  absl::MutexLock locker(&mutex_);
  //将新增的轨迹ID插入连接关系中，最初只与自身连接
  AddTrajectoryIfNeeded(trajectory_id);
  //确认轨迹状态是否可修改
  if (!CanAddWorkItemModifying(trajectory_id)) {
    LOG(WARNING) << "AddNode was called for finished or deleted trajectory.";
  }
  //生成一个节点
  const NodeId node_id = data_.trajectory_nodes.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  ++data_.num_trajectory_nodes;
  // Test if the 'insertion_submap.back()' is one we never saw before.
  //如果节点中第2张submap是新生成的，也插入到data_中
  if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
              ->data.submap != insertion_submaps.back()) {
    // We grow 'data_.submap_data' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    const SubmapId submap_id =
        data_.submap_data.Append(trajectory_id, InternalSubmapData());
    data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    LOG(INFO) << "Inserted submap " << submap_id << ".";
    kActiveSubmapsMetric->Increment();
  }
  return node_id;
}

/**********************************************************************************
function: 使用'constant_data'添加新节点。
        它的'constant_data->local_pose'是通过扫描匹配'insertion_submaps.front()'来确定的，
        并且节点数据被插入到'insertion_submaps'中。
        如果'insertion_submap .front().finished()'为'true'，则数据最后一次插入到该submap中。
**********************************************************************************/
NodeId PoseGraph2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  // 将局部位姿转换到全局地图坐标系下
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
  //将所有节点信息保存到data_中
  const NodeId node_id = AppendNode(constant_data, trajectory_id,
                                    insertion_submaps, optimized_pose);
  // LOG_INFO("711_loc_traj_id = %d, 711_loc_traj_id_node_index = %d",
  //          trajectory_id, node_id.node_index);
  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  //必须检查第1张submap是否已经结束（一旦结束后是不能再更新里面的信息的），因为它可能在执行lambda时已经改变了。
  const bool newly_finished_submap =
      insertion_submaps.front()->insertion_finished();
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    return ComputeConstraintsForNode(node_id, insertion_submaps,
                                     newly_finished_submap);
  });
  return node_id;
}

//向任务队列中插入一条任务，并标记当前时间戳
void PoseGraph2D::AddWorkItem(
    const std::function<WorkItem::Result()>& work_item) {
  absl::MutexLock locker(&work_queue_mutex_);
  //构造任务队列WorkQueue
  if (work_queue_ == nullptr) {
    work_queue_ = absl::make_unique<WorkQueue>();
    //生成一条全部执行完任务队列的任务，放入线程池执行
    auto task = absl::make_unique<common::Task>();
    task->SetWorkItem([this]() { DrainWorkQueue(); });
    thread_pool_->Schedule(std::move(task));
  }
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
  kWorkQueueSizeMetric->Set(work_queue_->size());
  kWorkQueueDelayMetric->Set(
      std::chrono::duration_cast<std::chrono::duration<double>>(
          now - work_queue_->front().time)
          .count());
}

void PoseGraph2D::AddTrajectoryIfNeeded(const int trajectory_id) {
  data_.trajectories_state[trajectory_id];
  CHECK(data_.trajectories_state.at(trajectory_id).state !=
        TrajectoryState::FINISHED);
  CHECK(data_.trajectories_state.at(trajectory_id).state !=
        TrajectoryState::DELETED);
  CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
        InternalTrajectoryState::DeletionState::NORMAL);
  //将该轨迹添加至连接池中
  data_.trajectory_connectivity_state.Add(trajectory_id);
  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        absl::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }
}

void PoseGraph2D::AddImuData(const int trajectory_id,
                             const sensor::ImuData& imu_data) {
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      optimization_problem_->AddImuData(trajectory_id, imu_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddOdometryData(const int trajectory_id,
                                  const sensor::OdometryData& odometry_data) {
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      optimization_problem_->AddFixedFramePoseData(trajectory_id,
                                                   fixed_frame_pose_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddLandmarkData(int trajectory_id,
                                  const sensor::LandmarkData& landmark_data) {
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    //检查对应轨迹是否正常，各项状态正常时才可以更改
    if (CanAddWorkItemModifying(trajectory_id)) {
      std::string print_string = "observation.id: ";
      std::map<std::string, PoseGraphInterface::LandmarkNode> tmp_landmark_nodes;
      for (const auto& observation : landmark_data.landmark_observations) {
        print_string += observation.id + " ";
        //将地标数据插入队列，等待优化
        // data_.landmark_nodes[observation.id].landmark_observations.emplace_back(
        //     PoseGraphInterface::LandmarkNode::LandmarkObservation{
        //         trajectory_id, landmark_data.time,
        //         observation.landmark_to_tracking_transform,
        //         observation.translation_weight, observation.rotation_weight});
        all_landmark_nodes_[observation.id].landmark_observations.emplace_back(
            PoseGraphInterface::LandmarkNode::LandmarkObservation{
                trajectory_id, landmark_data.time,
                observation.landmark_to_tracking_transform,
                observation.translation_weight, observation.rotation_weight});
        if(all_landmark_nodes_.count(observation.id) > 0){
          tmp_landmark_nodes[observation.id] = all_landmark_nodes_[observation.id];
        }else {
          tmp_landmark_nodes[observation.id].landmark_observations.emplace_back(
            PoseGraphInterface::LandmarkNode::LandmarkObservation{
                trajectory_id, landmark_data.time,
                observation.landmark_to_tracking_transform,
                observation.translation_weight, observation.rotation_weight});
        }
      }
      std::swap(data_.landmark_nodes, tmp_landmark_nodes);
      LOG(INFO) << print_string;
    }
    //新插入的待处理数据，标记为未优化
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::ComputeConstraint(const NodeId& node_id,
                                    const SubmapId& submap_id) {
  bool maybe_add_local_constraint = false;
  bool maybe_add_global_constraint = false;
  const TrajectoryNode::Data* constant_data;
  const Submap2D* submap;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
    if (!data_.submap_data.at(submap_id).submap->insertion_finished()) {
      // Uplink server only receives grids when they are finished, so skip
      // constraint search before that.
      return;
    }

    const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
    const common::Time last_connection_time =
        data_.trajectory_connectivity_state.LastConnectionTime(
            node_id.trajectory_id, submap_id.trajectory_id);
    if(m_match_model_ == 4) m_match_model_ = 2;
    LOG(INFO) << ", m_match_model_++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++== = " << m_match_model_;
    switch (m_match_model_) {
        case 2:
            /* code */
            {
                if (node_id.trajectory_id == submap_id.trajectory_id ||
                    node_time < last_connection_time +
                                    common::FromSeconds(options_.global_constraint_search_after_n_seconds())) {
                    // If the node and the submap belong to the same trajectory or if there
                    // has been a recent global constraint that ties that node's trajectory to
                    // the submap's trajectory, it suffices to do a match constrained to a
                    // local search window.
                    maybe_add_local_constraint = true;
                } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
                    maybe_add_global_constraint = true;
                }
                break;
            }
        case 3:
            /* code */
            maybe_add_local_constraint = true;
            break;

        default:
            break;
    }

    constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    submap = static_cast<const Submap2D*>(
        data_.submap_data.at(submap_id).submap.get());
  }

  if (maybe_add_local_constraint) {
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_id)
            .global_pose.inverse() *
        optimization_problem_->node_data().at(node_id).global_pose_2d;
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);
  } else if (maybe_add_global_constraint) {
    constraint_builder_.MaybeAddGlobalConstraint(submap_id, submap, node_id,
                                                 constant_data);
  }
}

WorkItem::Result PoseGraph2D::ComputeConstraintsForNode(
    const NodeId& node_id,
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
    const bool newly_finished_submap) {
  std::vector<SubmapId> submap_ids;
  std::vector<SubmapId> finished_submap_ids;
  std::set<NodeId> newly_finished_submap_node_ids;
  {
    absl::MutexLock locker(&mutex_);
    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    //将新增的submap的全局位姿添加到OptimizationProblem2D中
    submap_ids = InitializeGlobalSubmapPoses(
        node_id.trajectory_id, constant_data->time, insertion_submaps);
    CHECK_EQ(submap_ids.size(), insertion_submaps.size());
    //计算第1张submap在2D的局部和全局位姿
    const SubmapId matching_id = submap_ids.front();
    const transform::Rigid2d local_pose_2d =
        transform::Project2D(constant_data->local_pose *
                             transform::Rigid3d::Rotation(
                                 constant_data->gravity_alignment.inverse()));
    const transform::Rigid2d global_pose_2d =
        optimization_problem_->submap_data().at(matching_id).global_pose *
        constraints::ComputeSubmapPose(*insertion_submaps.front()).inverse() *
        local_pose_2d;
    //将节点的位姿信息添加到OptimizationProblem2D
    optimization_problem_->AddTrajectoryNode(
        matching_id.trajectory_id,
        optimization::NodeSpec2D{constant_data->time, local_pose_2d,
                                 global_pose_2d,
                                 constant_data->gravity_alignment});
    // LOG_INFO("To be optimized global_pose_2d(x=%f, y=%f, angle= %f), traj_id = %d",global_pose_2d.translation().x(),
    //          global_pose_2d.translation().y(),
    //          global_pose_2d.rotation().angle(),
    //          matching_id.trajectory_id);
    //分别给两张submap加入约束（该约束为submap内部，节点和submap的位姿关系）
    for (size_t i = 0; i < insertion_submaps.size(); ++i) {
      const SubmapId submap_id = submap_ids[i];
      // Even if this was the last node added to 'submap_id', the submap will
      // only be marked as finished in 'data_.submap_data' further below.
      CHECK(data_.submap_data.at(submap_id).state ==
            SubmapState::kNoConstraintSearch);
      data_.submap_data.at(submap_id).node_ids.emplace(node_id);
      const transform::Rigid2d constraint_transform =
          constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
          local_pose_2d;
      data_.constraints.push_back(
          Constraint{submap_id,
                     node_id,
                     {transform::Embed3D(constraint_transform),
                      options_.matcher_translation_weight(),
                      options_.matcher_rotation_weight()},
                     Constraint::INTRA_SUBMAP});
    }

    // TODO(gaschler): Consider not searching for constraints against
    // trajectories scheduled for deletion.
    // TODO(danielsievers): Add a member variable and avoid having to copy
    // them out here.
    for (const auto& submap_id_data : data_.submap_data) {
      if (submap_id_data.data.state == SubmapState::kFinished) {
        CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
        // LOG_INFO("data_.submap_data.size = %d,kFinished submap_traj_id = %d, kFinished submap_index = %d", data_.submap_data.size(),
        //          submap_id_data.id.trajectory_id, submap_id_data.id.submap_index);
        finished_submap_ids.emplace_back(submap_id_data.id);
      }
    }
    if (newly_finished_submap) {
      const SubmapId newly_finished_submap_id = submap_ids.front();
      InternalSubmapData& finished_submap_data =
          data_.submap_data.at(newly_finished_submap_id);
      CHECK(finished_submap_data.state == SubmapState::kNoConstraintSearch);
      finished_submap_data.state = SubmapState::kFinished;
      newly_finished_submap_node_ids = finished_submap_data.node_ids;
    }
  }

  //给所有已完成的submap计算约束
  for (const auto& submap_id : finished_submap_ids) {
    ComputeConstraint(node_id, submap_id);
  }

  //给刚结束的submap计算约束
  if (newly_finished_submap) {
    const SubmapId newly_finished_submap_id = submap_ids.front();
    // We have a new completed submap, so we look into adding constraints for
    // old nodes.
    for (const auto& node_id_data : optimization_problem_->node_data()) {
      // LOG_INFO("node_id_data.size = %d, node_id_data.node_traj_id = %d, "
      //          "node_id_data.node_index = %d", optimization_problem_->node_data().size(),
      //          node_id_data.id.trajectory_id, node_id_data.id.node_index);
      const NodeId& node_id = node_id_data.id;
      if (newly_finished_submap_node_ids.count(node_id) == 0) {
        ComputeConstraint(node_id, newly_finished_submap_id);
      }
    }
  }
  //节点约束添加完毕，线程池开始计算
  constraint_builder_.NotifyEndOfNode();
  absl::MutexLock locker(&mutex_);
  ++num_nodes_since_last_loop_closure_;
  //当轨迹节点数设置为0时，会禁用全局优化，而只启用局部SLAM
  if (options_.optimize_every_n_nodes() > 0 &&
      //新插入的轨迹节点数大于指定数量时，启动优化
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) {
    return WorkItem::Result::kRunOptimization;
  }
  return WorkItem::Result::kDoNotRunOptimization;
}

common::Time PoseGraph2D::GetLatestNodeTime(const NodeId& node_id,
                                            const SubmapId& submap_id) const {
  common::Time time = data_.trajectory_nodes.at(node_id).constant_data->time;
  const InternalSubmapData& submap_data = data_.submap_data.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    const NodeId last_submap_node_id =
        *data_.submap_data.at(submap_id).node_ids.rbegin();
    time = std::max(
        time,
        data_.trajectory_nodes.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

void PoseGraph2D::UpdateTrajectoryConnectivity(const Constraint& constraint) {
  CHECK_EQ(constraint.tag, Constraint::INTER_SUBMAP);
  const common::Time time =
      GetLatestNodeTime(constraint.node_id, constraint.submap_id);
  data_.trajectory_connectivity_state.Connect(
      constraint.node_id.trajectory_id, constraint.submap_id.trajectory_id,
      time);
}

void PoseGraph2D::DeleteTrajectoriesIfNeeded() {
  TrimmingHandle trimming_handle(this);
  for (auto& it : data_.trajectories_state) {
    if (it.second.deletion_state ==
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
      // TODO(gaschler): Consider directly deleting from data_, which may be
      // more complete.
      auto submap_ids = trimming_handle.GetSubmapIds(it.first);
      for (auto& submap_id : submap_ids) {
        trimming_handle.TrimSubmap(submap_id);
      }
      it.second.state = TrajectoryState::DELETED;
      it.second.deletion_state = InternalTrajectoryState::DeletionState::NORMAL;
    }
  }
}

//对约束队列中的所有约束执行优化操作
void PoseGraph2D::HandleWorkQueue(
    const constraints::ConstraintBuilder2D::Result& result) {
  {
    //拼接新增的约束
    absl::MutexLock locker(&mutex_);
    data_.constraints.insert(data_.constraints.end(), result.begin(),
                             result.end());
  }
  RunOptimization();

  if (global_slam_optimization_callback_) {
    std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
    std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
    {
      absl::MutexLock locker(&mutex_);
      const auto& submap_data = optimization_problem_->submap_data();
      const auto& node_data = optimization_problem_->node_data();
      for (const int trajectory_id : node_data.trajectory_ids()) {
        if (node_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
            submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
          continue;
        }
        trajectory_id_to_last_optimized_node_id.emplace(
            trajectory_id,
            std::prev(node_data.EndOfTrajectory(trajectory_id))->id);
        trajectory_id_to_last_optimized_submap_id.emplace(
            trajectory_id,
            std::prev(submap_data.EndOfTrajectory(trajectory_id))->id);
      }
    }
    global_slam_optimization_callback_(
        trajectory_id_to_last_optimized_submap_id,
        trajectory_id_to_last_optimized_node_id);
  }

  {
    absl::MutexLock locker(&mutex_);
    for (const Constraint& constraint : result) {
      UpdateTrajectoryConnectivity(constraint);
    }
    DeleteTrajectoriesIfNeeded();
    //penghu*
    TrimmingHandle trimming_handle(this);
    for (auto& trimmer : trimmers_) {
      trimmer->Trim(&trimming_handle);
    }
    //将已完成优化的从裁剪队列中删除
    trimmers_.erase(
        std::remove_if(trimmers_.begin(), trimmers_.end(),
                       [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
                         return trimmer->IsFinished();
                       }),
        trimmers_.end());

    num_nodes_since_last_loop_closure_ = 0;

    // Update the gauges that count the current number of constraints.
    double inter_constraints_same_trajectory = 0;
    double inter_constraints_different_trajectory = 0;
    for (const auto& constraint : data_.constraints) {
      if (constraint.tag ==
          cartographer::mapping::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        ++inter_constraints_same_trajectory;
      } else {
        ++inter_constraints_different_trajectory;
      }
    }
    kConstraintsSameTrajectoryMetric->Set(inter_constraints_same_trajectory);
    kConstraintsDifferentTrajectoryMetric->Set(
        inter_constraints_different_trajectory);
  }

  DrainWorkQueue();
}

//将任务队列中的任务全部执行完，然后清空
void PoseGraph2D::DrainWorkQueue() {
  bool process_work_queue = true;
  size_t work_queue_size;
  while (process_work_queue) {
    std::function<WorkItem::Result()> work_item;
    {
      absl::MutexLock locker(&work_queue_mutex_);
      if (work_queue_->empty()) {
        work_queue_.reset();
        return;
      }
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
      kWorkQueueSizeMetric->Set(work_queue_size);
    }
    //执行任务
    process_work_queue = work_item() == WorkItem::Result::kDoNotRunOptimization;
  }

  //需要进行优化的任务，放入后台ConstraintBuilder2D进行优化
  //目前只有给节点计算约束、结束trajectory时才会进行该步优化
  LOG(INFO) << "Remaining work items in queue: " << work_queue_size;
  // We have to optimize again.
  constraint_builder_.WhenDone(
      [this](const constraints::ConstraintBuilder2D::Result& result) {
        HandleWorkQueue(result);
      });
}

void PoseGraph2D::WaitForAllComputations() {
  int num_trajectory_nodes;
  {
    absl::MutexLock locker(&mutex_);
    num_trajectory_nodes = data_.num_trajectory_nodes;
  }

  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();

  auto report_progress = [this, num_trajectory_nodes,
                          num_finished_nodes_at_start]() {
    // Log progress on nodes only when we are actually processing nodes.
    if (num_trajectory_nodes != num_finished_nodes_at_start) {
      std::ostringstream progress_info;
      progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                    << 100. *
                           (constraint_builder_.GetNumFinishedNodes() -
                            num_finished_nodes_at_start) /
                           (num_trajectory_nodes - num_finished_nodes_at_start)
                    << "%...";
      std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
  };

  // First wait for the work queue to drain so that it's safe to schedule
  // a WhenDone() callback.
  {
    const auto predicate = [this]()
                               EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_) {
                                 return work_queue_ == nullptr;
                               };
    absl::MutexLock locker(&work_queue_mutex_);
    while (!work_queue_mutex_.AwaitWithTimeout(
        absl::Condition(&predicate),
        absl::FromChrono(common::FromSeconds(1.)))) {
      report_progress();
    }
  }

  // Now wait for any pending constraint computations to finish.
  absl::MutexLock locker(&mutex_);
  bool notification = false;
  constraint_builder_.WhenDone(
      [this,
       &notification](const constraints::ConstraintBuilder2D::Result& result)
          LOCKS_EXCLUDED(mutex_) {
            absl::MutexLock locker(&mutex_);
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());
            notification = true;
          });
  const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return notification;
  };
  while (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                  absl::FromChrono(common::FromSeconds(1.)))) {
    report_progress();
  }
  CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
}

void PoseGraph2D::DeleteTrajectory(const int trajectory_id) {
  {
    absl::MutexLock locker(&mutex_);
    auto it = data_.trajectories_state.find(trajectory_id);
    if (it == data_.trajectories_state.end()) {
      LOG(WARNING) << "Skipping request to delete non-existing trajectory_id: "
                   << trajectory_id;
      return;
    }
    it->second.deletion_state =
        InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION;
  }
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::ACTIVE);
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::DELETED);
    CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
          InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION);
    data_.trajectories_state.at(trajectory_id).deletion_state =
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::FinishTrajectory(const int trajectory_id) {
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(!IsTrajectoryFinished(trajectory_id));
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FINISHED;

    for (const auto& submap : data_.submap_data.trajectory(trajectory_id)) {
      data_.submap_data.at(submap.id).state = SubmapState::kFinished;
    }
    return WorkItem::Result::kRunOptimization;
  });
}

bool PoseGraph2D::IsTrajectoryFinished(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FINISHED;
}

void PoseGraph2D::FreezeTrajectory(const int trajectory_id) {
  {
    absl::MutexLock locker(&mutex_);
    data_.trajectory_connectivity_state.Add(trajectory_id);
  }
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(!IsTrajectoryFrozen(trajectory_id));
    // Connect multiple frozen trajectories among each other.
    // This is required for localization against multiple frozen trajectories
    // because we lose inter-trajectory constraints when freezing.
    for (const auto& entry : data_.trajectories_state) {
      const int other_trajectory_id = entry.first;
      if (!IsTrajectoryFrozen(other_trajectory_id)) {
        continue;
      }
      if (data_.trajectory_connectivity_state.TransitivelyConnected(
              trajectory_id, other_trajectory_id)) {
        // Already connected, nothing to do.
        continue;
      }
      data_.trajectory_connectivity_state.Connect(
          trajectory_id, other_trajectory_id, common::FromUniversal(0));
    }
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FROZEN;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

bool PoseGraph2D::IsTrajectoryFrozen(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FROZEN;
}

void PoseGraph2D::AddSubmapFromProto(
    const transform::Rigid3d& global_submap_pose, const proto::Submap& submap) {
  if (!submap.has_submap_2d()) {
    return;
  }

  const SubmapId submap_id = {submap.submap_id().trajectory_id(),
                              submap.submap_id().submap_index()};

  const transform::Rigid2d global_submap_pose_2d =
      transform::Project2D(global_submap_pose);
  {
    absl::MutexLock locker(&mutex_);
    const std::shared_ptr<const Submap2D> submap_ptr =
        std::make_shared<const Submap2D>(submap.submap_2d(),
                                         &conversion_tables_);
    AddTrajectoryIfNeeded(submap_id.trajectory_id);
    if (!CanAddWorkItemModifying(submap_id.trajectory_id)) return;
    data_.submap_data.Insert(submap_id, InternalSubmapData());
    data_.submap_data.at(submap_id).submap = submap_ptr;
    // Immediately show the submap at the 'global_submap_pose'.
    data_.global_submap_poses_2d.Insert(
        submap_id, optimization::SubmapSpec2D{global_submap_pose_2d});
  }

  // TODO(MichaelGrupp): MapBuilder does freezing before deserializing submaps,
  // so this should be fine.
  if (IsTrajectoryFrozen(submap_id.trajectory_id)) {
    kFrozenSubmapsMetric->Increment();
  } else {
    kActiveSubmapsMetric->Increment();
  }

  AddWorkItem(
      [this, submap_id, global_submap_pose_2d]() LOCKS_EXCLUDED(mutex_) {
        absl::MutexLock locker(&mutex_);
        data_.submap_data.at(submap_id).state = SubmapState::kFinished;
        optimization_problem_->InsertSubmap(submap_id, global_submap_pose_2d);
        return WorkItem::Result::kDoNotRunOptimization;
      });
}

void PoseGraph2D::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                   const proto::Node& node) {
  const NodeId node_id = {node.node_id().trajectory_id(),
                          node.node_id().node_index()};
  std::shared_ptr<const TrajectoryNode::Data> constant_data =
      std::make_shared<const TrajectoryNode::Data>(FromProto(node.node_data()));

  {
    absl::MutexLock locker(&mutex_);
    AddTrajectoryIfNeeded(node_id.trajectory_id);
    if (!CanAddWorkItemModifying(node_id.trajectory_id)) return;
    data_.trajectory_nodes.Insert(node_id,
                                  TrajectoryNode{constant_data, global_pose});
  }

  AddWorkItem([this, node_id, global_pose]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
        constant_data->gravity_alignment.inverse());
    optimization_problem_->InsertTrajectoryNode(
        node_id,
        optimization::NodeSpec2D{
            constant_data->time,
            transform::Project2D(constant_data->local_pose *
                                 gravity_alignment_inverse),
            transform::Project2D(global_pose * gravity_alignment_inverse),
            constant_data->gravity_alignment});
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::SetTrajectoryDataFromProto(
    const proto::TrajectoryData& data) {
  TrajectoryData trajectory_data;
  // gravity_constant and imu_calibration are omitted as its not used in 2d

  if (data.has_fixed_frame_origin_in_map()) {
    trajectory_data.fixed_frame_origin_in_map =
        transform::ToRigid3(data.fixed_frame_origin_in_map());

    const int trajectory_id = data.trajectory_id();
    AddWorkItem([this, trajectory_id, trajectory_data]()
                    LOCKS_EXCLUDED(mutex_) {
                      absl::MutexLock locker(&mutex_);
                      if (CanAddWorkItemModifying(trajectory_id)) {
                        optimization_problem_->SetTrajectoryData(
                            trajectory_id, trajectory_data);
                      }
                      return WorkItem::Result::kDoNotRunOptimization;
                    });
  }
}

void PoseGraph2D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {
  AddWorkItem([this, node_id, submap_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(submap_id.trajectory_id)) {
      data_.submap_data.at(submap_id).node_ids.insert(node_id);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  AddWorkItem([this, constraints]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    for (const auto& constraint : constraints) {
      CHECK(data_.trajectory_nodes.Contains(constraint.node_id));
      CHECK(data_.submap_data.Contains(constraint.submap_id));
      CHECK(data_.trajectory_nodes.at(constraint.node_id).constant_data !=
            nullptr);
      CHECK(data_.submap_data.at(constraint.submap_id).submap != nullptr);
      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
          CHECK(data_.submap_data.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          UpdateTrajectoryConnectivity(constraint);
          break;
      }
      const Constraint::Pose pose = {
          constraint.pose.zbar_ij *
              transform::Rigid3d::Rotation(
                  data_.trajectory_nodes.at(constraint.node_id)
                      .constant_data->gravity_alignment.inverse()),
          constraint.pose.translation_weight, constraint.pose.rotation_weight};
      data_.constraints.push_back(Constraint{
          constraint.submap_id, constraint.node_id, pose, constraint.tag});
    }
    LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) {
  // C++11 does not allow us to move a unique_ptr into a lambda.
  PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
  AddWorkItem([this, trimmer_ptr]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    trimmers_.emplace_back(trimmer_ptr);
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph2D::RunFinalOptimization() {
  {
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          options_.max_num_final_iterations());
      return WorkItem::Result::kRunOptimization;
    });
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          options_.optimization_problem_options()
              .ceres_solver_options()
              .max_num_iterations());
      return WorkItem::Result::kDoNotRunOptimization;
    });
  }
  WaitForAllComputations();
}

void PoseGraph2D::RunOptimization() {
  if (optimization_problem_->submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_,
  // data_.constraints, data_.frozen_trajectories and data_.landmark_nodes
  // when executing the Solve. Solve is time consuming, so not taking the mutex
  // before Solve to avoid blocking foreground processing.
  //执行求解时，需要保证没有其他线程正在访问optimization_problem_，data_.constraints,
  //data_.frozen_trajectories，以及data_.landmark_nodes。
  //Solve非常耗时，所以在使用Solve之前不要使用互斥锁，以避免阻塞前台处理。
  optimization_problem_->Solve(data_.constraints, GetTrajectoryStates(),
                               data_.landmark_nodes);
  absl::MutexLock locker(&mutex_);

  const auto& submap_data = optimization_problem_->submap_data();
  const auto& node_data = optimization_problem_->node_data();
  for (const int trajectory_id : node_data.trajectory_ids()) {
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node.id);
      mutable_trajectory_node.global_pose =
          transform::Embed3D(node.data.global_pose_2d) *
          transform::Rigid3d::Rotation(
              mutable_trajectory_node.constant_data->gravity_alignment);
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
    //推断所有没有包含在“optimization_problem_”中的点云姿态。
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        data_.global_submap_poses_2d, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();

    const NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    auto node_it =
        std::next(data_.trajectory_nodes.find(last_optimized_node_id));
    //将所有节点从原先的全局坐标转换到优化后的全局坐标
    for (; node_it != data_.trajectory_nodes.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node_it->id);
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }
  }
  //将优化后的全局地标位姿保存下来
  for (const auto& landmark : optimization_problem_->landmark_data()) {
    // data_.landmark_nodes[landmark.first].global_landmark_pose = landmark.second;
    all_landmark_nodes_[landmark.first].global_landmark_pose = landmark.second;
  }
  data_.global_submap_poses_2d = submap_data;
}

//检查轨迹状态是否正常，即未停止，也没有被删除
bool PoseGraph2D::CanAddWorkItemModifying(int trajectory_id) {
  auto it = data_.trajectories_state.find(trajectory_id);
  if (it == data_.trajectories_state.end()) {
    return true;
  }
  if (it->second.state == TrajectoryState::FINISHED) {
    // TODO(gaschler): Replace all FATAL to WARNING after some testing.
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has finished "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.deletion_state !=
      InternalTrajectoryState::DeletionState::NORMAL) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been scheduled for deletion "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.state == TrajectoryState::DELETED) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been deleted "
                  "but modification is requested, skipping.";
    return false;
  }
  return true;
}

MapById<NodeId, TrajectoryNode> PoseGraph2D::GetTrajectoryNodes() const {
  absl::MutexLock locker(&mutex_);
  return data_.trajectory_nodes;
}

MapById<NodeId, TrajectoryNodePose> PoseGraph2D::GetTrajectoryNodePoses()
    const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& node_id_data : data_.trajectory_nodes) {
    absl::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

std::map<int, PoseGraphInterface::TrajectoryState>
PoseGraph2D::GetTrajectoryStates() const {
  std::map<int, PoseGraphInterface::TrajectoryState> trajectories_state;
  absl::MutexLock locker(&mutex_);
  for (const auto& it : data_.trajectories_state) {
    trajectories_state[it.first] = it.second.state;
  }
  return trajectories_state;
}

std::map<std::string, transform::Rigid3d> PoseGraph2D::GetLandmarkPoses()
    const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& landmark : data_.landmark_nodes) {
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] =
        landmark.second.global_landmark_pose.value();
  }
  return landmark_poses;
}

void PoseGraph2D::SetLandmarkPose(const std::string& landmark_id,
                                  const transform::Rigid3d& global_pose,
                                  const bool frozen) {
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    data_.landmark_nodes[landmark_id].global_landmark_pose = global_pose;
    data_.landmark_nodes[landmark_id].frozen = frozen;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

sensor::MapByTime<sensor::ImuData> PoseGraph2D::GetImuData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->imu_data();
}

sensor::MapByTime<sensor::OdometryData> PoseGraph2D::GetOdometryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->odometry_data();
}

std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
PoseGraph2D::GetLandmarkNodes() const {
  absl::MutexLock locker(&mutex_);
  return data_.landmark_nodes;
}

std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraph2D::GetTrajectoryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->trajectory_data();
}

sensor::MapByTime<sensor::FixedFramePoseData>
PoseGraph2D::GetFixedFramePoseData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->fixed_frame_pose_data();
}

std::vector<PoseGraphInterface::Constraint> PoseGraph2D::constraints() const {
  std::vector<PoseGraphInterface::Constraint> result;
  absl::MutexLock locker(&mutex_);
  for (const Constraint& constraint : data_.constraints) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 data_.trajectory_nodes.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

//设置初始轨迹位姿
void PoseGraph2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                           const int to_trajectory_id,
                                           const transform::Rigid3d& pose,
                                           const common::Time time) {
  absl::MutexLock locker(&mutex_);
  data_.initial_trajectory_poses[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
  LOG(INFO) << "Set init pose, traj id(" << from_trajectory_id << " >> " << to_trajectory_id << "), "
            << "init pose(" << pose.translation().x() << ", " << pose.translation().y()
            << ", " << transform::RadToDeg(transform::GetYaw(pose.rotation()))<< ")";
}

transform::Rigid3d PoseGraph2D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(data_.trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 0);
  //取出时间戳大于等于time的节点
  const auto it = data_.trajectory_nodes.lower_bound(trajectory_id, time);
  //开始节点或者最后一个节点，无需插值
  if (it == data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)) {
    return data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)
        ->data.global_pose;
  }
  if (it == data_.trajectory_nodes.EndOfTrajectory(trajectory_id)) {
    return std::prev(data_.trajectory_nodes.EndOfTrajectory(trajectory_id))
        ->data.global_pose;
  }
  //由上一节点和当前节点位姿，通过时间戳插值得到全局位姿
  return transform::Interpolate(
             transform::TimestampedTransform{std::prev(it)->data.time(),
                                             std::prev(it)->data.global_pose},
             transform::TimestampedTransform{it->data.time(),
                                             it->data.global_pose},
             time)
      .transform;
}

transform::Rigid3d PoseGraph2D::GetLocalToGlobalTransform(
    const int trajectory_id) const {
  absl::MutexLock locker(&mutex_);
  return ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                       trajectory_id);
}

std::vector<std::vector<int>> PoseGraph2D::GetConnectedTrajectories() const {
  absl::MutexLock locker(&mutex_);
  return data_.trajectory_connectivity_state.Components();
}

PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapData(
    const SubmapId& submap_id) const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetAllSubmapData() const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock();
}

MapById<SubmapId, PoseGraphInterface::SubmapPose>
PoseGraph2D::GetAllSubmapPoses() const {
  absl::MutexLock locker(&mutex_);
  MapById<SubmapId, SubmapPose> submap_poses;
  for (const auto& submap_id_data : data_.submap_data) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        PoseGraph::SubmapPose{submap_data.submap->num_range_data(),
                              submap_data.pose});
  }
  return submap_poses;
}

/**********************************************************************************
function: 获取最后一张submap局部向全局的变换关系
**********************************************************************************/
transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  //还没有全局submap位姿时，根据初始位姿初始化一个
  //如果没有给定初始位姿，则认为当前点为地图原点，此时局部submap和全局坐标系原点重合
  if (begin_it == end_it) {
    const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    if (it != data_.initial_trajectory_poses.end()) {
      //用当前时间戳从相对轨迹中插值得到一个全局位姿，然后乘以当前位姿相对该全局位姿的相对量，得到转换关系
      return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                                 it->second.time) *
             it->second.relative_pose;
    } else {
      return transform::Rigid3d::Identity();
    }
  }
  //如果submap已经有全局位姿，则用最后一张submap来计算转换关系
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;

  std::string result_string = "";
  double ref_cols_x = 0.0, ref_cols_y = 0.0;
  m_file_interface->GetRefColsPosition(ref_cols_x, ref_cols_y);
  if(ref_cols_x != 0.0 || ref_cols_y != 0.0) {
    Eigen::Vector2f pose_maybe_in_submap_local(ref_cols_x, ref_cols_y);
    for(auto iter = begin_it; iter != end_it; ++iter) {
      transform::Rigid3d global_pose_tmp = transform::Embed3D(global_submap_poses.at(iter->id).global_pose);
      transform::Rigid3d local_pose_tmp = data_.submap_data.at(iter->id).submap->local_pose();
      transform::Rigid3d local_to_global = global_pose_tmp * local_pose_tmp.inverse();
  
      const auto it = data_.submap_data.find(iter->id);
      if (it != data_.submap_data.end()) {
        const cartographer::mapping::Submap2D* tmp_submap2d = static_cast<const cartographer::mapping::Submap2D*>(it->data.submap.get());
        const cartographer::mapping::Grid2D* tmp_grid = tmp_submap2d->grid();
        const cartographer::mapping::MapLimits tmp_limit = tmp_grid->limits();
        // LOG(INFO) << "iter->id.submap_index: " << iter->id.submap_index << ", local_to_global: "
        //           << local_to_global.translation().x() << ", "
        //           << local_to_global.translation().y() << ", "
        //           << transform::GetYaw(local_to_global.rotation());
                  
        transform::Rigid2d pose_maybe_in_submap({{ref_cols_x, ref_cols_y}, 0.0});
        pose_maybe_in_submap = transform::Project2D(local_to_global).inverse() * pose_maybe_in_submap;
        pose_maybe_in_submap_local = Eigen::Vector2f(pose_maybe_in_submap.translation().x(), pose_maybe_in_submap.translation().y());
        
        bool in_submap = tmp_limit.Contains(tmp_limit.GetCellIndex(pose_maybe_in_submap_local));
        if(in_submap){
          result_string += absl::StrCat(iter->id.submap_index, " ");
        }
      }
    }
    m_file_interface->SetRefColsResponseResults(result_string);
    LOG(INFO) << "point (" << pose_maybe_in_submap_local[0] << ", " << pose_maybe_in_submap_local[1] << ") is in submap: " << result_string;
  }
  
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return transform::Embed3D(
    global_submap_poses.at(last_optimized_submap_id).global_pose) *
  data_.submap_data.at(last_optimized_submap_id)
      .submap->local_pose()
      .inverse();
}

PoseGraphInterface::SubmapData PoseGraph2D::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = data_.submap_data.find(submap_id);
  if (it == data_.submap_data.end()) {
    return {};
  }
  auto submap = it->data.submap;
  if (data_.global_submap_poses_2d.Contains(submap_id)) {
    // We already have an optimized pose.
    return {submap,
            transform::Embed3D(
                data_.global_submap_poses_2d.at(submap_id).global_pose)};
  }
  // We have to extrapolate.
  return {submap, ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

// std::string PoseGraph2D::JudgePointInSubmaps() {
//   ;
// }

PoseGraph2D::TrimmingHandle::TrimmingHandle(PoseGraph2D* const parent)
    : parent_(parent) {}

int PoseGraph2D::TrimmingHandle::num_submaps(const int trajectory_id) const {
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::TrimmingHandle::GetOptimizedSubmapData() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : parent_->data_.submap_data) {
    if (submap_id_data.data.state != SubmapState::kFinished ||
        !parent_->data_.global_submap_poses_2d.Contains(submap_id_data.id)) {
      continue;
    }
    // LOG_INFO("submap(data_.submap_data.size = %d, over_jiance _submap_traj = %d, over_jiance_submap_index = %d)", parent_->data_.submap_data.size(),
    //          submap_id_data.id.trajectory_id, submap_id_data.id.submap_index);
    submaps.Insert(
        submap_id_data.id,
        SubmapData{submap_id_data.data.submap,
                   transform::Embed3D(parent_->data_.global_submap_poses_2d
                                          .at(submap_id_data.id)
                                          .global_pose)});
  }
  return submaps;
}

std::vector<SubmapId> PoseGraph2D::TrimmingHandle::GetSubmapIds(
    int trajectory_id) const {
  std::vector<SubmapId> submap_ids;
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  for (const auto& it : submap_data.trajectory(trajectory_id)) {
    submap_ids.push_back(it.id);
  }
  return submap_ids;
}

const MapById<NodeId, TrajectoryNode>&
PoseGraph2D::TrimmingHandle::GetTrajectoryNodes() const {
  return parent_->data_.trajectory_nodes;
}

const std::vector<PoseGraphInterface::Constraint>&
PoseGraph2D::TrimmingHandle::GetConstraints() const {
  return parent_->data_.constraints;
}

bool PoseGraph2D::TrimmingHandle::IsFinished(const int trajectory_id) const {
  return parent_->IsTrajectoryFinished(trajectory_id);
}

void PoseGraph2D::TrimmingHandle::SetTrajectoryState(int trajectory_id,
                                                     TrajectoryState state) {
  parent_->data_.trajectories_state[trajectory_id].state = state;
}

void PoseGraph2D::TrimmingHandle::TrimSubmap(const SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
  // with 'submap_id' is gone.
  std::set<NodeId> nodes_to_retain;
  for (const Constraint& constraint : parent_->data_.constraints) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
        constraint.submap_id != submap_id) {
      nodes_to_retain.insert(constraint.node_id);
    }
  }
  // Remove all 'data_.constraints' related to 'submap_id'.
  std::set<NodeId> nodes_to_remove;
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (constraint.submap_id == submap_id) {
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
            nodes_to_retain.count(constraint.node_id) == 0) {
          // This node will no longer be INTRA_SUBMAP contrained and has to be
          // removed.
          nodes_to_remove.insert(constraint.node_id);
        }
      } else {
        constraints.push_back(constraint);
      }
    }
    parent_->data_.constraints = std::move(constraints);
  }
  // Remove all 'data_.constraints' related to 'nodes_to_remove'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      }
    }
    parent_->data_.constraints = std::move(constraints);
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);
  parent_->data_.submap_data.Trim(submap_id);
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  parent_->optimization_problem_->TrimSubmap(submap_id);

  // We have one submap less, update the gauge metrics.
  kDeletedSubmapsMetric->Increment();
  if (parent_->IsTrajectoryFrozen(submap_id.trajectory_id)) {
    kFrozenSubmapsMetric->Decrement();
  } else {
    kActiveSubmapsMetric->Decrement();
  }

  // Remove the 'nodes_to_remove' from the pose graph and the optimization
  // problem.
  for (const NodeId& node_id : nodes_to_remove) {
    parent_->data_.trajectory_nodes.Trim(node_id);
    parent_->optimization_problem_->TrimTrajectoryNode(node_id);
  }
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph2D::GetSubmapDataUnderLock() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : data_.submap_data) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

void PoseGraph2D::SetGlobalSlamOptimizationCallback(
    PoseGraphInterface::GlobalSlamOptimizationCallback callback) {
  global_slam_optimization_callback_ = callback;
}

void PoseGraph2D::RegisterMetrics(metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_work_queue_delay",
      "Age of the oldest entry in the work queue in seconds");
  kWorkQueueDelayMetric = latency->Add({});
  auto* queue_size =
      family_factory->NewGaugeFamily("mapping_2d_pose_graph_work_queue_size",
                                     "Number of items in the work queue");
  kWorkQueueSizeMetric = queue_size->Add({});
  auto* constraints = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_constraints",
      "Current number of constraints in the pose graph");
  kConstraintsDifferentTrajectoryMetric =
      constraints->Add({{"tag", "inter_submap"}, {"trajectory", "different"}});
  kConstraintsSameTrajectoryMetric =
      constraints->Add({{"tag", "inter_submap"}, {"trajectory", "same"}});
  auto* submaps = family_factory->NewGaugeFamily(
      "mapping_2d_pose_graph_submaps", "Number of submaps in the pose graph.");
  kActiveSubmapsMetric = submaps->Add({{"state", "active"}});
  kFrozenSubmapsMetric = submaps->Add({{"state", "frozen"}});
  kDeletedSubmapsMetric = submaps->Add({{"state", "deleted"}});
}

}  // namespace mapping
}  // namespace cartographer
