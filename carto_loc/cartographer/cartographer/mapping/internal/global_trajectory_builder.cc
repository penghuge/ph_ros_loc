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

#include "cartographer/mapping/internal/global_trajectory_builder.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/metrics/family_factory.h"
#include "glog/logging.h"

// #include "cartographer/mapping/id.h" //lyy

namespace cartographer {
namespace mapping {
namespace {

static auto* kLocalSlamMatchingResults = metrics::Counter::Null();
static auto* kLocalSlamInsertionResults = metrics::Counter::Null();

//模板类，用于整合2D和3D
template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
public:
    // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
    // 'TimedPointCloudData' may be added in that case.
    //可以给'local_trajectory_builder'传递一个'nullptr'，但是这样就不能添加'TimedPointCloudData'
    GlobalTrajectoryBuilder(
            SlamCommon::CFileInterface *file_interface,
            SlamCommon::CConfigFileOperator *config_operator,
            std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
            const int trajectory_id, PoseGraph* const pose_graph,
            const LocalSlamResultCallback& local_slam_result_callback)
        : file_interface_(file_interface),
          config_operator_(config_operator),
          trajectory_id_(trajectory_id),
          pose_graph_(pose_graph),
          local_trajectory_builder_(std::move(local_trajectory_builder)),
          local_slam_result_callback_(local_slam_result_callback) {

        m_match_model_ = file_interface_->GetPureLocalization();
        LOG_INFO("m_match_model_ = %d", m_match_model_);

        submap_id_ = 0;//lyy
        assigned_flag_ = false;//lyy
    }
    ~GlobalTrajectoryBuilder() override {}

    GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
    GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

    void AddSensorData(
            const std::string& sensor_id,
            const sensor::TimedPointCloudData& timed_point_cloud_data) override {
        CHECK(local_trajectory_builder_)
                << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";
                
        local_trajectory_builder_->AddSubmapIdAndFlag(submap_id_, assigned_flag_);//lyy
        //向局部轨迹构造器中添加激光数据，主要是进行局部激光匹配
        std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
                matching_result = local_trajectory_builder_->AddRangeData(
                    sensor_id, timed_point_cloud_data);
        if (matching_result == nullptr) {
            // The range data has not been fully accumulated yet.
            return;
        }
        //penghu*
        if(m_match_model_ == 1){
            //局部激光匹配结果计数加1
            kLocalSlamMatchingResults->Increment();
            //lyy
            int matched_submap_id = local_trajectory_builder_->GetMatchedSubmapId();
            int tmp_node_index = int(1000.0f * local_trajectory_builder_->GetBestCandidateScore());
            // std::unique_ptr<InsertionResult> insertion_result = absl::make_unique<InsertionResult>(
            //                                 InsertionResult{
            //                                 NodeId(matched_submap_id, tmp_node_index), 
            //                                 nullptr, 
            //                                 {}});
            std::unique_ptr<InsertionResult> insertion_result =
                absl::make_unique<InsertionResult>(InsertionResult{
                    NodeId(matched_submap_id, tmp_node_index), nullptr,
                    std::vector<std::shared_ptr<const Submap>>(
                    matching_result->insertion_result->insertion_submaps.begin(),
                    matching_result->insertion_result->insertion_submaps.end())});

            //如果激光数据成功插入submap中
            if (matching_result->insertion_result != nullptr) {
                //激光插入计数加1
                kLocalSlamInsertionResults->Increment();
            }
            if (local_slam_result_callback_) {
                local_slam_result_callback_(
                            trajectory_id_, matching_result->time, matching_result->local_pose,
                            std::move(matching_result->range_data_in_local),
                            std::move(insertion_result)/* nullptr  lyy */);
            }
        }else{
            //局部激光匹配结果计数加1
            kLocalSlamMatchingResults->Increment();
            std::unique_ptr<InsertionResult> insertion_result;
            //如果激光数据成功插入submap中
            if (matching_result->insertion_result != nullptr) {
                //激光插入计数加1
                kLocalSlamInsertionResults->Increment();
                //添加节点约束
                auto node_id = pose_graph_->AddNode(
                            matching_result->insertion_result->constant_data, trajectory_id_,
                            matching_result->insertion_result->insertion_submaps);

                CHECK_EQ(node_id.trajectory_id, trajectory_id_);
                //生成一份插入结果
                insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
                                                                          node_id, matching_result->insertion_result->constant_data,
                                                                          std::vector<std::shared_ptr<const Submap>>(
                                                                          matching_result->insertion_result->insertion_submaps.begin(),
                                                                          matching_result->insertion_result->insertion_submaps.end())});
            }

            //记录局部slam结果
            if (local_slam_result_callback_) {
                local_slam_result_callback_(
                            trajectory_id_, matching_result->time, matching_result->local_pose,
                            std::move(matching_result->range_data_in_local),
                            std::move(insertion_result));
            }
        }
    }

    void AddSensorData(const std::string& /*sensor_id*/,
                       const sensor::ImuData& imu_data) override {
        if (local_trajectory_builder_) {
            local_trajectory_builder_->AddImuData(imu_data);
        }
        if (m_match_model_ != 1) {  // ph repair Memory Leak‌
            pose_graph_->AddImuData(trajectory_id_, imu_data);
        }
    }

    void AddSensorData(const std::string& /*sensor_id*/,
                       const sensor::OdometryData& odometry_data) override {
        CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
        if (local_trajectory_builder_) {
            // LOG(INFO)<<"614 FOR ODOM7";
            local_trajectory_builder_->AddOdometryData(odometry_data);
        }

        //odom数据加入优化效果不好,位移和旋转权重要减小，甚至设置为0
        if (m_match_model_ != 1) { //ph repair Memory Leak‌
            pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
        }
    }

    void AddSensorData(
            const std::string& /*sensor_id*/,
            const sensor::FixedFramePoseData& fixed_frame_pose) override {
        if (fixed_frame_pose.pose.has_value()) {
            CHECK(fixed_frame_pose.pose.value().IsValid())
                    << fixed_frame_pose.pose.value();
        }
        if (m_match_model_ != 1) {  // ph repair Memory Leak‌
            pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
        }
    }

    void AddSensorData(const std::string& /*sensor_id*/,
                       const sensor::LandmarkData& landmark_data) override {
        if (m_match_model_ != 1) {  // ph repair Memory Leak‌
            pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
        }
    }

    //lyy
    void AddSubmapIdAndFlag(int submap_id, bool assigned_flag) override {
        submap_id_ = submap_id;
        assigned_flag_ = assigned_flag;
    }

    void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                local_slam_result_data) override {
        CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                             "local_trajectory_builder_ present.";
        local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
    }

private:
    //penghu*
    SlamCommon::CFileInterface *file_interface_ = nullptr;
    SlamCommon::CConfigFileOperator *config_operator_ = nullptr;
    int m_match_model_ = 1;
    const int trajectory_id_;
    PoseGraph* const pose_graph_;
    //LocalTrajectoryBuilder2D
    std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;
    LocalSlamResultCallback local_slam_result_callback_;
    int submap_id_;//lyy
    bool assigned_flag_;//lyy
};

}  // namespace

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
        SlamCommon::CFileInterface *file_interface,
        SlamCommon::CConfigFileOperator *config_operator,
        std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
        const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
        const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback) {
    return absl::make_unique<
            GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(   file_interface, config_operator,
                                                                                        std::move(local_trajectory_builder), trajectory_id, pose_graph,
                                                                                        local_slam_result_callback);
}

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
        SlamCommon::CFileInterface *file_interface,
        SlamCommon::CConfigFileOperator *config_operator,
        std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
        const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
        const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback) {
    return absl::make_unique<
            GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>(   file_interface, config_operator,
                                                                                        std::move(local_trajectory_builder), trajectory_id, pose_graph,
                                                                                        local_slam_result_callback);
}

void GlobalTrajectoryBuilderRegisterMetrics(metrics::FamilyFactory* factory) {
    auto* results = factory->NewCounterFamily(
                "mapping_global_trajectory_builder_local_slam_results",
                "Local SLAM results");
    kLocalSlamMatchingResults = results->Add({{"type", "MatchingResult"}});
    kLocalSlamInsertionResults = results->Add({{"type", "InsertionResult"}});
}

}  // namespace mapping
}  // namespace cartographer
