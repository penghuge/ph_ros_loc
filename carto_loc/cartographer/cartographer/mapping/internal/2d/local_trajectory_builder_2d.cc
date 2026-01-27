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
#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/transform/transform.h"


namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
        metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
        SlamCommon::CFileInterface *file_interface,
        SlamCommon::CConfigFileOperator *config_operator,
        const proto::LocalTrajectoryBuilderOptions2D& options,
        const std::vector<std::string>& expected_range_sensor_ids)
    : m_file_interface(file_interface),
      m_config_operator(config_operator),
      options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {

    std::vector<double> vec;
    if(!m_file_interface->GetLocalTrajectoryBuilder2dOptions(vec)){
        LOG_ERROR("GetLocalTrajectoryBuilder2dOptions failed!");
    }else{
        LOG_INFO("GetLocalTrajectoryBuilder2dOptions success!");
    }

    if (vec.size() == 8){
        m_initial_scan_match_min_score_threshold = vec[0];
        m_initial_scan_match_confidence_score_threshold = vec[1];
        m_use_ceres_to_refine = vec[2];
        m_initial_scan_match_linear_window = vec[3];
        m_initial_scan_match_angle_window = vec[4];
        m_initial_scan_match_min_time = vec[5];
        appy_ = vec[6];
        one_scan_ = vec[7];
        LOG_INFO("m_initial_scan_match_min_score_threshold: %f, m_initial_scan_match_confidence_score_threshold: %f, m_use_ceres_to_refine: %d, "
            "m_initial_scan_match_linear_window: %f, m_initial_scan_match_angle_window: %f, m_initial_scan_match_min_time: %f, appy_: %d, one_scan_: %d", 
            m_initial_scan_match_min_score_threshold, m_initial_scan_match_confidence_score_threshold, m_use_ceres_to_refine, 
            m_initial_scan_match_linear_window, m_initial_scan_match_angle_window, m_initial_scan_match_min_time, appy_, one_scan_);
    }
    m_match_model_ = m_file_interface->GetPureLocalization();
    if(m_match_model_ == 4) m_match_model_ = 2;
    LOG_INFO("m_match_model_ = %d", m_match_model_);

    if(m_match_model_ == 1 || m_match_model_ == 3){
        LOG_INFO("m_match_model_: %d, m_init_global_scan_match constructed!!", m_match_model_);
        m_init_global_scan_match = new cartographer::mapping::global_scan_matcher_2d_local();
        m_init_global_scan_match->Init(options_,
                                    m_initial_scan_match_min_score_threshold,
                                    m_initial_scan_match_confidence_score_threshold,
                                    m_state_filename, m_use_ceres_to_refine, one_scan_, m_file_interface, m_config_operator);
    }

    expand_option_.options = active_submaps_.Getoptions().options;
    expand_option_.submaps = active_submaps_.Getoptions().submaps;
}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {
    //penghu 24/4/21
    m_stop_recv = true;
}
//penghu 24/4/12
bool LocalTrajectoryBuilder2D::is_seed_ = false;
SlamCommon::Pose3D LocalTrajectoryBuilder2D::seed_pose_ = {0., 0., 0.};


//penghu 24/4/21
bool LocalTrajectoryBuilder2D::handdata( std::vector<std::uint16_t> &yuntai_data,
                                         sensor::RangeData &rangedata)
{
    float angle = 0;
    if(yuntai_data.size() != 1200)
    {
        //LOG_INFO("yuntai_data no match");
        return false;
    }
    for (size_t i = 0; i < yuntai_data.size(); ++i)
    {
        const float first_echo = yuntai_data[i] * 0.001;
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const cartographer::sensor::RangefinderPoint point{ rotation * (first_echo * Eigen::Vector3f::UnitX())};
        rangedata.returns.push_back(point);
        angle += SlamCommon::DegToRad(0.3);  //0.0052
    }
    return true;
}
//penghu 24/5/08
std::vector<std::shared_ptr<const Submap2D>> LocalTrajectoryBuilder2D::ExpandSubmap(const sensor::RangeData& range_data_in_local)
{
    //LOG_INFO("yuntai_insert submap");
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
            active_submaps_.InsertRangeData(range_data_in_local);
    return insertion_submaps;
}
//penghu 24/5/08
bool LocalTrajectoryBuilder2D::expand(SlamCommon::Pose3D &yun_pose)
{
    //LOG_INFO("yuntai_419 expand(%f, %f,%f)", yun_pose.x, yun_pose.y, yun_pose.theta);
    std::string path = m_file_interface->GetCurrentPath();
    path += DIR_DOCS_ID + std::string(DIR_FLAG) + DOCS_MAP_ID + std::string(DIR_FLAG);
    const std::string pbstream_filename = absl::StrCat("318", ".pbstream");

    SlamCommon::OccupancyGrid occu_grid;
    occu_grid.resolution = 0.05;
    occu_grid.header.frame_id = std::string("odometry");
    occu_grid.occupied_threash = 0.51;
    occu_grid.free_threash = 0.49;
    occu_grid.negate = false;

    ::cartographer::io::ProtoStreamReader reader(path + pbstream_filename);
    ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

    //LOG_INFO("yuntai_Loading submap slices from serialized data: %s.", pbstream_filename.c_str());
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
            submap_slices;
    ::cartographer::mapping::ValueConversionTables conversion_tables;
    ::cartographer::io::DeserializeAndFillSubmapSlices(
                &deserializer, &submap_slices, &conversion_tables);

    //add submap
    //LOG_INFO("yuntai_start add submap");
    const ::cartographer::mapping::SubmapId submap_id{42, 42};
    submap_slices[submap_id].pose = cartographer::transform::Rigid3d(
    {yun_pose.x, yun_pose.y, 0.},
                cartographer::transform::AngleAxisVectorToRotationQuaternion(
                    Eigen::Vector3d(0., 0., yun_pose.theta)));
    ::CartoMapppingBridge::SlamSubmapQueryRequestStru request;
    ::CartoMapppingBridge::SlamSubmapQueryResponseStru response;
    request.trajectory_id = submap_id.trajectory_id;
    request.submap_index = submap_id.submap_index;

    cartographer::mapping::proto::SubmapQuery::Response response_proto;
    if(expand_result_.front() != nullptr){
        //LOG_INFO("yuntai_have submap data");
        submap_ = std::move(expand_result_.front());
        submap_->ToResponseProto(submap_slices[submap_id].pose, &response_proto);
    }
    else
    {
        //LOG_INFO("yuntai no submap data");
        return false;
    }

    response.submap_version = response_proto.submap_version();
    for (const auto& texture_proto : response_proto.textures())
    {
        //LOG_INFO("yuntai_add texture..");
        response.textures.emplace_back();
        auto& texture = response.textures.back();
        texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                             texture_proto.cells().end());
        texture.width = texture_proto.width();
        texture.height = texture_proto.height();
        texture.resolution = texture_proto.resolution();
        texture.slice_pose = ::CartoMapppingBridge::ToGeometryMsgPose(
                    cartographer::transform::ToRigid3(texture_proto.slice_pose()));
    }
    response.status.message = "Success.";
    response.status.code = ::CartoMapppingBridge::SlamStatusResponseStru::OK;
    auto submap_textures = absl::make_unique<::cartographer::io::SubmapTextures>();
    submap_textures->version = response.submap_version;
    for (const auto& texture : response.textures)
    {
        const std::string compressed_cells(texture.cells.begin(),
                                           texture.cells.end());
        submap_textures->textures.emplace_back(::cartographer::io::SubmapTexture{
                                                   ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                                   texture.height),
                                                   texture.width, texture.height, texture.resolution,
                                                   CartoMapppingBridge::ToRigid3d(texture.slice_pose)});
    }
    //LOG_INFO("yuntai_get submap_textures");
    if (submap_textures == nullptr)
    {
        //LOG_INFO("yuntai_ERROR submap_textures");
        return false;
    }
    CHECK(!submap_textures->textures.empty());
    submap_slices[submap_id].version = submap_textures->version;
    const auto fetched_texture = submap_textures->textures.begin();
    submap_slices[submap_id].width = fetched_texture->width;
    submap_slices[submap_id].height = fetched_texture->height;
    submap_slices[submap_id].slice_pose = fetched_texture->slice_pose;
    submap_slices[submap_id].resolution = fetched_texture->resolution;
    submap_slices[submap_id].cairo_data.clear();
    submap_slices[submap_id].surface = ::cartographer::io::DrawTexture(
                fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
                fetched_texture->width, fetched_texture->height,
                &submap_slices[submap_id].cairo_data);
    //start expand
    //LOG_INFO("yuntai_Generating combined map image from submap slices.");
    auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, occu_grid.resolution);
    //LOG_INFO("yuntai_strat get pgm1");
    ::CartoMapppingBridge::CreateOccupancyGridMsg(result, occu_grid.resolution, occu_grid.header.frame_id,
                                                  SlamCommon::TimeNow(), &occu_grid);
    //LOG_INFO("yuntai_strat get pgm2");
    m_file_interface->SaveExpandMapData("expand", occu_grid);

    //penghu 24/5/9
    if(appy_)
    {
        //
       m_occupancy_grid = new SlamCommon::OccupancyGrid();
       m_occupancy_grid = m_file_interface->GetExpandGrid("expand");
       if (nullptr == m_occupancy_grid)
       {
           return false;
       }

        std::string pa = m_file_interface->GetCurrentPath();
        pa += DIR_DOCS_ID + std::string(DIR_FLAG) + DOCS_EXPAND_MAP_ID + std::string(DIR_FLAG);
        const std::string phstream_filename = absl::StrCat("516", ".pbstream");
        //test
        // 反序列化 .pbstream 文件
        cartographer::io::ProtoStreamReader reader(path + pbstream_filename);
        cartographer::io::ProtoStreamDeserializer deserializer(&reader);
        cartographer::io::ProtoStreamWriter writer(pa + phstream_filename);

        cartographer::mapping::proto::SerializedData data;
        cartographer::mapping::proto::SerializedData last_submap_data;

        // 读取 kSubmap 之前的所有数据并写入新文件
        while (deserializer.ReadNextSerializedData(&data)) {
            // 将读取到的数据写入新文件
            writer.WriteProto(data);
            // last_submap_data = data;
        }

        // 确保所有数据都已写入并关闭 writer
        if (!writer.Close()) {
            // LOG_ERROR("Failed to close the output pbstream file.");
            return false;
        }
        last_submap_data = data;

        //penghu test
        // Validate the written file
        {
            ::cartographer::io::ProtoStreamReader ph(pa + phstream_filename);
            ::cartographer::mapping::proto::SerializedData ph_data;
            while(ph.ReadProto(&ph_data))
            {
                //LOG_INFO("Validated data case: %d", ph_data.data_case());
            }
        }

        if(m_match_model_ == 1){
            m_init_global_scan_match->Init(options_,
                                        m_initial_scan_match_min_score_threshold,
                                        m_initial_scan_match_confidence_score_threshold,
                                        pa + phstream_filename, m_use_ceres_to_refine, one_scan_, m_file_interface, m_config_operator);
        }

    }
    return true;
}
//penghu 24/4/21 end

sensor::RangeData LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
        const transform::Rigid3f& transform_to_gravity_aligned_frame,
        const sensor::RangeData& range_data) const {
    //所有激光点根据重力方向，只取有效高度范围内的点
    const sensor::RangeData cropped =
            sensor::CropRangeData(sensor::TransformRangeData(
                                      range_data, transform_to_gravity_aligned_frame),
                                  options_.min_z(), options_.max_z());
    return sensor::RangeData{
        cropped.origin,
                //设定voxel大小，对激光进行过滤
                sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.returns),
                sensor::VoxelFilter(options_.voxel_filter_size()).Filter(cropped.misses)};
}

std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
        const common::Time /*time*/, const transform::Rigid2d& pose_prediction,
        const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
    //确保活动submap中存在submap
    if (active_submaps_.submaps().empty()) {
        return absl::make_unique<transform::Rigid2d>(pose_prediction);
    }
    //penghu find matching_submap
    //  std::shared_ptr<const Submap2D> matching_submap =
    //取第一张活动地图进行激光匹配
    std::shared_ptr<const Submap2D> matching_submap =
            active_submaps_.submaps().front();
    // The online correlative scan matcher will refine the initial estimate for
    // the Ceres scan matcher.
    //在线相关性激光匹配，计算量非常大，但是可以得到一个非常准确的预测位姿，提升建图效果
    transform::Rigid2d initial_ceres_pose = pose_prediction;

    if (options_.use_online_correlative_scan_matching()) {
        const double score = real_time_correlative_scan_matcher_.Match(
                    pose_prediction, filtered_gravity_aligned_point_cloud,
                    *matching_submap->grid(), &initial_ceres_pose);
        kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
    }

    auto pose_observation = absl::make_unique<transform::Rigid2d>();
    ceres::Solver::Summary summary;
    //调用ceres进行激光匹配，以获取与预测位姿最接近，且匹配效果最好的位姿，保存在pose_observation中
    ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                              filtered_gravity_aligned_point_cloud,
                              *matching_submap->grid(), pose_observation.get(),
                              &summary, 0.);
    if (pose_observation) {
        //将代价值、距离残差、角度残差记录下来
        kCeresScanMatcherCostMetric->Observe(summary.final_cost);
        const double residual_distance =
                (pose_observation->translation() - pose_prediction.translation())
                .norm();
        kScanMatcherResidualDistanceMetric->Observe(residual_distance);
        const double residual_angle =
                std::abs(pose_observation->rotation().angle() -
                         pose_prediction.rotation().angle());
        kScanMatcherResidualAngleMetric->Observe(residual_angle);
    }
    return pose_observation;
    //    return absl::make_unique<transform::Rigid2d>(initial_ceres_pose);
}

//penghu*
void LocalTrajectoryBuilder2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                                        const int to_trajectory_id,
                                                        const transform::Rigid3d& pose,
                                                        const common::Time time) {
    absl::MutexLock locker(&mutex_);
    m_init_pose = pose;
    LOG(INFO) << "Set init pose, traj id(" << from_trajectory_id << " >> " << to_trajectory_id << "), "
              << "init pose(" << pose.translation().x() << ", " << pose.translation().y()
              << ", " << transform::RadToDeg(transform::GetYaw(pose.rotation()))<< ")";
}

//penghu 24/3/29
bool LocalTrajectoryBuilder2D::SetSeedPose(SlamCommon::Pose3D seedpose)
{
    // absl::MutexLock locker(&mutex_);
    LocalTrajectoryBuilder2D::is_seed_ = true;
    LocalTrajectoryBuilder2D::seed_pose_ = seedpose;
    //LOG_INFO("seed_pose_(%f, %f, %f)", seed_pose_.x, seed_pose_.y, seed_pose_.theta);
    //LOG_INFO("is seed = %d", is_seed_);
    return true;
}

std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
        const std::string& sensor_id,
        const sensor::TimedPointCloudData& unsynchronized_data) {

    //同步新的激光数据，也就是拼接所有激光束，并按时间戳排序
    auto synchronized_data =
            range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
    if (synchronized_data.ranges.empty()) {
        LOG(INFO) << "Range data collator filling buffer.";
        return nullptr;
    }

    //时间标记为同步完的激光的测量时间
    const common::Time& time = unsynchronized_data.time;
    // std::cout << "time " << time << std::endl;
    // LOG_DEBUG("synchronized_scan_data_time = %f, synchronized_data.ranges.size() = %d",
    //           cartographer::common::ToSeconds(time.time_since_epoch()), synchronized_data.ranges.size());
    // Initialize extrapolator now if we do not ever use an IMU.
    if (!options_.use_imu_data()) {
        InitializeExtrapolator(time);
    }

    if (extrapolator_ == nullptr) {
        // Until we've initialized the extrapolator with our first IMU message, we
        // cannot compute the orientation of the rangefinder.
        //在我们用第一帧IMU消息初始化位姿估算器之前，我们无法计算测距仪的方向。
        LOG(INFO) << "Extrapolator not yet initialized.";
        return nullptr;
    }

    CHECK(!synchronized_data.ranges.empty());
    // TODO(gaschler): Check if this can strictly be 0.
    //之前已经将最后一帧激光束的时间点计算成0，越早负的越多
    CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
    //第一个激光点的时间戳
    const common::Time time_first_point =
            time +
            common::FromSeconds(synchronized_data.ranges.front().point_time.time);
    //如果第一帧激光束比位姿推测器中最后一次推测出的位姿的时间还早，该轮激光数据还是不能要，
    //因为要先预测，才能进行激光匹配
    if (time_first_point < extrapolator_->GetLastPoseTime()) {
        LOG(INFO) << "Extrapolator is still initializing.";
        return nullptr;
    }

    //计算每个激光点的时间戳所对应的机器人位姿
    std::vector<transform::Rigid3f> range_data_poses;
    range_data_poses.reserve(synchronized_data.ranges.size());
    bool warned = false;
    for (const auto& range : synchronized_data.ranges) {
        common::Time time_point = time + common::FromSeconds(range.point_time.time);
        if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
            if (!warned) {
                LOG(ERROR)
                        << "Timestamp of individual range data point jumps backwards from "
                        << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
                warned = true;
            }
            time_point = extrapolator_->GetLastExtrapolatedTime();
        }
        range_data_poses.push_back(
                    extrapolator_->ExtrapolatePose(time_point).cast<float>());
    }

    if (num_accumulated_ == 0) {
        // 'accumulated_range_data_.origin' is uninitialized until the last
        // accumulation.
        accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
    }

    // Drop any returns below the minimum range and convert returns beyond the
    // maximum range into misses.
    for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
        const sensor::TimedRangefinderPoint& hit =
                synchronized_data.ranges[i].point_time;
        //激光原点在局部坐标系下的坐标
        const Eigen::Vector3f origin_in_local =
                range_data_poses[i] *
                synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
        //激光点在局部坐标系下的坐标
        sensor::RangefinderPoint hit_in_local =
                range_data_poses[i] * sensor::ToRangefinderPoint(hit);
        const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
        const float range = delta.norm();
        //小于最小距离的测距数据会被丢弃
        if (range >= options_.min_range()) {
            //测距数据在最大/最小范围内的数据，直接保存
            if (range <= options_.max_range()) {
                accumulated_range_data_.returns.push_back(hit_in_local);
            }
            //超过最大距离的数据，用参数文件中missing_data_ray_length代替
            else {
                hit_in_local.position =
                        origin_in_local +
                        options_.missing_data_ray_length() / range * delta;
                accumulated_range_data_.misses.push_back(hit_in_local);
            }
        }
    }

    // 低频统计（AddRangeData 阶段）：这里是“最原始”点数口径（未 Crop / 未 VoxelFilter / 未 AdaptiveVoxelFilter）。
    // 说明：此处已经做了 min_range/max_range 处理：
    //   - returns: [min_range, max_range] 内
    //   - misses : > max_range 被截断到 missing_data_ray_length
    {
        static common::Time last_log_time_raw = common::Time::min();
        if (last_log_time_raw == common::Time::min() ||
            (time - last_log_time_raw) >= common::FromSeconds(1.0)) {
            last_log_time_raw = time;
            const int unsynced_ranges = static_cast<int>(unsynchronized_data.ranges.size());
            const int synced_ranges = static_cast<int>(synchronized_data.ranges.size());
            const int raw_returns_acc = static_cast<int>(accumulated_range_data_.returns.size());
            const int raw_misses_acc = static_cast<int>(accumulated_range_data_.misses.size());

            LOG(INFO) << "[LTB2D] raw_points(AddRangeData)"
                      << " sensor_id=" << sensor_id
                      << " unsynced_ranges="
                      << unsynced_ranges
                      << " synced_ranges="
                      << synced_ranges
                      << " accumulated_returns="
                      << raw_returns_acc
                      << " accumulated_misses="
                      << raw_misses_acc
                      << " num_accumulated="
                      << (static_cast<int>(num_accumulated_) + 1)
                      << "/"
                      << static_cast<int>(options_.num_accumulated_range_data());
        }
    }

    ++num_accumulated_;

    //如果激光点数比较少，可以通过设置num_accumulated_range_data，来多累加几帧，
    //因为，激光数据越多，建图效果越好
    if (num_accumulated_ >= options_.num_accumulated_range_data()) {
        const common::Time current_sensor_time = synchronized_data.time;
        //所有累计激光的时间跨度
        absl::optional<common::Duration> sensor_duration;
        if (last_sensor_time_.has_value()) {
            sensor_duration = current_sensor_time - last_sensor_time_.value();
        }
        last_sensor_time_ = current_sensor_time;
        num_accumulated_ = 0;
        const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
                    extrapolator_->EstimateGravityOrientation(time));
        // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
        // 'time'.
        accumulated_range_data_.origin = range_data_poses.back().translation();
        return AddAccumulatedRangeData(
                    time,
                    TransformToGravityAlignedFrameAndFilter(
                        gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
                        accumulated_range_data_),
                    gravity_alignment, sensor_duration);
    }
    return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
        const common::Time time,
        const sensor::RangeData& gravity_aligned_range_data,
        const transform::Rigid3d& gravity_alignment,
        const absl::optional<common::Duration>& sensor_duration) {
    if (gravity_aligned_range_data.returns.empty()) {
        LOG(WARNING) << "Dropped empty horizontal range data.";
        return nullptr;
    }

    // Computes a gravity aligned pose prediction.
    const transform::Rigid3d non_gravity_aligned_pose_prediction =
            extrapolator_->ExtrapolatePose(time);

    const transform::Rigid2d pose_prediction = transform::Project2D(
                non_gravity_aligned_pose_prediction * gravity_alignment.inverse());
    LOG_INFO_THROTTLE(
        1.0,
        "Step1、ExtrapolatePose.x,y,yaw: %f, %f, %f, Extrapolate_gravity_alignment: %f,%f,%f, "
        "scan_before_pose(%f,%f,%f)",
        non_gravity_aligned_pose_prediction.translation().x(), non_gravity_aligned_pose_prediction.translation().y(),
        cartographer::transform::GetYaw(non_gravity_aligned_pose_prediction), gravity_alignment.translation().x(),
        gravity_alignment.translation().y(), cartographer::transform::GetYaw(gravity_alignment),
        pose_prediction.translation().x(), pose_prediction.translation().y(), pose_prediction.rotation().angle());
    //自适应体素滤波
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
            sensor::AdaptiveVoxelFilter(options_.adaptive_voxel_filter_options())
            .Filter(gravity_aligned_range_data.returns);

    // 低频统计：帮助评估降采样对点数/实时性的影响（每秒最多 1 条）
    // 注意：gravity_aligned_range_data 已经在 TransformToGravityAlignedFrameAndFilter() 中做过裁剪 + VoxelFilter。
    {
        static common::Time last_log_time = common::Time::min();
        if (last_log_time == common::Time::min() ||
            (time - last_log_time) >= common::FromSeconds(1.0)) {
            last_log_time = time;

            // 未降采样前：这里我们能拿到的“最早阶段”是 gravity_aligned_range_data（已裁剪高度+范围、并做过 VoxelFilter）。
            // 真正的“最原始（未裁剪/未Voxel）”点数在 AddRangeData() 累积阶段，还需要额外埋点才能拿到。
            const int pre_adaptive_returns = static_cast<int>(gravity_aligned_range_data.returns.size());
            const int pre_adaptive_misses  = static_cast<int>(gravity_aligned_range_data.misses.size());
            const int after_adaptive_returns = static_cast<int>(filtered_gravity_aligned_point_cloud.size());

            LOG(INFO) << "[LTB2D+++++++++++++++] points pre_adaptive(=after_voxel) returns="
                      << pre_adaptive_returns << " misses=" << pre_adaptive_misses
                      << " after_adaptive=" << after_adaptive_returns
                      << " voxel_filter_size=" << options_.voxel_filter_size()
                      << " adaptive(max_len="
                      << options_.adaptive_voxel_filter_options().max_length()
                      << ",min_pts="
                      << options_.adaptive_voxel_filter_options().min_num_points()
                      << ",max_range="
                      << options_.adaptive_voxel_filter_options().max_range()
                      << ") use_online_corr="
                      << (options_.use_online_correlative_scan_matching() ? 1 : 0);
        }
    }

    if (filtered_gravity_aligned_point_cloud.empty()) {
        return nullptr;
    }
    m_start_mach_time = SlamCommon::TimeNow();
    TestResult test_result;
    test_result.pose.has_intial_guess = true;
    if (m_first_scan) {
        if (first_scan_filter_count_ > 5) {
            first_scan_filter_count_ = 0;
            m_first_scan = false;
        }
        LOG_INFO_THROTTLE(1.0, "m_init_pose (%f, %f, %f), irst_scan_filter_count: %d", m_init_pose.translation().x(),
                          m_init_pose.translation().y(),
                          SlamCommon::RadToDeg(cartographer::transform::GetYaw(m_init_pose)), first_scan_filter_count_);
        test_result.pose.rigid = m_init_pose;
        first_scan_filter_count_++;
    } else if (LocalTrajectoryBuilder2D::is_seed_) {
        LocalTrajectoryBuilder2D::is_seed_ = false;
        test_result.pose.rigid = cartographer::transform::Rigid3d(
            {LocalTrajectoryBuilder2D::seed_pose_.x, LocalTrajectoryBuilder2D::seed_pose_.y, 0.},
            cartographer::transform::AngleAxisVectorToRotationQuaternion(
                Eigen::Vector3d(0., 0., LocalTrajectoryBuilder2D::seed_pose_.theta)));
        LOG_INFO_THROTTLE(1.0, "seed pose(%f, %f, %f), time1 = %fs", test_result.pose.rigid.translation().x(),
                          test_result.pose.rigid.translation().y(),
                          SlamCommon::RadToDeg(cartographer::transform::GetYaw(test_result.pose.rigid)),
                          cartographer::common::ToSeconds(time.time_since_epoch()));
    } else {
        test_result.pose.rigid = non_gravity_aligned_pose_prediction;
    }
    test_result.pose.search_window = m_initial_scan_match_linear_window;
    test_result.pose.angle_window = m_initial_scan_match_angle_window;
    test_result.pose.min_search_time = m_initial_scan_match_min_time;

    switch (m_match_model_) {  // 1: pure localize; 2: mapping; 3: expand; 
        case 1: {
            LOG_INFO_THROTTLE(1.0, "Step2、m_window(%f, %f), m_init_global_scan_match->Match_before_pose =(%f, %f, %f)",
                              m_initial_scan_match_linear_window, m_initial_scan_match_angle_window,
                              test_result.pose.rigid.translation().x(), test_result.pose.rigid.translation().y(),
                              cartographer::transform::GetYaw(test_result.pose.rigid));
            cartographer::transform::Rigid3d local_to_global;                          // lyy
            m_init_global_scan_match->AddSubmapIdAndFlag(submap_id_, assigned_flag_);  // lyy
            if (!m_init_global_scan_match->Match(gravity_alignment, filtered_gravity_aligned_point_cloud,
                                                 &(test_result.pose), m_file_interface->GetVel(), local_to_global)) {
                LOG_ERROR("Falied to locate in global map!");
                test_result.point_data = filtered_gravity_aligned_point_cloud;
                test_result.flag = false;
                return nullptr;
            }
            auto gravity_aligned_range_data_pose = local_to_global * test_result.pose.rigid;

            m_file_interface->SetCore(test_result.pose.prob);
            test_result.flag = true;
            test_result.pose.rigid = gravity_aligned_range_data_pose * gravity_alignment;  // lyy
            LOG_INFO_THROTTLE( 1.0, "++++++++++++++++Match_use time_1=%fs!", SlamCommon::ToSeconds(SlamCommon::TimeNow() - m_start_mach_time));

            //----------------------------------------------------------------
            // 定位输出二、建图匹配结果（单子图构建模式）
            const auto submap_build_start = std::chrono::steady_clock::now();
            // ScanMatch 一定只使用当前唯一的活跃子图（ScanMatch 内部用 active_submaps_.submaps().front()）。
            std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
                ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);
            if (pose_estimate_2d == nullptr) {
                return nullptr;
            }

            // 产生用于插图的位姿：若当前子图是首帧（num_range_data==0），强制使用 global-match 位姿。
            // 注意：ActiveSubmaps2D 的单子图实现会在子图完成后 clear()，下一帧会创建新子图，num_range_data 会从 0
            // 重新开始。
            const bool is_first_range_data_in_submap =
                active_submaps_.submaps().empty() || active_submaps_.submaps().front()->num_range_data() == 0;

            transform::Rigid3d pose_estimate = transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
            if (is_first_range_data_in_submap) {
                pose_estimate = test_result.pose.rigid;
            }

            // test_result.flag = true;
            // test_result.pose.rigid = pose_estimate;

            sensor::RangeData range_data_in_local_for_insert =
                TransformRangeData(gravity_aligned_range_data, transform::Embed3D(pose_estimate_2d->cast<float>()));

            std::unique_ptr<InsertionResult> insertion_result =
                InsertSingleMapIntoSubmap(time, range_data_in_local_for_insert, filtered_gravity_aligned_point_cloud,
                                         pose_estimate, gravity_alignment.rotation());
            LOG_INFO_THROTTLE( 1.0, "++++++++++++++++Match_use time_2=%fs!", SlamCommon::ToSeconds(SlamCommon::TimeNow() - m_start_mach_time));
            // const auto submap_build_end = std::chrono::steady_clock::now();
            // const double submap_build_cost_ms = 1000. * common::ToSeconds(submap_build_end - submap_build_start);
            // 1Hz 低频打印：避免每帧刷屏。
            // static auto s_last_print_time = std::chrono::steady_clock::now();
            // static double s_cost_ms_sum = 0.0;
            // static int s_cost_ms_cnt = 0;
            // s_cost_ms_sum += submap_build_cost_ms;
            // ++s_cost_ms_cnt;
            // if (common::ToSeconds(submap_build_end - s_last_print_time) >= 1.0) {
            //     const double avg_ms = s_cost_ms_cnt > 0 ? (s_cost_ms_sum / s_cost_ms_cnt) : 0.0;
            //     LOG_INFO(
            //         "[LTB2D] submap_build_cost_ms(avg_1s)=%f last_ms=%f (scan_match+insert) is_first_in_submap=%d "
            //         "active_submaps=%zu",
            //         avg_ms, submap_build_cost_ms, is_first_range_data_in_submap ? 1 : 0,
            //         active_submaps_.submaps().size());
            //     s_last_print_time = submap_build_end;
            //     s_cost_ms_sum = 0.0;
            //     s_cost_ms_cnt = 0;
            // }
            //----------------------------------------------------------------

            extrapolator_->AddPose(time, test_result.pose.rigid);
            m_best_pose.Reset(test_result.pose.rigid.translation().x(), test_result.pose.rigid.translation().y(),
                              cartographer::transform::GetYaw(test_result.pose.rigid));
            LOG_INFO_THROTTLE(
                1.0, "ph_step_3: success locate in global map , Mach_after_best pose(%f, %f, %f), Match_use time_3=%fs!",
                m_best_pose.x, m_best_pose.y, m_best_pose.theta,
                SlamCommon::ToSeconds(SlamCommon::TimeNow() - m_start_mach_time));
            sensor::RangeData range_data_in_local =
                TransformRangeData(gravity_aligned_range_data, gravity_aligned_range_data_pose.cast<float>());

            return absl::make_unique<MatchingResult>(
                MatchingResult{time, test_result.pose.rigid, std::move(range_data_in_local), nullptr});
            }
        case 2: {
            //  local map frame <- gravity-aligned frame
            std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
                ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);
            // LOG_INFO("scan_later_pose(%f,%f,%f)",pose_estimate_2d->translation().x(),
            //          pose_estimate_2d->translation().y(), pose_estimate_2d->rotation().angle());

            if (pose_estimate_2d == nullptr) {
                LOG(WARNING) << "Scan matching failed.";
                return nullptr;
            }
            // 经过优化后的估计位姿
            const transform::Rigid3d pose_estimate = transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
            // LOG_INFO("optimizePose_x,y,yaw,gravity_alignment =(%f,%f,%f,%f)", pose_estimate.translation().x(),
            //          pose_estimate.translation().y(), pose_estimate.rotation().w(),
            //          gravity_alignment.rotation().w());
            // 将优化位姿反向更新给位姿推算器
            extrapolator_->AddPose(time, pose_estimate);

            // 向submap中插入激光数据
            sensor::RangeData range_data_in_local =
                TransformRangeData(gravity_aligned_range_data, transform::Embed3D(pose_estimate_2d->cast<float>()));
            std::unique_ptr<InsertionResult> insertion_result =
                InsertIntoSubmap(time, range_data_in_local, filtered_gravity_aligned_point_cloud, pose_estimate,
                                 gravity_alignment.rotation());

            // 统计数据
            const auto wall_time = std::chrono::steady_clock::now();
            if (last_wall_time_.has_value()) {
                const auto wall_time_duration = wall_time - last_wall_time_.value();
                kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
                if (sensor_duration.has_value()) {
                    kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                                 common::ToSeconds(wall_time_duration));
                }
            }
            const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
            if (last_thread_cpu_time_seconds_.has_value()) {
                const double thread_cpu_duration_seconds =
                    thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
                if (sensor_duration.has_value()) {
                    kLocalSlamCpuRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                                    thread_cpu_duration_seconds);
                }
            }
            last_wall_time_ = wall_time;
            last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
            m_best_pose.Reset(pose_estimate.translation().x(), pose_estimate.translation().y(),
                              cartographer::transform::GetYaw(pose_estimate));
            // LOG_DEBUG("success locate in global map , best pose(%f, %f, %f), use time=%fs!",
            //           m_best_pose.x, m_best_pose.y,
            //           SlamCommon::RadToDeg(m_best_pose.theta),
            //           SlamCommon::ToSeconds(SlamCommon::TimeNow() - m_start_mach_time));
            return absl::make_unique<MatchingResult>(
                MatchingResult{time, pose_estimate, std::move(range_data_in_local), std::move(insertion_result)});
        }
        case 3: {
            if (start_expand_) {
                cartographer::transform::Rigid3d local_to_global;
                m_init_global_scan_match->AddSubmapIdAndFlag(submap_id_, assigned_flag_);
                if (!m_init_global_scan_match->Match(gravity_alignment, filtered_gravity_aligned_point_cloud,
                                                     &(test_result.pose), m_file_interface->GetVel(),
                                                     local_to_global)) {
                    LOG_ERROR("Falied to locate in global map!");
                    test_result.point_data = filtered_gravity_aligned_point_cloud;
                    test_result.flag = false;
                    return nullptr;
                }
                auto gravity_aligned_range_data_pose = local_to_global * test_result.pose.rigid;

                m_file_interface->SetCore(test_result.pose.prob);
                test_result.flag = true;
                test_result.pose.rigid = gravity_aligned_range_data_pose * gravity_alignment;  // lyy
                extrapolator_->AddPose(time, test_result.pose.rigid);
                m_best_pose.Reset(test_result.pose.rigid.translation().x(), test_result.pose.rigid.translation().y(),
                                  cartographer::transform::GetYaw(test_result.pose.rigid));
                LOG_INFO_THROTTLE(
                    1.0,
                    "ph_step_3: success locate in global map , Mach_after_best pose(%f, %f, %f), Match_use time=%fs!",
                    m_best_pose.x, m_best_pose.y, m_best_pose.theta,
                    SlamCommon::ToSeconds(SlamCommon::TimeNow() - m_start_mach_time));
                sensor::RangeData range_data_in_local =
                    TransformRangeData(gravity_aligned_range_data, gravity_aligned_range_data_pose.cast<float>());
                LOG_ERROR("F----------------score =  %f!", test_result.pose.prob);
                if (start_expand_count_ > 50 && test_result.pose.prob > 0.5) {
                    start_expand_count_ = 0;
                    start_expand_ = false;
                }
                start_expand_count_ ++;
                return absl::make_unique<MatchingResult>(
                    MatchingResult{time, test_result.pose.rigid, std::move(range_data_in_local), nullptr});
            }

            const transform::Rigid2d expand_pose_prediction = transform::Project2D(test_result.pose.rigid * gravity_alignment.inverse()); //penghu for expand
            std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
                ScanMatch(time, expand_pose_prediction, filtered_gravity_aligned_point_cloud);
            // LOG_INFO("scan_later_pose(%f,%f,%f)",pose_estimate_2d->translation().x(),
            //          pose_estimate_2d->translation().y(), pose_estimate_2d->rotation().angle());

            if (pose_estimate_2d == nullptr) {
                LOG(WARNING) << "Scan matching failed.";
                return nullptr;
            }
            // 经过优化后的估计位姿
            const transform::Rigid3d pose_estimate = transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
            // LOG_INFO("optimizePose_x,y,yaw,gravity_alignment =(%f,%f,%f,%f)", pose_estimate.translation().x(),
            //          pose_estimate.translation().y(), pose_estimate.rotation().w(),
            //          gravity_alignment.rotation().w());
            // 将优化位姿反向更新给位姿推算器
            extrapolator_->AddPose(time, pose_estimate);

            // 向submap中插入激光数据
            sensor::RangeData range_data_in_local =
                TransformRangeData(gravity_aligned_range_data, transform::Embed3D(pose_estimate_2d->cast<float>()));
            std::unique_ptr<InsertionResult> insertion_result =
                InsertIntoSubmap(time, range_data_in_local, filtered_gravity_aligned_point_cloud, pose_estimate,
                                 gravity_alignment.rotation());

            // 统计数据
            const auto wall_time = std::chrono::steady_clock::now();
            if (last_wall_time_.has_value()) {
                const auto wall_time_duration = wall_time - last_wall_time_.value();
                kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
                if (sensor_duration.has_value()) {
                    kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                                 common::ToSeconds(wall_time_duration));
                }
            }
            const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
            if (last_thread_cpu_time_seconds_.has_value()) {
                const double thread_cpu_duration_seconds =
                    thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
                if (sensor_duration.has_value()) {
                    kLocalSlamCpuRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                                    thread_cpu_duration_seconds);
                }
            }
            last_wall_time_ = wall_time;
            last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
            m_best_pose.Reset(pose_estimate.translation().x(), pose_estimate.translation().y(),
                              cartographer::transform::GetYaw(pose_estimate));
            // LOG_DEBUG("success locate in global map , best pose(%f, %f, %f), use time=%fs!",
            //           m_best_pose.x, m_best_pose.y,
            //           SlamCommon::RadToDeg(m_best_pose.theta),
            //           SlamCommon::ToSeconds(SlamCommon::TimeNow() - m_start_mach_time));
            return absl::make_unique<MatchingResult>(
                MatchingResult{time, pose_estimate, std::move(range_data_in_local), std::move(insertion_result)});
        }
        default:
            return nullptr;
    }
}

std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
        const common::Time time, const sensor::RangeData& range_data_in_local,
        const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
        const transform::Rigid3d& pose_estimate,
        const Eigen::Quaterniond& gravity_alignment) {
    //如果位姿非常接近，则过滤掉本轮激光数据，返回空
    if (motion_filter_.IsSimilar(time, pose_estimate)) {
        return nullptr;
    }
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
            active_submaps_.InsertRangeData(range_data_in_local);
    return absl::make_unique<InsertionResult>(InsertionResult{
                                                  std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
                                                      time,
                                                      gravity_alignment,
                                                      filtered_gravity_aligned_point_cloud,
                                                      {},  // 'high_resolution_point_cloud' is only used in 3D.
                                                      {},  // 'low_resolution_point_cloud' is only used in 3D.
                                                      {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
                                                      pose_estimate}),
                                                  std::move(insertion_submaps)});
}

std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertSingleMapIntoSubmap(
        const common::Time time, const sensor::RangeData& range_data_in_local,
        const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
        const transform::Rigid3d& pose_estimate,
        const Eigen::Quaterniond& gravity_alignment) {
    //如果位姿非常接近，则过滤掉本轮激光数据，返回空
    if (motion_filter_.IsSimilar(time, pose_estimate)) {
        return nullptr;
    }
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
            active_submaps_.InsertSingleMapRangeData(range_data_in_local);
    return absl::make_unique<InsertionResult>(InsertionResult{
                                                  std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
                                                      time,
                                                      gravity_alignment,
                                                      filtered_gravity_aligned_point_cloud,
                                                      {},  // 'high_resolution_point_cloud' is only used in 3D.
                                                      {},  // 'low_resolution_point_cloud' is only used in 3D.
                                                      {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
                                                      pose_estimate}),
                                                  std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
    CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
    InitializeExtrapolator(imu_data.time);
    extrapolator_->AddImuData(imu_data);
}

void LocalTrajectoryBuilder2D::AddOdometryData(
        const sensor::OdometryData& odometry_data) {
    if (extrapolator_ == nullptr) {
        // Until we've initialized the extrapolator we cannot add odometry data.
        LOG(INFO) << "Extrapolator not yet initialized.";
        return;
    }
    // LOG(INFO)<<"614 for odom8";
    extrapolator_->AddOdometryData(odometry_data);
}

void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
    if (extrapolator_ != nullptr) {
        return;
    }
    // We derive velocities from poses which are at least 1 ms apart for numerical
    // stability. Usually poses known to the extrapolator will be further apart
    // in time and thus the last two are used.
    //为了使数值稳定，我们从至少相距1毫秒的位姿中推导出速度。
    //通常推测器知道的位姿在时间上有更大的距离，因此使用最后两个位姿。
    CHECK(!options_.pose_extrapolator_options().use_imu_based());
    // TODO(gaschler): Consider using InitializeWithImu as 3D does.
    extrapolator_ = absl::make_unique<PoseExtrapolator>(
                ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                                    .constant_velocity()
                                                    .pose_queue_duration()),
                options_.pose_extrapolator_options()
                .constant_velocity()
                .imu_gravity_time_constant());
    extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
        metrics::FamilyFactory* family_factory) {
    auto* latency = family_factory->NewGaugeFamily(
                "mapping_2d_local_trajectory_builder_latency",
                "Duration from first incoming point cloud in accumulation to local slam "
                "result");
    kLocalSlamLatencyMetric = latency->Add({});
    auto* real_time_ratio = family_factory->NewGaugeFamily(
                "mapping_2d_local_trajectory_builder_real_time_ratio",
                "sensor duration / wall clock duration.");
    kLocalSlamRealTimeRatio = real_time_ratio->Add({});

    auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
                "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
                "sensor duration / cpu duration.");
    kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
    auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
    auto* scores = family_factory->NewHistogramFamily(
                "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
                score_boundaries);
    kRealTimeCorrelativeScanMatcherScoreMetric =
            scores->Add({{"scan_matcher", "real_time_correlative"}});
    auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
    auto* costs = family_factory->NewHistogramFamily(
                "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
                cost_boundaries);
    kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
    auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
    auto* residuals = family_factory->NewHistogramFamily(
                "mapping_2d_local_trajectory_builder_residuals",
                "Local scan matcher residuals", distance_boundaries);
    kScanMatcherResidualDistanceMetric =
            residuals->Add({{"component", "distance"}});
    kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
