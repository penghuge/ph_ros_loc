#include "faster_csm.h"

#include <fstream>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <Eigen/Geometry>
#include <absl/memory/memory.h>
#include <cartographer/common/math.h>
#include <cartographer/mapping/2d/grid_2d.h>
#include <cartographer/sensor/point_cloud.h>
#include <cartographer/transform/transform.h>
#include <cartographer/mapping/2d/probability_grid.h>

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {

void SortCandidates(std::vector<candidate_2d::Candidate2D>* const candidates)
{
    std::sort(candidates->begin(),
              candidates->end(),
              std::greater<candidate_2d::Candidate2D>());
}
}  // namespace

FasterCSM::FasterCSM(std::deque<mapping::proto::Submap2D>* const submap2d_protos,
                     const int branch_and_bound_depth, 
                     const bool use_ceres_to_refine,
                     SlamCommon::CFileInterface *file_interface)
   : _branch_and_bound_depth(branch_and_bound_depth),
     _is_precomputed(submap2d_protos->size(), false),
     _resolution(submap2d_protos->front().grid().limits().resolution()),
     _use_ceres_to_refine(use_ceres_to_refine),
     file_interface_(file_interface)
{
    std::swap(*submap2d_protos, _submap2d_protos);

    if(!file_interface_->GetMapLimits(_limits)){
        LOG_ERROR("GetMapLimits failed!");
    }else{
        LOG_INFO("GetMapLimits success!");
    }

    if(!file_interface_->GetPrecomputationStackForCSM(_precomputation_grid_stacks)){
        LOG_ERROR("GetPrecomputationStackForCSM failed!");
    }else{
        LOG_INFO("GetPrecomputationStackForCSM success!");
    }

    if (_use_ceres_to_refine)
    {
        _ceres_sm = absl::make_unique<CeresSM>();
    }
}

bool FasterCSM::MatchInMap(const sensor::PointCloud& point_cloud,
                           const bool use_initial_guess,
                           const double search_window,
                           const double min_search_time,
                           const float min_score,
                           const float confidence_score,
                           float* const score,
                           transform::Rigid2d* const pose_estimate,
                           int& matched_submap_id,/*lyy*/
                           const std::vector<cartographer::transform::Rigid2d>& local_to_global_vec/*lyy*/)
{
    // CHECK(score && pose_estimate && matched_submap_id);//lyy
    CHECK(score && pose_estimate);//lyy
    SlamCommon::Time start_time = SlamCommon::TimeNow();

    Eigen::Vector2d initial_pose={0.0, 0.0};
    double initial_angle = 0.0;//lyy
    transform::Rigid2d tmp_pose_estimate = *pose_estimate;//lyy
    candidate_2d::Candidate2D best_candidate(0, 0, 0, candidate_2d::SearchParameters(), 0);
    best_candidate.score = min_score;
    int best_submap_index = 0;
    int precomputed_size = (int)_is_precomputed.size();
    m_stop_matching = false;
    //penghu* 24/3/23 遍历每张submap
    for(int i = 0; i < precomputed_size; ++i)
    {
        // LOG_INFO("ip_step_1: i: %d, precomputed_size: %d, _submap2d_protos size: %d", i, precomputed_size, _submap2d_protos.size());
        if(m_stop_matching)
        {
            // LOG_INFO("ip_step_1: stop match ");
            return false;
        }
        //生成整形与浮点数映射关系快速查找表
        // if (!_is_precomputed[i])
        // {
        //     ComputeSubmap2dProto(i);
        //     _is_precomputed[i] = true;
        // }
        
        //lyy: tranform initial pose from global to local
        transform::Rigid2d tmp_pose_estimate_precompute = *pose_estimate;
        tmp_pose_estimate_precompute = local_to_global_vec[i].inverse() * tmp_pose_estimate_precompute;

        const auto local_submap_pose2d = cartographer::transform::Project2D(
                    cartographer::transform::ToRigid3(_submap2d_protos[i].local_pose()));
        // const transform::Rigid2d trans_node_submap = local_submap_pose2d.inverse() * (*pose_estimate);
        const transform::Rigid2d trans_node_submap = local_submap_pose2d.inverse() * tmp_pose_estimate_precompute;//lyy
        double distance = trans_node_submap.translation().norm();
        distanceIndexPairs.push_back({distance, i});
    }

    std::sort(distanceIndexPairs.begin(), distanceIndexPairs.end(),
              [](const DistanceIndexPair& a, const DistanceIndexPair& b){
        return a.distance < b.distance;
    });

    //penghu* 24/3/23
    LOG_INFO("fast_csm distanceIndexPairs.size(): %d", static_cast<int>(distanceIndexPairs.size()));
    for(int i = 0; i < std::min(4, static_cast<int>(distanceIndexPairs.size())); ++i)
    {
        int index = distanceIndexPairs[i].index;//lyy

        //是否采用初始位姿，不采用时，取地图中心点
        //lyy: tranform initial pose from global to local
        tmp_pose_estimate = *pose_estimate;
        tmp_pose_estimate = local_to_global_vec[index].inverse() * tmp_pose_estimate;

        //lyy
        const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
            point_cloud,
            transform::Rigid3f::Rotation(Eigen::AngleAxisf(tmp_pose_estimate.rotation().cast<float>().angle(), Eigen::Vector3f::UnitZ())));

        //搜索窗口的大小，有初始位姿时，缩小搜索范围
        const double linear_search_window = use_initial_guess
                                        ? search_window
                                        : 1e6 * _resolution;
        const double angular_search_window = M_PI;//lyy
        //搜索窗口指定搜索角度为PI，最终搜索窗口会放大为2*PI
        const candidate_2d::SearchParameters initial_search_parameters(
                    linear_search_window, angular_search_window, rotated_point_cloud/*point_cloud*/, _resolution);//lyy: M_PI
        LOG(INFO) << "initial csm linear_search_window: " << linear_search_window << ", angular_search_window: " << angular_search_window
                << ", resolution: " << _resolution << ", angular resolution: " << 57.3*initial_search_parameters.angular_perturbation_step_size
                << " deg";
        //遍历旋转激光束
        const std::vector<sensor::PointCloud> rotated_scans =
                candidate_2d::GenerateRotatedScans(
                    rotated_point_cloud/*point_cloud*/, initial_search_parameters);

        const Eigen::Vector2d center = use_initial_guess
                    ? Eigen::Vector2d(tmp_pose_estimate.translation().x(),
                                    tmp_pose_estimate.translation().y())
                    : _limits[index].max() - 0.5 * _limits[index].resolution() *
                    Eigen::Vector2d(_limits[index].cell_limits().num_y_cells,
                                    _limits[index].cell_limits().num_x_cells);
        const double center_angle = use_initial_guess ? tmp_pose_estimate.rotation().angle() : 0.0;//lyy
        // LOG_INFO("ip_step_3: Use init guss: %d, center: %f, %f",
        //          use_initial_guess, center.x(), center.y());

        //将旋转后的激光束离散到栅格中
        const std::vector<candidate_2d::DiscreteScan2D> discrete_scans
                = candidate_2d::DiscretizeScans(
                    _limits[index], rotated_scans,
                    Eigen::Translation2f(center.x(), center.y()));
        candidate_2d::SearchParameters search_parameters = initial_search_parameters;

        //将窗口缩放到激光扫描视野内
        search_parameters.ShrinkToFit(discrete_scans, _limits[index].cell_limits());

        //根据分支定界深度，先提取低分辨率下打分高于阈值的candidate
        std::vector<candidate_2d::Candidate2D> lowest_resolution_candidates
                = GenerateLowestResolutionCandidates(
                    search_parameters, discrete_scans, min_score, index);
        //根据初步打分值,重新排列candidate，将分值高的排在前面
        SortCandidates(&lowest_resolution_candidates);
        //执行分支定界，进一步细化candidate的打分值
        const candidate_2d::Candidate2D tmp_best_candidate = BranchAndBound(
                    discrete_scans, search_parameters, lowest_resolution_candidates,
                    _branch_and_bound_depth - 1, best_candidate.score);
        LOG_INFO("i: %d, submap index: %d, tmp_best_candidate score: %f, Candidate_X_Y_th: (%f, %f, %f)",
                 i, index, tmp_best_candidate.score, tmp_best_candidate.x, tmp_best_candidate.y, 
                 SlamCommon::RadToDeg(cartographer::common::NormalizeAngleDifference(tmp_best_candidate.orientation)));
        if (tmp_best_candidate.score <= best_candidate.score)
        {
            continue;
        }

        //搜索到更高分值的candidate
        // LOG_INFO("ip_step_4: Candidate match best score update: %f >> %f,"
        //          " pose(%f, %f) >> (%f, %f), submap index: %d",
        //          best_candidate.score, tmp_best_candidate.score,
        //          initial_pose.x() + best_candidate.x,
        //          initial_pose.y() + best_candidate.y,
        //          center.x() + best_candidate.x,
        //          center.y() + best_candidate.y, i);
        LOG_INFO("ip_step_4: Candidate_X_Y: (%f, %f, %f), Candidate match best score update: %f >> %f,"
                 " pose(%f, %f, %f) >> (%f, %f, %f), i: %d, submap index: %d",
                 tmp_best_candidate.x, tmp_best_candidate.y, 57.3*cartographer::common::NormalizeAngleDifference(tmp_best_candidate.orientation),
                 best_candidate.score, tmp_best_candidate.score,
                 initial_pose.x() + best_candidate.x,
                 initial_pose.y() + best_candidate.y,
                 57.3*cartographer::common::NormalizeAngleDifference(initial_angle + best_candidate.orientation),
                 center.x() + tmp_best_candidate.x,
                 center.y() + tmp_best_candidate.y,
                 57.3*cartographer::common::NormalizeAngleDifference(center_angle + tmp_best_candidate.orientation), 
                 i, index);
        
        best_candidate = tmp_best_candidate;
        initial_pose = center;
        initial_angle = center_angle;//lyy
        best_submap_index = index;
        if (best_candidate.score > confidence_score)
        {
            LOG_INFO("Hooray, match score(%f) > threshold(%f)",
                     best_candidate.score, confidence_score);
            // If do this, the result maybe not the global optimum.
            if (SlamCommon::ToSeconds(SlamCommon::TimeNow() - start_time)
                    < min_search_time)
            {
                // If time still allowed when find a high confidence result,
                // continue searching.
                continue;
            }
            else
            {
                LOG_INFO("Time over, stop matching");
                distanceIndexPairs.clear();
                distanceIndexPairs.shrink_to_fit();
                _limits.clear();
                _limits.shrink_to_fit();
                _precomputation_grid_stacks.clear();
                _precomputation_grid_stacks.shrink_to_fit();
                for(int i=0; i<_is_precomputed.size(); i++){
                    _is_precomputed[i] = false;
                }
                break;
            }
        }
    }

    matched_submap_id = best_submap_index;//lyy
    // LOG_INFO("ip_step_5: best score: %f, min score: %f", best_candidate.score, min_score);

    if (best_candidate.score <= min_score)
    {
        distanceIndexPairs.clear();
        distanceIndexPairs.shrink_to_fit();
        _limits.clear();
        _limits.shrink_to_fit();
        _precomputation_grid_stacks.clear();
        _precomputation_grid_stacks.shrink_to_fit();
        for(int i=0; i<_is_precomputed.size(); i++){
            _is_precomputed[i] = false;
        }

        return false;
    }

    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d({initial_pose.x() + best_candidate.x,
                                         initial_pose.y() + best_candidate.y},
                                        //Eigen::Rotation2Dd::Identity() * //lyy
                                        tmp_pose_estimate.rotation()* //lyy
                                        Eigen::Rotation2Dd(best_candidate.orientation));
    SlamCommon::Time now = SlamCommon::TimeNow();
    // LOG_INFO("Scan match finished, pose extimate: %s, score: %f, cost time: %fs",
    //          pose_estimate->DebugString().c_str(), *score,
    //          SlamCommon::ToSeconds(now - start_time));
    start_time = now;
    if (_use_ceres_to_refine)
    {
        // LOG_INFO("Attemp to use ceres to refine matching...");
        ValueConversionTables conversion_tables;
        LOG_INFO("begin ceres:");
        _ceres_sm->Match(*pose_estimate, point_cloud,
                         mapping::ProbabilityGrid(
                             _submap2d_protos[best_submap_index].grid(),
                             &conversion_tables),
                         pose_estimate);
        now = SlamCommon::TimeNow();
        // LOG_INFO("Ceres scam match finish, pose extimate: %s, cost time: %fs",
        //          pose_estimate->DebugString().c_str(),
        //          SlamCommon::ToSeconds(now - start_time));
        LOG_INFO("Ceres scam match finish, pose extimate: %s, cost time: %f s",
            pose_estimate->DebugString().c_str(),
            SlamCommon::ToSeconds(now - start_time));
    }
    distanceIndexPairs.clear();
    distanceIndexPairs.shrink_to_fit();
    _limits.clear();
    _limits.shrink_to_fit();
    _precomputation_grid_stacks.clear();
    _precomputation_grid_stacks.shrink_to_fit();
    for(int i=0; i<_is_precomputed.size(); i++){
        _is_precomputed[i] = false;
    }

    return true;
}

std::vector<candidate_2d::Candidate2D> FasterCSM::GenerateLowestResolutionCandidates(
    const candidate_2d::SearchParameters& search_parameters,
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    const float min_score, 
    const int submap_id) const
{
    const int linear_step_size = 1 << (_branch_and_bound_depth - 1);
    int num_candidates = 0;
    //根据分支定界深度，将每个角度上的激光覆盖面栅格化，每个栅格是一个candidate，并累计candidate数量
    for (int scan_index = 0; scan_index != search_parameters.num_scans; ++scan_index)
    {
        const int num_lowest_resolution_linear_x_candidates =
                (search_parameters.linear_bounds[scan_index].max_x
                 - search_parameters.linear_bounds[scan_index].min_x
                 + linear_step_size) / linear_step_size;
        const int num_lowest_resolution_linear_y_candidates =
                (search_parameters.linear_bounds[scan_index].max_y
                 - search_parameters.linear_bounds[scan_index].min_y
                 + linear_step_size) / linear_step_size;
        num_candidates += num_lowest_resolution_linear_x_candidates *
                num_lowest_resolution_linear_y_candidates;
    }
    std::vector<candidate_2d::Candidate2D> candidates;
    candidates.reserve(num_candidates);
    //计算每个candidate的打分值，并提取打分值大于阈值的candidate
    for (int scan_index = 0; scan_index != search_parameters.num_scans; ++scan_index)
    {
        for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
             x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
             x_index_offset += linear_step_size)
        {
            for (int y_index_offset = search_parameters.linear_bounds[scan_index].min_y;
                 y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
                 y_index_offset += linear_step_size)
            {
                candidate_2d::Candidate2D candidate(
                            scan_index, x_index_offset, y_index_offset,
                            search_parameters, submap_id);
                // filter the lowest_resolution candidate by min_score
                CalCandidateScore(_branch_and_bound_depth - 1,
                                  discrete_scans, &candidate);
                //保存高于目前最低分值的所有栅格
                if (candidate.score > min_score)
                {
                    candidates.emplace_back(candidate);
                }
            }
        }
    }
    candidates.shrink_to_fit();
    return candidates;
}

void FasterCSM::CalCandidateScore(
    const int precomputation_grid_depth, 
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    candidate_2d::Candidate2D* const candidate) const
{
    const auto& submap_id = candidate->submap_id;
    const auto& precomputation_grid =
            _precomputation_grid_stacks[submap_id]->Get(
                precomputation_grid_depth);
    int sum = 0;
    //遍历该帧离散激光中的每个激光点
    for (const auto& xy_index : discrete_scans[candidate->scan_index])
    {
        const Eigen::Array2i proposed_xy_index(
                    xy_index.x() + candidate->x_index_offset,
                    xy_index.y() + candidate->y_index_offset);
        //返回值为0-255，代表在最大和最小分值之间的概率
        sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    //计算每个激光点的平均概率，并将该概率从[0, 255]转换到最大最小概率值之间
    candidate->score = precomputation_grid.ToScore(
                        sum / static_cast<float>(
                            discrete_scans[candidate->scan_index].size()));
}

// TODO: using thread_pool to accelerate ?
void FasterCSM::ScoreCandidates(
    const int precomputation_grid_depth,
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    std::vector<candidate_2d::Candidate2D>* const candidates) const
{
    for (candidate_2d::Candidate2D& candidate : *candidates)
    {
        CalCandidateScore(precomputation_grid_depth, discrete_scans, &candidate);
    }
    SortCandidates(candidates);
}

candidate_2d::Candidate2D FasterCSM::BranchAndBound(
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    const candidate_2d::SearchParameters& search_parameters,
    const std::vector<candidate_2d::Candidate2D>& candidates,
    const int candidate_depth,
    float min_score) const
{
    //分支定界达到搜索深度，返回排在最前面的分值最高的栅格位置
    if (candidate_depth == 0)
    {
        // Return the best candidate.
        return *candidates.begin();
    }

    candidate_2d::Candidate2D best_high_resolution_candidate(
                0, 0, 0, candidate_2d::SearchParameters(), 0);
    best_high_resolution_candidate.score = min_score;
    for (const candidate_2d::Candidate2D& candidate : candidates)
    {
        //如果后续候选位置的匹配度均小于目前得到的最大匹配度，则没必要继续搜索
        //这样做可以加快搜索速度，但是风险是第一层分支定界匹配结果可能会导致最终结果进入局部最优
        if (candidate.score <= min_score)
        {
            break;
        }
        std::vector<candidate_2d::Candidate2D> higher_resolution_candidates;
        const int half_width = 1 << (candidate_depth - 1);
        for (int x_offset : {0, half_width})
        {
            if (candidate.x_index_offset + x_offset >
                    search_parameters.linear_bounds[candidate.scan_index].max_x)
            {
                break;
            }
            for (int y_offset : {0, half_width})
            {
                if (candidate.y_index_offset + y_offset >
                    search_parameters.linear_bounds[candidate.scan_index].max_y)
                {
                    break;
                }
                higher_resolution_candidates.emplace_back(
                    candidate.scan_index, candidate.x_index_offset + x_offset,
                    candidate.y_index_offset + y_offset,
                    search_parameters, candidate.submap_id);
            }
        }
        ScoreCandidates(candidate_depth - 1, discrete_scans,
                        &higher_resolution_candidates);
        best_high_resolution_candidate = std::max(
                    best_high_resolution_candidate,
                    BranchAndBound(discrete_scans, search_parameters,
                                   higher_resolution_candidates, candidate_depth - 1,
                                   best_high_resolution_candidate.score));
    }
    return best_high_resolution_candidate;
}

void FasterCSM::ComputeSubmap2dProto(const int index)
{
    ValueConversionTables conversion_tables;
    const mapping::proto::Submap2D& submap_proto = _use_ceres_to_refine
                                                 ? _submap2d_protos[index]
                                                 : _submap2d_protos.front();
    Submap2D submap_2d(submap_proto, &conversion_tables);
    if (!_use_ceres_to_refine)
    {
        _submap2d_protos.pop_front();
    }

    CHECK_EQ(submap_2d.grid()->limits().resolution(), _resolution)
            << "The submaps' resolution must be same.";
    _limits.emplace_back(submap_2d.grid()->limits());
    _precomputation_grid_stacks.emplace_back(
                std::make_shared<PrecomputationStack>(
                    *(submap_2d.grid()), _branch_and_bound_depth));
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
