#include "fast_scan_matcher_2d_local.h"

#include <absl/memory/memory.h>
#include <cartographer/common/math.h>
#include <cartographer/mapping/2d/grid_2d.h>
#include <cartographer/sensor/point_cloud.h>
#include <cartographer/transform/transform.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {
void SortCandidates(std::vector<candidate_2d::Candidate2D>* const candidates) {
  std::sort(candidates->begin(), candidates->end(),
            std::greater<candidate_2d::Candidate2D>());
}
}  // namespace

fast_scan_matcher_2d_local::fast_scan_matcher_2d_local(
    std::deque<mapping::proto::Submap2D>* const submap2d_protos,
    const int branch_and_bound_depth, const bool use_ceres_to_refine,
    const cartographer::mapping::proto::LocalTrajectoryBuilderOptions2D&
        options,
    SlamCommon::CFileInterface* file_interface,
    SlamCommon::CConfigFileOperator* config_operator)
    : _branch_and_bound_depth(branch_and_bound_depth),
      _is_precomputed(submap2d_protos->size(), false),
      _resolution(submap2d_protos->front().grid().limits().resolution()),
      _use_ceres_to_refine(use_ceres_to_refine),
      options_(options),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      // penghu 24/5/29
      file_interface_(file_interface),
      config_operator_(config_operator)

{
  std::swap(*submap2d_protos, _submap2d_protos);
  std::vector<double> vec;
  if(!file_interface_->GetFastScanMatcher2dLocalOptions(vec)){
    LOG_ERROR("GetFastScanMatcher2dLocalOptions failed!");
  }else{
    LOG_INFO("GetFastScanMatcher2dLocalOptions success!");
  }

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
  
  if (vec.size() == 3){
    yaw_con_= vec[0];
    precison_ = vec[1];
    num_submap_ = vec[2];
    LOG_INFO("yaw_con_: %d, precison_: %d, num_submap_: %d", yaw_con_, precison_, num_submap_);
  }

  if (!file_interface_->GetPrecomputedProbabilityGrids(_precomputed_probability_grids)) {
      LOG_ERROR("GetPrecomputedProbabilityGrids failed!");
  } else {
      LOG_INFO("GetPrecomputedProbabilityGrids success!");
  }
}

bool fast_scan_matcher_2d_local::MatchInMap(
    const sensor::PointCloud& point_cloud, const bool use_initial_guess,
    const double search_window, const double angle_window, const bool on_scan,
    const float min_score, const float confidence_score, float* const score,
    transform::Rigid2d* const pose_estimate,
    int& matched_submap_id,
    const double& v,
    const std::vector<cartographer::transform::Rigid2d>& local_to_global_vec) {
  CHECK(score && pose_estimate);
  SlamCommon::Time start_time = SlamCommon::TimeNow();
  LOG_INFO_THROTTLE(1.0, "steep_426_1_run1_pose_estimate(%f, %f, %f), search(%f, %f), v=%f", pose_estimate->translation().x(),
                      pose_estimate->translation().y(),
                      SlamCommon::RadToDeg(pose_estimate->rotation().angle()),
                      search_window, angle_window, v);

  Eigen::Vector2d initial_pose = {0.0, 0.0};
  transform::Rigid2d tmp_pose_estimate = *pose_estimate;
  double initial_angle = 0.0;
  candidate_2d::Candidate2D best_candidate(0, 0, 0, candidate_2d::SearchParameters(), 0);
  best_candidate.score = min_score;
  int best_submap_index = 0;
  int precomputed_size = (int)_is_precomputed.size();
  m_stop_matching = false;
  // penghu*
  for (int i = 0; i < precomputed_size; ++i) {
      if (m_stop_matching) {
          return false;
      }

      tmp_pose_estimate = *pose_estimate;
      tmp_pose_estimate = local_to_global_vec[i].inverse() * tmp_pose_estimate;

      const auto local_submap_pose2d =
          cartographer::transform::Project2D(cartographer::transform::ToRigid3(_submap2d_protos[i].local_pose()));

      const transform::Rigid2d trans_node_submap = local_submap_pose2d.inverse() * tmp_pose_estimate;
      double distance = trans_node_submap.translation().norm();
      distanceIndexPairs.push_back({distance, i});
  }

  std::sort(distanceIndexPairs.begin(), distanceIndexPairs.end(),
            [](const DistanceIndexPair& a, const DistanceIndexPair& b) {
              return a.distance < b.distance;
            });

  SlamCommon::Time start_time1 = SlamCommon::TimeNow();
  std::string candidate_submap_list = "";
  for (int i = 0; i < num_submap_; i++) {
    candidate_submap_list += std::to_string(distanceIndexPairs[i].index);
    candidate_submap_list += " ";
  }
  candidate_submap_list = "candidate_submap_list: " + candidate_submap_list;
  LOG_ERROR_THROTTLE(1.0,
                     "LOCAL_CARTO_num_submap_: %d, distanceIndexPairs.size: %d, on_scan: %d, pose_estimate(%f, %f, "
                     "%f), search(%f, %f), v= %f,  %s",
                     num_submap_, distanceIndexPairs.size(), on_scan, pose_estimate->translation().x(),
                     pose_estimate->translation().y(), SlamCommon::RadToDeg(pose_estimate->rotation().angle()),
                     search_window, angle_window, v,
                     candidate_submap_list.c_str());

  for (int i = 0; i < std::min(num_submap_, static_cast<int>(distanceIndexPairs.size())); ++i) {
    // lyy: if assigned submap_id, just scanmatch this submap
    if (m_assigned_submap_id_flag && i > 0) {
      break;
    }

    int index = distanceIndexPairs[i].index;

    // lyy
    if (m_assigned_submap_id_flag) {
      index = m_assigned_submap_id;
    }

    tmp_pose_estimate = *pose_estimate;
    tmp_pose_estimate = local_to_global_vec[index].inverse() * tmp_pose_estimate;

    const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                            tmp_pose_estimate.rotation().cast<float>().angle(),
                            Eigen::Vector3f::UnitZ())));
    const double linear_search_window =
        use_initial_guess ? search_window : 1e6 * _resolution;

    const candidate_2d::SearchParameters initial_search_parameters(
        linear_search_window, angle_window, rotated_point_cloud, _resolution);

    const std::vector<sensor::PointCloud> rotated_scans =
        candidate_2d::GenerateRotatedScans(rotated_point_cloud, initial_search_parameters);

    const Eigen::Vector2d center =
        use_initial_guess
            ? Eigen::Vector2d(tmp_pose_estimate.translation().x(),
            tmp_pose_estimate.translation().y())
            : _limits[index].max() -
                  0.5 * _limits[index].resolution() *
                      Eigen::Vector2d(_limits[index].cell_limits().num_y_cells,
                                      _limits[index].cell_limits().num_x_cells);
    const double center_angle = use_initial_guess ? tmp_pose_estimate.rotation().angle() : 0.0;
    LOG_ERROR_THROTTLE(
        1.0,
        "LOCAL_CARTO_steep_426_2_run2_use init guss: %d, center: %f, %f, submap_index(%d), submap_id(%d), "
        "node_to_submap_distance(%f), index = %d, i = %d, m_assigned_submap_id_flag: %d, m_assigned_submap_id: %d",
        use_initial_guess, center.x(), center.y(), i, index, distanceIndexPairs[i].distance, index, i,
        m_assigned_submap_id_flag, m_assigned_submap_id);

    const std::vector<candidate_2d::DiscreteScan2D> discrete_scans =
        candidate_2d::DiscretizeScans(_limits[index], rotated_scans, Eigen::Translation2f(center.x(), center.y()));

    candidate_2d::SearchParameters search_parameters = initial_search_parameters;

    // penghu 24/4/16
    if (on_scan) {
      std::vector<candidate_2d::Candidate2D> candidates = GenerateExhaustiveSearchCandidates(search_parameters, index);
      OneScoreCandidates(*_precomputed_probability_grids[index], discrete_scans, &candidates);

      const candidate_2d::Candidate2D& one_best_candidate = *std::max_element(candidates.begin(), candidates.end());

      if (one_best_candidate.score <= best_candidate.score) {
        LOG_DEBUG("one_best_candidate.score(%f) <= min.score(%f)", one_best_candidate.score, best_candidate.score);
        continue;
      }

      best_candidate = one_best_candidate;
      best_submap_index = index;
    } else {
      // 根据分支定界深度，先提取低分辨率下打分高于阈值的candidate
      std::vector<candidate_2d::Candidate2D> lowest_resolution_candidates =
          GenerateLowestResolutionCandidates(search_parameters, discrete_scans,
                                             min_score, index);

      SortCandidates(&lowest_resolution_candidates);
      const candidate_2d::Candidate2D tmp_best_candidate = BranchAndBound(
          discrete_scans, search_parameters, lowest_resolution_candidates,
          _branch_and_bound_depth - 1, best_candidate.score);

      if (tmp_best_candidate.score <= best_candidate.score) {
        continue;
      }

      best_candidate = tmp_best_candidate;
      best_submap_index = index;
    }

    initial_pose = center;
    initial_angle = center_angle;

    // if (best_candidate.score > 0.4) break;
  }
  m_best_candidate_score = best_candidate.score;
  m_matched_submap_id = best_submap_index;
  LOG_DEBUG_THROTTLE(1.0, "steep_426_6_Candidate_X_Y(%f, %f, %f), before_optimize_pose(%f, %f, %f),after_optimize_pose(%f, %f, %f), best_submap_id = %d, best score: %f, min score: %f",
              best_candidate.x, best_candidate.y, 57.3*cartographer::common::NormalizeAngleDifference(best_candidate.orientation),
              initial_pose.x(), initial_pose.y(), 57.3*cartographer::common::NormalizeAngleDifference(initial_angle),
              initial_pose.x() + best_candidate.x, initial_pose.y() + best_candidate.y,
              57.3*cartographer::common::NormalizeAngleDifference(initial_angle + best_candidate.orientation),
              best_submap_index, best_candidate.score, min_score);
  matched_submap_id = best_submap_index;

  if(m_best_candidate_score > 0.5 && best_candidate.x == 0 && best_candidate.y == 0)
  {
    singlemap_flag_ = true;
  }else{
    singlemap_flag_ = false;
  }

  if (precison_) {
    *score = best_candidate.score;
    auto pose_observation = absl::make_unique<transform::Rigid2d>();
    ceres::Solver::Summary summary;
    if (best_candidate.score > 0.5) {
      ceres_scan_matcher_.Match(
          pose_estimate->translation(), *pose_estimate, point_cloud,
          *_precomputed_probability_grids[best_submap_index],
          pose_observation.get(), &summary, 2);
      *pose_estimate = std::move(*pose_observation);
    } else {
      if (best_candidate.score > pre_ && (best_candidate.x != 0 || best_candidate.y != 0)) {
        if (yaw_con_) {
          *pose_estimate =
              transform::Rigid2d({initial_pose.x() + best_candidate.x,
                                  initial_pose.y() + best_candidate.y},
                                 Eigen::Rotation2Dd(initial_angle));
        } else {
          *pose_estimate = transform::Rigid2d(
              {initial_pose.x() + best_candidate.x,
               initial_pose.y() + best_candidate.y},
              Eigen::Rotation2Dd(initial_angle) *
                  Eigen::Rotation2Dd(best_candidate.orientation));
        }
        ceres_scan_matcher_.Match(
            pose_estimate->translation(), *pose_estimate, point_cloud,
            *_precomputed_probability_grids[best_submap_index],
            pose_observation.get(), &summary, 3);
        pre_ = best_candidate.score;
        *pose_estimate = std::move(*pose_observation);
        distanceIndexPairs.clear();
        return true;
      } else {
        pre_ = best_candidate.score;
        distanceIndexPairs.clear();
        return false;
      }
    }
    pre_ = best_candidate.score;
    distanceIndexPairs.clear();
    return true;
  } else {
    if (best_candidate.score <= confidence_score || file_interface_->Get_NaviType() /* || file_interface_->GetSign()*/) {
      distanceIndexPairs.clear();
      matched_submap_id = -1;
      if(file_interface_->Get_NaviType()){
        best_candidate.score = 0.666;
      }
      
      LOG_WARNING_THROTTLE(1.0, "best_candidate.score: %f <= %f, matched_submap_id: %d, Get_NaviType(): %d",
                        best_candidate.score, confidence_score, matched_submap_id, file_interface_->Get_NaviType());
      return false;
    }

    *score = best_candidate.score;
    if (yaw_con_) {
      *pose_estimate = transform::Rigid2d({initial_pose.x() + best_candidate.x,
                                           initial_pose.y() + best_candidate.y},
                                          Eigen::Rotation2Dd(initial_angle));
    } else {
      *pose_estimate = transform::Rigid2d(
          {initial_pose.x() + best_candidate.x,
           initial_pose.y() + best_candidate.y},
          Eigen::Rotation2Dd(initial_angle) *
              Eigen::Rotation2Dd(best_candidate.orientation));
    }
    SlamCommon::Time now = SlamCommon::TimeNow();
    start_time = now;
    auto pose_observation = absl::make_unique<transform::Rigid2d>();
    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(
        pose_estimate->translation(), *pose_estimate, point_cloud,
        *_precomputed_probability_grids[best_submap_index],
        pose_observation.get(), &summary, v);
    *pose_estimate = std::move(*pose_observation);
    now = SlamCommon::TimeNow();
    LOG_DEBUG_THROTTLE(1.0, "steep_426_9_Ceres scam match finish, pose extimate: %s, cost time: %fs, csm+ceres_cost: %fs", pose_estimate->DebugString().c_str(),
                SlamCommon::ToSeconds(now - start_time), SlamCommon::ToSeconds(SlamCommon::TimeNow() - start_time1));

    distanceIndexPairs.clear();
    return true;
  }
}

std::vector<candidate_2d::Candidate2D>
fast_scan_matcher_2d_local::GenerateLowestResolutionCandidates(
    const candidate_2d::SearchParameters& search_parameters,
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    const float min_score, const int submap_id) const {
  const int linear_step_size = 1 << (_branch_and_bound_depth - 1);
  int num_candidates = 0;
  // 根据分支定界深度，将每个角度上的激光覆盖面栅格化，每个栅格是一个candidate，并累计candidate数量
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<candidate_2d::Candidate2D> candidates;
  candidates.reserve(num_candidates);
  // LOG_INFO("candidates.SIZE = %d", num_candidates);
  // 计算每个candidate的打分值，并提取打分值大于阈值的candidate
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidate_2d::Candidate2D candidate(scan_index, x_index_offset,
                                            y_index_offset, search_parameters,
                                            submap_id);
        // filter the lowest_resolution candidate by min_score
        CalCandidateScore(_branch_and_bound_depth - 1, discrete_scans,
                          &candidate);
        // 保存高于目前最低分值的所有栅格
        if (candidate.score > min_score) {
          candidates.emplace_back(candidate);
        }
      }
    }
  }
  candidates.shrink_to_fit();
  return candidates;
}

// penghu*
std::vector<candidate_2d::Candidate2D>
fast_scan_matcher_2d_local::GenerateExhaustiveSearchCandidates(
    const candidate_2d::SearchParameters& search_parameters,
    const int submap_id) const {
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<candidate_2d::Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters, submap_id);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void fast_scan_matcher_2d_local::CalCandidateScore(
    const int precomputation_grid_depth,
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    candidate_2d::Candidate2D* const candidate) const {
  const auto& submap_id = candidate->submap_id;
  const auto& precomputation_grid =
      _precomputation_grid_stacks[submap_id]->Get(precomputation_grid_depth);
  int sum = 0;
  // 遍历该帧离散激光中的每个激光点
  for (const auto& xy_index : discrete_scans[candidate->scan_index]) {
    const Eigen::Array2i proposed_xy_index(
        xy_index.x() + candidate->x_index_offset,
        xy_index.y() + candidate->y_index_offset);
    // 返回值为0-255，代表在最大和最小分值之间的概率
    sum += precomputation_grid.GetValue(proposed_xy_index);
  }
  // 计算每个激光点的平均概率，并将该概率从[0, 255]转换到最大最小概率值之间
  candidate->score = precomputation_grid.ToScore(
      sum / static_cast<float>(discrete_scans[candidate->scan_index].size()));
  //    LOG_INFO("candidate_score= %f, candidate_xy(%f, %f),
  //    candidate_offset_xy(%f, %f)", candidate->score,
  //             candidate->x, candidate->y,
  //             candidate->x_index_offset, candidate->y_index_offset);
}

// TODO: using thread_pool to accelerate ?
void fast_scan_matcher_2d_local::ScoreCandidates(
    const int precomputation_grid_depth,
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    std::vector<candidate_2d::Candidate2D>* const candidates) const {
  //    LOG_INFO("candidates.SIZE = %d", candidates->size());
  for (candidate_2d::Candidate2D& candidate : *candidates) {
    //        LOG_INFO("candidates_submap_id = %d", candidate.submap_id);
    CalCandidateScore(precomputation_grid_depth, discrete_scans, &candidate);
  }
  SortCandidates(candidates);
}

candidate_2d::Candidate2D fast_scan_matcher_2d_local::BranchAndBound(
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    const candidate_2d::SearchParameters& search_parameters,
    const std::vector<candidate_2d::Candidate2D>& candidates,
    const int candidate_depth, float min_score) const {
  // 分支定界达到搜索深度，返回排在最前面的分值最高的栅格位置
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  candidate_2d::Candidate2D best_high_resolution_candidate(
      0, 0, 0, candidate_2d::SearchParameters(), 0);
  best_high_resolution_candidate.score = min_score;
  for (const candidate_2d::Candidate2D& candidate : candidates) {
    // 如果后续候选位置的匹配度均小于目前得到的最大匹配度，则没必要继续搜索
    // 这样做可以加快搜索速度，但是风险是第一层分支定界匹配结果可能会导致最终结果进入局部最优
    if (candidate.score <= min_score) {
      break;
    }
    std::vector<candidate_2d::Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters,
            candidate.submap_id);
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

void fast_scan_matcher_2d_local::ComputeSubmap2dProto(const int index) {
  ValueConversionTables conversion_tables;
  const mapping::proto::Submap2D& submap_proto =
      _use_ceres_to_refine ? _submap2d_protos[index] : _submap2d_protos.front();
  Submap2D submap_2d(submap_proto, &conversion_tables);
  if (!_use_ceres_to_refine) {
    _submap2d_protos.pop_front();
  }

  CHECK_EQ(submap_2d.grid()->limits().resolution(), _resolution)
      << "The submaps' resolution must be same.";
  _limits.emplace_back(submap_2d.grid()->limits());
  _precomputation_grid_stacks.emplace_back(
      std::make_shared<PrecomputationStack>(*(submap_2d.grid()),
                                             _branch_and_bound_depth));
}

// penghu 24/4/16
void fast_scan_matcher_2d_local::OneScoreCandidates(
    const Grid2D& grid,
    const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
    std::vector<candidate_2d::Candidate2D>* candidates) const {
  for (candidate_2d::Candidate2D& candidate : *candidates) {
    candidate.score = OneComputeCandidateScore(
        static_cast<const ProbabilityGrid&>(grid),
        discrete_scans[candidate.scan_index], candidate.x_index_offset,
        candidate.y_index_offset);
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) * (1e-1) +
                               std::abs(candidate.orientation) * (1e-1)));
  }
}

// penghu 24/4/16
float fast_scan_matcher_2d_local::OneComputeCandidateScore(
    const ProbabilityGrid& probability_grid,
    const candidate_2d::DiscreteScan2D& discrete_scan, int x_index_offset,
    int y_index_offset) const {
  float candidate_score = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    candidate_score += probability;
  }
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
