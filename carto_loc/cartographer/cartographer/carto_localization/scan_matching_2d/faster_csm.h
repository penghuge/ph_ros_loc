#ifndef _SCAN_MATCHING_FASTER_CSM_H_
#define _SCAN_MATCHING_FASTER_CSM_H_

#include "candidate_2d.h"
#include "precomputation.h"
#include "ceres_sm.h"

#include <memory>
#include <vector>
#include <deque>
#include <cartographer/common/port.h>
#include <cartographer/sensor/point_cloud.h>
#include <cartographer/mapping/2d/submap_2d.h>
#include "cartographer/common/file_interface.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

class FasterCSM
{
public:
    FasterCSM(std::deque<mapping::proto::Submap2D>* const submap2d_protos,
            const int branch_and_bound_depth,
            const bool use_ceres_to_refine,
            SlamCommon::CFileInterface *file_interface);
    ~FasterCSM() {};
    FasterCSM(const FasterCSM&) = delete;
    FasterCSM& operator=(const FasterCSM&) = delete;
    //penghu* 24/3/23
    struct DistanceIndexPair {
        double distance;
        int index;
    };

    bool MatchInMap(const sensor::PointCloud& point_cloud,
                    const bool use_initial_guess,
                    const double search_window,
                    const double min_search_time,
                    const float min_score,
                    const float confidence_score,
                    float* const score,
                    transform::Rigid2d* const pose_estimate,
                    int& matched_submap_id/*lyy*/,
                    const std::vector<cartographer::transform::Rigid2d>& local_to_global_vec/*lyy*/);
    void StopMatching()
    {
        m_stop_matching = true;
    }

private:
    std::vector<candidate_2d::Candidate2D> GenerateLowestResolutionCandidates(
      const candidate_2d::SearchParameters& search_parameters,
      const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
      const float min_score,
      const int submap_id) const;

    void CalCandidateScore(
      const int precomputation_grid_depth,
      const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
      candidate_2d::Candidate2D* const candidate) const;

    void ScoreCandidates(
      const int precomputation_grid_depth,
      const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
      std::vector<candidate_2d::Candidate2D>* const candidates) const;

    candidate_2d::Candidate2D BranchAndBound(
      const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
      const candidate_2d::SearchParameters& search_parameters,
      const std::vector<candidate_2d::Candidate2D>& candidates,
      int candidate_depth, float min_score) const;

    void ComputeSubmap2dProto(const int index);

private:
    const int _branch_and_bound_depth;
    std::vector<MapLimits> _limits;
    std::deque<mapping::proto::Submap2D> _submap2d_protos;
    std::vector<bool> _is_precomputed;
    std::vector<std::shared_ptr<PrecomputationStack> > _precomputation_grid_stacks;
    const double _resolution;
    const bool _use_ceres_to_refine;
    std::unique_ptr<CeresSM> _ceres_sm;
    bool m_stop_matching = false;
    //penghu* 24/3/23
    std::vector<DistanceIndexPair> distanceIndexPairs;
    SlamCommon::CFileInterface *file_interface_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // _SCAN_MATCHING_FASTER_CSM_H_
