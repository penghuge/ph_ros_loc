#ifndef FAST_SCAN_MATCHER_2D_LOCAL_H
#define FAST_SCAN_MATCHER_2D_LOCAL_H
#include "cartographer/carto_localization/scan_matching_2d/candidate_2d.h"
#include "cartographer/carto_localization/scan_matching_2d/precomputation.h"
#include "cartographer/carto_localization/scan_matching_2d/ceres_sm.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/proto/2d/local_trajectory_builder_options_2d.pb.h"

#include <memory>
#include <vector>
#include <deque>
#include <cartographer/common/port.h>
#include <cartographer/sensor/point_cloud.h>
#include <cartographer/mapping/2d/submap_2d.h>
#include <cartographer/common/format_transform.h>
//penghu 24/4/16
#include <cartographer/mapping/2d/probability_grid.h>
//penghu 24/5/29
#include "cartographer/common/config_file_operator.h"
#include "cartographer/common/file_interface.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
class fast_scan_matcher_2d_local
{
public:
    fast_scan_matcher_2d_local(std::deque<mapping::proto::Submap2D>* const submap2d_protos,
                               const int branch_and_bound_depth,
                               const bool use_ceres_to_refine,
                               const  cartographer::mapping::proto::LocalTrajectoryBuilderOptions2D& options,
                               SlamCommon::CFileInterface *file_interface,
                               SlamCommon::CConfigFileOperator *config_operator);
    ~fast_scan_matcher_2d_local() {};
    fast_scan_matcher_2d_local(const fast_scan_matcher_2d_local&) = delete;
    fast_scan_matcher_2d_local& operator=(const fast_scan_matcher_2d_local&) = delete;

    struct DistanceIndexPair {
        double distance;
        int index;
    };

    bool MatchInMap(const sensor::PointCloud& point_cloud,
                    const bool use_initial_guess,
                    const double search_window,
                    const double angle_window,
                    const bool on_scan,
                    const float min_score,
                    const float confidence_score,
                    float* const score,
                    transform::Rigid2d* const pose_estimate,
                    // int* const matched_submap_id,
                    int& matched_submap_id,//lyy
                    const double &v, const std::vector<cartographer::transform::Rigid2d>& local_to_global_vec);
    void StopMatching()
    {
        m_stop_matching = true;
    }

    void AddSubmapIdAndFlag(int submap_id, bool assigned_flag) {
        m_assigned_submap_id = submap_id;
        m_assigned_submap_id_flag = assigned_flag;
    }

    float GetBestCandidateScore(){
        return m_best_candidate_score;
    }

    int GetMatchedSubmapId(){
        return m_matched_submap_id;
    }

private:
    std::vector<candidate_2d::Candidate2D> GenerateLowestResolutionCandidates(
            const candidate_2d::SearchParameters& search_parameters,
            const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
            const float min_score,
            const int submap_id) const;
    //penghu*
    std::vector<candidate_2d::Candidate2D> GenerateExhaustiveSearchCandidates(
            const candidate_2d::SearchParameters& search_parameters,
            const int submap_id) const;
    //penghu 24/4/16
    void OneScoreCandidates(const Grid2D& grid,
                         const std::vector<candidate_2d::DiscreteScan2D>& discrete_scans,
                         std::vector<candidate_2d::Candidate2D>* candidates) const;
    //penghu 24/4/16
    float OneComputeCandidateScore(const ProbabilityGrid& probability_grid,
                                const candidate_2d::DiscreteScan2D& discrete_scan,
                                int x_index_offset, int y_index_offset) const;

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
    std::vector<DistanceIndexPair> distanceIndexPairs;
    const int _branch_and_bound_depth;
    std::vector<MapLimits> _limits;
    std::deque<mapping::proto::Submap2D> _submap2d_protos;
    std::vector<bool> _is_precomputed;
    std::vector<std::shared_ptr<PrecomputationStack> > _precomputation_grid_stacks;
    std::vector<std::unique_ptr<cartographer::mapping::ProbabilityGrid>> _precomputed_probability_grids;
    const double _resolution;
    const bool _use_ceres_to_refine;
    std::unique_ptr<CeresSM> _ceres_sm;
    bool m_stop_matching = false;
    int m_assigned_submap_id = 0;//lyy
    bool m_assigned_submap_id_flag = false;//lyy
    const cartographer::mapping::proto::LocalTrajectoryBuilderOptions2D options_;
    scan_matching::CeresScanMatcher2D ceres_scan_matcher_;
    //penghu 24/5/29
    SlamCommon::CFileInterface *file_interface_ = nullptr;
    SlamCommon::CConfigFileOperator *config_operator_ = nullptr;
    //penghu 24/5/29
    bool yaw_con_ = false;
    int model_ = 0;
    bool precison_ = false;
    int count_ = 0;
    double pre_ = 0.;
    int num_submap_ = 0;
    float m_best_candidate_score = 0.0f;
    int m_matched_submap_id = -1;

};


}

}

}


#endif // FAST_SCAN_MATCHER_2D_LOCAL_H
