#ifndef _SCAN_MATCHING_CSM_CANDIDATE_2D_H_
#define _SCAN_MATCHING_CSM_CANDIDATE_2D_H_

#include <vector>
#include <Eigen/Core>
#include <cartographer/common/lua_parameter_dictionary.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer/mapping/2d/xy_index.h>
#include <cartographer/sensor/point_cloud.h>

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace candidate_2d {

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

struct SearchParameters
{
    // Linear search window in pixel offsets; bounds are inclusive.
    struct LinearBounds
    {
        int min_x;
        int max_x;
        int min_y;
        int max_y;
    };

    SearchParameters() {}
    SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud& point_cloud, double resolution);

    void ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                   const CellLimits& cell_limits);

    int num_angular_perturbations;
    double angular_perturbation_step_size;
    double resolution;
    int num_scans;
    std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// trans to grid
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

struct Candidate2D
{
    //penghu* tianjia chushihualiebiao
    Candidate2D(const int init_scan_index,
              const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters,
              const int para_submap_id);
    // The submap_id this candidate belongs to.
    int submap_id = 0;
    // Index into the rotated scans vector.
    int scan_index = 0;
    // Linear offset from the initial pose.
    int x_index_offset = 0;
    int y_index_offset = 0;
    // Pose of this Candidate2D relative to the initial pose.
    double x = 0.;
    double y = 0.;
    double orientation = 0.;
    // Score, higher is better.
    float score = 0.f;
    // For compare
    bool operator<(const Candidate2D& other) const { return score < other.score; }
    bool operator>(const Candidate2D& other) const { return score > other.score; }
};

} // namespace candidate_2d
} // namespace scan_matching
} // namespace mapping
} // namespace cartographer

#endif  // _SCAN_MATCHING_CSM_CANDIDATE_2D_H_
