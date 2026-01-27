#ifndef _SCAN_MATCHING_PRECOMPUTATION_H_
#define _SCAN_MATCHING_PRECOMPUTATION_H_

#include <Eigen/Core>
#include <cartographer/mapping/2d/grid_2d.h>
#include <cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h>
#include "cartographer/logging/log.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
// A precomputed grid that contains in each cell (x0, y0) the maximum
// probability in the width x width area defined by x0 <= x < x0 + width and
// y0 <= y < y0.
class Precomputation
{
public:
    Precomputation(const Grid2D& grid, const CellLimits& limits, int width,
                 std::vector<float>* reusable_intermediate_grid);
    ~Precomputation() {}

    // Returns a value between 0 and 255 to represent probabilities between
    // min_score and max_score.
    int GetValue(const Eigen::Array2i& xy_index) const
    {
        const Eigen::Array2i local_xy_index = xy_index - offset_;
        // The static_cast<unsigned> is for performance to check with 2 comparisons
        // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
        // local_xy_index.x() >= wide_limits_.num_x_cells ||
        // local_xy_index.y() >= wide_limits_.num_y_cells
        // instead of using 4 comparisons.
        if (static_cast<unsigned>(local_xy_index.x()) >=
                static_cast<unsigned>(wide_limits_.num_x_cells) ||
            static_cast<unsigned>(local_xy_index.y()) >=
                static_cast<unsigned>(wide_limits_.num_y_cells))
        {
            return 0;
        }
        const int stride = wide_limits_.num_x_cells;
        return cells_[local_xy_index.x() + local_xy_index.y() * stride];
    }

    // Maps values from [0, 255] to [min_score, max_score].
    float ToScore(float value) const
    {
        return min_score_ + value * ((max_score_ - min_score_) / 255.f);
    }

private:
    uint8 ComputeCellValue(float probability) const;

    // Offset of the precomputation grid in relation to the 'grid'
    // including the additional 'width' - 1 cells.
    const Eigen::Array2i offset_;
    // Size of the precomputation grid.
    const CellLimits wide_limits_;
    const float min_score_;
    const float max_score_;
    // Probabilites mapped to 0 to 255.
    std::vector<uint8> cells_;
};

class PrecomputationStack
{
public:
    PrecomputationStack(const Grid2D& grid, const int branch_and_bound_depth);
    ~PrecomputationStack() {}

    const Precomputation& Get(int index)
    {
        return precomputation_grids_[index];
    }

private:
    std::vector<Precomputation> precomputation_grids_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif // _SCAN_MATCHING_PRECOMPUTATION_H_
