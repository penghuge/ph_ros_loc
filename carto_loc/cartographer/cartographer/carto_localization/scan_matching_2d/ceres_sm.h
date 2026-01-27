#ifndef _SCAN_MATCHING_CERES_SM_H_
#define _SCAN_MATCHING_CERES_SM_H_

#include <Eigen/Core>
#include <cartographer/mapping/2d/grid_2d.h>
#include <cartographer/transform/transform.h>
#include <cartographer/sensor/point_cloud.h>
#include <ceres/ceres.h>

#include "cartographer/common/slam_math.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

class CeresSM
{
public:
    CeresSM();
    ~CeresSM();
    CeresSM(const CeresSM&) = delete;
    CeresSM& operator=(const CeresSM&) = delete;

    void Match(const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate) const;
    private:
    ceres::Solver::Options _ceres_solver_options;
    double _occupied_space_weight, _translation_weight, _rotation_weight;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // _SCAN_MATCHING_CERES_SM_H_
