// Internal-only header for process-local fusion and degeneracy interfaces.
// Do NOT install or expose publicly. Include only within core library.

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_DEGENERACY_FLAG_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_DEGENERACY_FLAG_H_

#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Degeneracy flag (process-local)
void SetDegeneracyDetected(bool v);
bool GetDegeneracyDetected();

// Latest localization (process-local)
void SetLatestLocalization2D(const cartographer::transform::Rigid2d& pose, double score);
bool GetLatestLocalization2D(cartographer::transform::Rigid2d* pose, double* score);
bool GetLatestLocalizationXYT(double* x, double* y, double* theta, double* score);

// External odometry handover (process-local)
void SetExternalOdometryPose2D(double x, double y, double theta);
bool GetExternalOdometryPose2D(double* x, double* y, double* theta);
void SetUseExternalOdom(bool v);
bool GetUseExternalOdom();

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_DEGENERACY_FLAG_H_