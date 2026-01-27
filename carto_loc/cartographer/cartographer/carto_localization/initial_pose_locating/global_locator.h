#ifndef GLOBAL_LOCATER_H_
#define GLOBAL_LOCATER_H_

#include "cartographer/mapping_bridge/node_options.h"
#include "cartographer/mapping_bridge/trajectory_options.h"
#include "cartographer/common/slam_math.h"

#include <Eigen/Eigen>
#include <cartographer/sensor/point_cloud.h>
#include "cartographer/common/file_interface.h"

namespace Localization {
namespace CartoLocalization {
class GlobalLocator
{
public:
    struct Pose
    {
        cartographer::transform::Rigid3d rigid;
        float prob = 0.0;
        bool has_intial_guess = false;
        double search_window = 5.;
        double min_search_time = 0.;

//        std::string DebugString() const {
//        return absl::Substitute(
//           "{ xyz: [$0, $1, $2], q: [$3, $4, $5, $6], prob: [$7] }",
//           rigid.translation().x(), rigid.translation().y(),
//           rigid.translation().z(), rigid.rotation().w(),
//           rigid.rotation().x(), rigid.rotation().y(), rigid.rotation().z(),
//           prob);
//        }
        std::string DebugString() const
        {
            return absl::Substitute(
                        "$0 $1 $2 $3 $4 $5 $6",
                        rigid.translation().x(), rigid.translation().y(),
                        rigid.translation().z(), rigid.rotation().x(),
                        rigid.rotation().y(), rigid.rotation().z(), rigid.rotation().w());
        }
    };

    GlobalLocator() {};
    virtual ~GlobalLocator() {};
    GlobalLocator(const GlobalLocator&) = delete;
    GlobalLocator& operator=(const GlobalLocator&) = delete;

    virtual bool Init(const CartoMapppingBridge::NodeOptions &node_options,
                      const CartoMapppingBridge::TrajectoryOptions &traj_options,
                      SlamCommon::CFileInterface *file_interface,
                      const float min_score_thresh,
                      const float confidence_score_thresh,
                      const std::string& pbstream_filepath,
                      const bool use_ceres_to_refine) = 0;
    virtual bool Match(const cartographer::sensor::PointCloud& point_cloud,
                     const cartographer::transform::Rigid3d& trans,
                     Pose* const res) const = 0;
    virtual void StopMatching() = 0;
    virtual Pose GetRelativePoseOfLastNode(const Pose &pose) const = 0;

protected:
    float m_min_score_thresh, m_confidence_score_thresh;
};

struct TestResult
{
    GlobalLocator::Pose pose;
    SlamCommon::SlamLaserScanData laser_data;
    double use_time = 0.;
    bool flag = false;
};

} // namespace CartoLocalization
} // namespace Localization

#endif // GLOBAL_LOCATER_H_
