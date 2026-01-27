#ifndef GLOBAL_SCAN_MATCHER_LOCAL_H
#define GLOBAL_SCAN_MATCHER_LOCAL_H

#include "cartographer/mapping_bridge/node_options.h"
#include "cartographer/mapping_bridge/trajectory_options.h"
#include "cartographer/common/slam_math.h"

#include <Eigen/Eigen>
#include <cartographer/sensor/point_cloud.h>
//penghu 24/5/29
#include "cartographer/common/config_file_operator.h"
#include "cartographer/common/file_interface.h"

namespace cartographer {
namespace mapping {

class GlobalLocal
{
public:
    struct Pose
    {
        cartographer::transform::Rigid3d rigid;
        float prob = 0.0;
        bool has_intial_guess = false;
        double search_window = 5.;
        double angle_window = M_PI;
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

    GlobalLocal() {};
    virtual ~GlobalLocal() {};
    GlobalLocal(const GlobalLocal&) = delete;
    GlobalLocal& operator=(const GlobalLocal&) = delete;

    virtual bool Init(const proto::LocalTrajectoryBuilderOptions2D& options,
                      const float min_score_thresh,
                      const float confidence_score_thresh,
                      const std::string& pbstream_filepath,
                      const bool use_ceres_to_refine,
                      const bool use_one_scan,
                      SlamCommon::CFileInterface *file_interface,
                      SlamCommon::CConfigFileOperator *config_operator) = 0;
    virtual bool Match(const transform::Rigid3d& gravity_alignment,
                       const cartographer::sensor::PointCloud& point_cloud,
                       Pose* const res, const double &vel, cartographer::transform::Rigid3d& local_to_global) const = 0;//lyy
    virtual void StopMatching() = 0;
    virtual Pose GetRelativePoseOfLastNode(const Pose &pose) const = 0;
    virtual void AddSubmapIdAndFlag(int submap_id, bool assigned_flag) = 0;//lyy

protected:
    float m_min_score_thresh, m_confidence_score_thresh;
    //penghu 24/5/15
    bool is_one_scan_ = false;
};

}

}

#endif // GLOBAL_SCAN_MATCHER_LOCAL_H
