#ifndef GLOBAL_SCAN_MATCHER_2D_LOCAL_H
#define GLOBAL_SCAN_MATCHER_2D_LOCAL_H

#include "fast_scan_matcher_2d_local.h"
#include "global_scan_matcher_local.h"
#include "cartographer/common/file_interface.h"

namespace cartographer {
namespace mapping {

class global_scan_matcher_2d_local : public GlobalLocal {
public:
    global_scan_matcher_2d_local();
    ~global_scan_matcher_2d_local() override {};
    global_scan_matcher_2d_local(const global_scan_matcher_2d_local&) = delete;
    global_scan_matcher_2d_local& operator =(const global_scan_matcher_2d_local&) = delete;

    bool Init(const proto::LocalTrajectoryBuilderOptions2D& options,
              const float min_score_thresh,
              const float confidence_score_thresh,
              const std::string& pbstream_filepath,
              const bool use_ceres_to_refine,
              const bool use_on_scan,
              SlamCommon::CFileInterface *file_interface,
              SlamCommon::CConfigFileOperator *config_operator) override;
    bool Match(const transform::Rigid3d& gravity_alignment,
               const cartographer::sensor::PointCloud& point_cloud,
               Pose* const res, const double &vel, cartographer::transform::Rigid3d& local_to_global) const override;//lyy
    void StopMatching() override
    {
        if(nullptr != _faster_csm_ptr)
        {
            _faster_csm_ptr->StopMatching();
        }
    }
    Pose GetRelativePoseOfLastNode(const Pose &pose) const override;

    void AddSubmapIdAndFlag(int submap_id, bool assigned_flag) override {
        submap_id_ = submap_id;
        assigned_flag_ = assigned_flag;
    }

    float GetBestCandidateScore(){
        return _faster_csm_ptr->GetBestCandidateScore();
    }

    int GetMatchedSubmapId(){
        return _faster_csm_ptr->GetMatchedSubmapId();
    }

  private:
    bool LoadSubmaps(const proto::LocalTrajectoryBuilderOptions2D& options,
                     const std::string& pbfilepath,
                     const bool use_ceres_to_refine,
                     SlamCommon::CFileInterface *file_interface,
                     SlamCommon::CConfigFileOperator *config_operator);
    cartographer::sensor::PointCloud ProcessPointCloud(
        const cartographer::sensor::PointCloud& cloud_in,
        const cartographer::transform::Rigid3d& trans) const;

  private:
    CartoMapppingBridge::NodeOptions m_node_options;
    CartoMapppingBridge::TrajectoryOptions m_traj_options;
    cartographer::transform::Rigid2d _the_last_node_pose;
    std::vector<cartographer::transform::Rigid2d> _local_to_global_vec;
    std::unique_ptr<
        cartographer::mapping::scan_matching::fast_scan_matcher_2d_local> _faster_csm_ptr;

    int submap_id_;//lyy
    bool assigned_flag_;//lyy
};

}
}



#endif // GLOBAL_SCAN_MATCHER_2D_LOCAL_H
