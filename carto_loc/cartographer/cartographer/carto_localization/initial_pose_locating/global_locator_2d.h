#ifndef GLOBAL_LOCATER_2D_H_
#define GLOBAL_LOCATER_2D_H_

#include "../scan_matching_2d/faster_csm.h"
#include "../initial_pose_locating/global_locator.h"
#include "cartographer/common/file_interface.h"

namespace Localization {
namespace CartoLocalization {

class GlobalLocator2D : public GlobalLocator {
public:
  GlobalLocator2D();
  ~GlobalLocator2D() override {};
  GlobalLocator2D(const GlobalLocator2D&) = delete;
  GlobalLocator2D& operator=(const GlobalLocator2D&) = delete;
  
  bool Init(const CartoMapppingBridge::NodeOptions &node_options,
            const CartoMapppingBridge::TrajectoryOptions &traj_options,
            SlamCommon::CFileInterface *file_interface,
            const float min_score_thresh,
            const float confidence_score_thresh,
            const std::string& pbstream_filepath,
            const bool use_ceres_to_refine) override;
  bool Match(const cartographer::sensor::PointCloud& point_cloud, 
             const cartographer::transform::Rigid3d& trans, 
             Pose* const res) const override;
  void StopMatching() override
  {
      if(nullptr != _faster_csm_ptr)
      {
          _faster_csm_ptr->StopMatching();
      }
  }
  Pose GetRelativePoseOfLastNode(const Pose &pose) const override;

private:
  bool LoadSubmaps(const std::string& pbfilepath,
                   const bool use_ceres_to_refine,
                   SlamCommon::CFileInterface *file_interface);
  cartographer::sensor::PointCloud ProcessPointCloud(
      const cartographer::sensor::PointCloud& cloud_in, 
      const cartographer::transform::Rigid3d& trans) const;

private:
  CartoMapppingBridge::NodeOptions m_node_options;
  CartoMapppingBridge::TrajectoryOptions m_traj_options;
  cartographer::transform::Rigid2d _the_last_node_pose;
  std::vector<cartographer::transform::Rigid2d> _local_to_global_vec;
  std::unique_ptr<
      cartographer::mapping::scan_matching::FasterCSM> _faster_csm_ptr;
};

} // namespace CartoLocalization
} // namespace Localization

#endif // GLOBAL_LOCATER_2D_H_
