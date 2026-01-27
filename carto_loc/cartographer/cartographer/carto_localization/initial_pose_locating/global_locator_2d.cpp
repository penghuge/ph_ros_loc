#include "global_locator_2d.h"

#include <deque>
#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include <cartographer/mapping/2d/submap_2d.h>

namespace Localization {
namespace CartoLocalization {

using Rigid2d = cartographer::transform::Rigid2d;
using Rigid3d = cartographer::transform::Rigid3d;

namespace {

void ErgodicTrajectoryNodes(
    const cartographer::mapping::proto::PoseGraph& pose_graph_proto, 
    Rigid2d* last_node_pose)
{
    CHECK(last_node_pose);
    // read all trajectory nodes
    int last_node_index = 0;
    cartographer::transform::proto::Rigid3d tmp_last_node_pose_proto;
    for (const auto &trajectory_proto : pose_graph_proto.trajectory())
    {
        for (const auto &node_proto : trajectory_proto.node())
        {
            if (node_proto.node_index() > last_node_index)
            {
                last_node_index = node_proto.node_index();
                tmp_last_node_pose_proto = node_proto.pose();
            }
        }
    }
    *last_node_pose =
    cartographer::transform::Project2D(
                cartographer::transform::ToRigid3(tmp_last_node_pose_proto));
    // LOG_INFO("the last trajectory node pose: %s", (*last_node_pose).DebugString().c_str());
}

template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation)
{
    const Eigen::Matrix<T, 3, 1> direction =
        rotation * Eigen::Matrix<T, 3, 1>::UnitX();
    return atan2(direction.y(), direction.x());
}
} // namespace

GlobalLocator2D::GlobalLocator2D() {}

bool GlobalLocator2D::Init(const CartoMapppingBridge::NodeOptions &node_options,
                           const CartoMapppingBridge::TrajectoryOptions &traj_options,
                           SlamCommon::CFileInterface *file_interface,
                           const float min_score_thresh,
                           const float confidence_score_thresh,
                           const std::string& pbstream_filepath, 
                           const bool use_ceres_to_refine)
{
    m_min_score_thresh = min_score_thresh;
    m_confidence_score_thresh = confidence_score_thresh;
    m_node_options = node_options;
    m_traj_options = traj_options;
    // Load submaps.
    // if(!pbstream_filepath.empty()) CHECK(!pbstream_filepath.empty());

    return LoadSubmaps(pbstream_filepath, use_ceres_to_refine, file_interface);
}

bool GlobalLocator2D::LoadSubmaps(const std::string& pbfilepath,
                                  const bool use_ceres_to_refine,
                                  SlamCommon::CFileInterface *file_interface)
{
    SlamCommon::Time start_time = SlamCommon::TimeNow();
    std::deque<cartographer::mapping::proto::Submap2D> submap2d_protos;
    if(!file_interface->GetSubmap2dProtos(submap2d_protos)){
        LOG_ERROR("LoadSubmaps: GetSubmap2dProtos failed!");
    }else{
        LOG_INFO("LoadSubmaps: GetSubmap2dProtos success!");
    }

    if(!file_interface->GetLocalToGlobalVec(_local_to_global_vec)){
        LOG_ERROR("LoadSubmaps: GetLocalToGlobalVec failed!");
        usleep(10000);
    }
    LOG_INFO("LoadSubmaps: GetLocalToGlobalVec success!");

    const int branch_and_bound_depth =
            m_node_options.map_builder_options.pose_graph_options()
            .constraint_builder_options().fast_correlative_scan_matcher_options()
            .branch_and_bound_depth();
    _faster_csm_ptr = absl::make_unique<cartographer::mapping::scan_matching::FasterCSM>(
                &submap2d_protos, branch_and_bound_depth, use_ceres_to_refine, file_interface);

    double cost_time = SlamCommon::ToSeconds(SlamCommon::TimeNow() - start_time);
    LOG_INFO("LoadSubmaps: copy all %d submaps cost %f s, branch_and_bound_depth: %d", _local_to_global_vec.size(), cost_time, branch_and_bound_depth);
    return true;
}

bool GlobalLocator2D::Match(const cartographer::sensor::PointCloud& point_cloud, 
                            const cartographer::transform::Rigid3d& trans, 
                            Pose* const res) const
{
    CHECK(res);
    //裁剪并转换激光点云到车身坐标系
    const cartographer::sensor::PointCloud processed_cloud =
            ProcessPointCloud(point_cloud, trans);
    if (processed_cloud.empty())
    {
        // LOG_ERROR_STREAM("Laser scan for matching is empty!");
        return false;
    }

    float score_tmp = 0.f;
    Rigid2d pose_estimate_tmp = Rigid2d::Identity();
    //使用初始位姿
    if(res->has_intial_guess)
    {
        pose_estimate_tmp = cartographer::transform::Project2D(res->rigid);
    }
    int matched_submap_id = 0;
    
    SlamCommon::Time start_time = SlamCommon::TimeNow();
    if (_faster_csm_ptr->MatchInMap(processed_cloud, res->has_intial_guess,
                                    res->search_window, res->min_search_time,
                                    m_min_score_thresh, m_confidence_score_thresh,
                                    &score_tmp, &pose_estimate_tmp,
                                    matched_submap_id,/*lyy*/
                                    _local_to_global_vec/*lyy*/))
    {
        const Rigid2d pose_global =
                _local_to_global_vec[matched_submap_id] * pose_estimate_tmp;

        //将2D坐标转换到3D
        res->rigid = cartographer::transform::Embed3D(pose_global);
        res->prob = score_tmp;

        double cost_time = SlamCommon::ToSeconds(SlamCommon::TimeNow() -start_time);
        LOG_INFO("faster_csm_ptr->MatchInMap success! cost %f s", cost_time);
        return true;
    }else{
        double cost_time = SlamCommon::ToSeconds(SlamCommon::TimeNow() -start_time);
        LOG_INFO("faster_csm_ptr->MatchInMap failed! cost %f s", cost_time);
        return false;
    }
}

//获取相对于最后节点的相对位姿
GlobalLocator::Pose GlobalLocator2D::GetRelativePoseOfLastNode(
        const GlobalLocator::Pose &pose) const
{
    // Global_pose and _the_last_node_pose are both in map's coordinate.
    // We need to calculate the relative_pose of
    // global_pose in _the_last_node_pose's coordinate.
    double tmp_x = pose.rigid.translation().x() - _the_last_node_pose.translation().x();
    double tmp_y = pose.rigid.translation().y() - _the_last_node_pose.translation().y();
    double tmp_theta = _the_last_node_pose.rotation().angle();
    double yaw = GetYaw(pose.rigid.rotation());
    double relative_pose_yaw = yaw - tmp_theta;
    GlobalLocator::Pose relative_pose;
    relative_pose.rigid =
            Rigid3d({tmp_x * cos(tmp_theta) + tmp_y * sin(tmp_theta),
                     - tmp_x * sin(tmp_theta) + tmp_y * cos(tmp_theta), 0.},
                    Eigen::AngleAxisd(relative_pose_yaw, Eigen::Vector3d::UnitZ()));
    relative_pose.prob = pose.prob;

    return relative_pose;
}

cartographer::sensor::PointCloud GlobalLocator2D::ProcessPointCloud(
        const cartographer::sensor::PointCloud& cloud_in,
        const cartographer::transform::Rigid3d& trans) const
{
    if (cloud_in.empty())
    {
        return cartographer::sensor::PointCloud();
    }
    // crop
    const auto& trj_2d_op =
      m_traj_options.trajectory_builder_options.trajectory_builder_2d_options();
    const float min_range_sq = trj_2d_op.min_range() * trj_2d_op.min_range();
    const float max_range_sq = trj_2d_op.max_range() * trj_2d_op.max_range();
    cartographer::sensor::PointCloud cloud_crop;
    //裁剪掉不在指定范围内的激光点
    for (const auto& pt : cloud_in)
    {
        const float range_sq = pt.position.x() * pt.position.x()
                             + pt.position.y() * pt.position.y();
        if (range_sq < min_range_sq || range_sq > max_range_sq)
        {
            continue;
        }
        cloud_crop.emplace_back(pt);
    }
    // transform to tracking，将激光点的cartesian坐标转换到车身坐标系下
    const cartographer::sensor::PointCloud transformed_cloud =
            cartographer::sensor::TransformPointCloud(
                cloud_crop, trans.cast<float>());

    return transformed_cloud;
}

} // namespace CartoLocalization
} // namespace Localization
