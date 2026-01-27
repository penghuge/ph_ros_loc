#include "global_scan_matcher_2d_local.h"

#include <deque>
#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include <cartographer/mapping/2d/submap_2d.h>

namespace cartographer {
namespace mapping {

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

global_scan_matcher_2d_local::global_scan_matcher_2d_local() {
    submap_id_ = 0;//lyy
    assigned_flag_ = false;//lyy
}

bool global_scan_matcher_2d_local::Init(const proto::LocalTrajectoryBuilderOptions2D& options,
                                        const float min_score_thresh,
                                        const float confidence_score_thresh,
                                        const std::string& pbstream_filepath,
                                        const bool use_ceres_to_refine,
                                        const bool use_on_scan,
                                        SlamCommon::CFileInterface *file_interface,
                                        SlamCommon::CConfigFileOperator *config_operator)
{
    m_min_score_thresh = min_score_thresh;
    m_confidence_score_thresh = confidence_score_thresh;
    is_one_scan_ = use_on_scan;
    // Load submaps.
    // CHECK(!pbstream_filepath.empty());
    return LoadSubmaps(options, pbstream_filepath, use_ceres_to_refine, file_interface, config_operator);
}

bool global_scan_matcher_2d_local::LoadSubmaps(const proto::LocalTrajectoryBuilderOptions2D& options,
                                               const std::string& pbfilepath,
                                               const bool use_ceres_to_refine,
                                               SlamCommon::CFileInterface *file_interface,
                                               SlamCommon::CConfigFileOperator *config_operator)
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
    }else{
        LOG_INFO("LoadSubmaps: GetLocalToGlobalVec success!");
    }

   const int branch_and_bound_depth =
           m_node_options.map_builder_options.pose_graph_options()
           .constraint_builder_options().fast_correlative_scan_matcher_options()
           .branch_and_bound_depth();
    _faster_csm_ptr = absl::make_unique<
            cartographer::mapping::scan_matching::fast_scan_matcher_2d_local>(
                &submap2d_protos, 7/*branch_and_bound_depth*/, use_ceres_to_refine, options, file_interface, config_operator);
    
    double cost_time = SlamCommon::ToSeconds(SlamCommon::TimeNow() - start_time);
    LOG_INFO("LoadSubmaps: copy all %d submaps cost %f s, branch_and_bound_depth: %d", _local_to_global_vec.size(), cost_time, branch_and_bound_depth);
    
    return true;
}


bool global_scan_matcher_2d_local::Match(const transform::Rigid3d& gravity_alignment,
                                         const cartographer::sensor::PointCloud& point_cloud,
                                         Pose* const res, const double &cur_vel, 
                                         cartographer::transform::Rigid3d& local_to_global) const
{
    CHECK(res);
    // LOG_DEBUG("non_gravity_aligned_pose_prediction(%f, %f, %f)", res->rigid.translation().x(),
    //          res->rigid.translation().y(),
    //          SlamCommon::RadToDeg(cartographer::transform::GetYaw(res->rigid)));
    // LOG_DEBUG("gravity_alignment(%f, %f, %f)", gravity_alignment.translation().x(),
    //          gravity_alignment.translation().y(),
    //          SlamCommon::RadToDeg(cartographer::transform::GetYaw(gravity_alignment)));
    if(point_cloud.empty())
    {
        // LOG_INFO("scan for matching is empty!!!!");
        return false;
    }
    float score_tmp = 0.f;

    Rigid2d pose_estimate_tmp = Rigid2d::Identity();
    if(res->has_intial_guess){
        pose_estimate_tmp = transform::Project2D(res->rigid * gravity_alignment.inverse());
    }
    int matched_submap_id = 0;
    _faster_csm_ptr->AddSubmapIdAndFlag(submap_id_, assigned_flag_);//lyy
    _faster_csm_ptr->MatchInMap(point_cloud, res->has_intial_guess,
                                res->search_window, res->angle_window, is_one_scan_,
                                m_min_score_thresh, m_confidence_score_thresh,
                                &score_tmp, &pose_estimate_tmp,
                                matched_submap_id, cur_vel, _local_to_global_vec);

    double tmp_x = _local_to_global_vec[matched_submap_id].translation().x();
    double tmp_y = _local_to_global_vec[matched_submap_id].translation().y();
    double tmp_th = _local_to_global_vec[matched_submap_id].rotation().angle();
    const Rigid2d tmp_pose = Rigid2d({tmp_x, tmp_y}, tmp_th);
    if(matched_submap_id == -1){
        local_to_global = cartographer::transform::Embed3D(Rigid2d::Identity());//lyy
        LOG_WARNING_THROTTLE(1.0, "matched_submap_id = -1 !!");
    }else{
        local_to_global = cartographer::transform::Embed3D(tmp_pose);//lyy
    }
    
    const Rigid2d pose_global = pose_estimate_tmp;

    //将2D坐标转换到3D
    res->rigid = cartographer::transform::Embed3D(pose_global);
    res->prob = score_tmp;

    return true;
}

//获取相对于最后节点的相对位姿
global_scan_matcher_2d_local::Pose global_scan_matcher_2d_local::GetRelativePoseOfLastNode(
        const GlobalLocal::Pose &pose) const
{
    // Global_pose and _the_last_node_pose are both in map's coordinate.
    // We need to calculate the relative_pose of
    // global_pose in _the_last_node_pose's coordinate.
    double tmp_x = pose.rigid.translation().x() - _the_last_node_pose.translation().x();
    double tmp_y = pose.rigid.translation().y() - _the_last_node_pose.translation().y();
    double tmp_theta = _the_last_node_pose.rotation().angle();
    double yaw = GetYaw(pose.rigid.rotation());
    double relative_pose_yaw = yaw - tmp_theta;
    GlobalLocal::Pose relative_pose;
    relative_pose.rigid =
            Rigid3d({tmp_x * cos(tmp_theta) + tmp_y * sin(tmp_theta),
                     - tmp_x * sin(tmp_theta) + tmp_y * cos(tmp_theta), 0.},
                    Eigen::AngleAxisd(relative_pose_yaw, Eigen::Vector3d::UnitZ()));
    relative_pose.prob = pose.prob;

    return relative_pose;
}

cartographer::sensor::PointCloud global_scan_matcher_2d_local::ProcessPointCloud(
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
    const float min_range_sq = 0.3 * 0.3/*trj_2d_op.min_range() * trj_2d_op.min_range()*/;
    const float max_range_sq = 100 * 100/*trj_2d_op.max_range() * trj_2d_op.max_range()*/;
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

}
}

