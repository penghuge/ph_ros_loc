#include "task_manager.h"

namespace task
{
TaskManager::TaskManager() : mpc_loader_("nav_core", "nav_core::BaseLocalPlanner")
{
    vec_step_task_stru_.clear();
    vec_step_task_stru_global_.clear();
    vec_path_point_global_.clear();

    nh_.param<std::string>("/ip_address_dtof", ip_camera_dtof_, std::string("192.168.100.82"));
    nh_.param<bool>("/flag_use_2d_loc", flag_use_2d_loc_, true);
    nh_.param<bool>("/flag_show_project_point", flag_show_project_point_, false);
    nh_.param<bool>("/flag_enable_mpc", flag_enable_mpc_, false);
    nh_.param<bool>("/flag_enable_pallet_identify", flag_enable_pallet_identify_, false);
    nh_.param<double>("/min_dis_ahead_point", min_dis_ahead_point_, 1.5);
    nh_.param<double>("/scale_ahead_point", scale_ahead_point_, 0.7);
    nh_.param<double>("/step_generate_path", step_generate_path_, 0.1);
    nh_.param<double>("/threshold_arrive_des", threshold_arrive_des_, 1.0);
    nh_.param<double>("/distance_head", distance_head_, 1.0);
    nh_.param<double>("/distance_tail", distance_tail_, 1.0);
    nh_.param<double>("/dis_derail", dis_derail_, 0.2);
    nh_.param<std::string>("/topic_task_rcs", topic_task_rcs_, "task_rcs");

    pub_dense_path_ = nh_.advertise<nav_msgs::Path>("dense_path_show", 1);
    pub_task_points_ = nh_.advertise<visualization_msgs::Marker>("points_task_show", 1);
    pub_ahead_point_ = nh_.advertise<visualization_msgs::Marker>("ahead_point_show", 1);
    pub_base_project_point_ = nh_.advertise<visualization_msgs::Marker>("base_project_point_show", 1);
    pub_head_project_point_ = nh_.advertise<visualization_msgs::Marker>("head_project_point_show", 1);
    pub_tail_project_point_ = nh_.advertise<visualization_msgs::Marker>("tail_project_point_show", 1);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    pub_nav_data_ = nh_.advertise<common_msgs::nav_data>("nav_data", 1);
    pub_action_data_ = nh_.advertise<common_msgs::action_data>("action_data", 1);
    pub_ipc_res_data_ = nh_.advertise<common_msgs::ipc_res_data>("ipc_res_data", 1);
    sub_manual_control_ = nh_.subscribe("/manual_control", 100, &TaskManager::ManualControlProcess, this);

    update_params_srv_ = nh_.advertiseService("UpdateTaskManagerParams", &TaskManager::UpdateParamsSrv, this);

    if (flag_use_2d_loc_) {
        sub_odom_ = nh_.subscribe("/odom", 100, &TaskManager::OdomProcess, this);
        sub_current_pose_ = nh_.subscribe("/tracked_pose", 100, &TaskManager::Current2dPoseProcess, this);
    } else {
        sub_current_pose_ = nh_.subscribe("/fused_localization", 100, &TaskManager::Current3dPoseProcess, this);
    }
    sub_rcs_task_ = nh_.subscribe(topic_task_rcs_, 10, &TaskManager::TaskProcess, this);
    sub_plc_res_ = nh_.subscribe("plc_res_nav_data", 10, &TaskManager::ProcessPlcResData, this);
    sub_plc_action_res_ = nh_.subscribe("plc_res_action_data", 10, &TaskManager::ProcessPlcResActionData, this);

    if (flag_enable_pallet_identify_) {
        // init pallet identify
        p_pallet_identify_ = std::make_unique<pallet_idenfity::PalletIdentifyM4>();
        flag_init_camera_ = p_pallet_identify_->InitPalletIdentify(ip_camera_dtof_);
        if (flag_init_camera_) {
            ROS_INFO(" pi_step_1 : InitPalletIdentify success ......");
        } else {
            ROS_INFO(" pi_err_1 : InitPalletIdentify failed ......");
        }
        sleep(1);  // wait for m4 pallet identify thread data
        p_pallet_identify_->TestPalletIdentifyData();
    }

    // init mpc
    /*
    std::string mpc_planner_name = std::string("mpc_path_follower/MpcPathFollowerRos");
    try {
        mpc_planner_ = mpc_loader_.createInstance(mpc_planner_name);
        ROS_INFO(" Load plugin %s successed ", mpc_planner_name.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
        ROS_ERROR(
            "Failed to create the %s planner, are you sure it is properly registered and that the containing library "
            "is built? Exception: %s",
            mpc_planner_name.c_str(), ex.what());
    }
    */

    // 设置动态参数服务器
    dyn_server_.setCallback(boost::bind(&TaskManager::DynamicReconfigureCallback, this, _1, _2));

    ROS_INFO(
        " topic of task_rcs is: %s, min_dis_ahead_point: %f, scale_ahead_point: %f, threshold_arrive_des: %f, "
        "distance_head: %f, distance_tail: %f",
        topic_task_rcs_.c_str(), min_dis_ahead_point_, scale_ahead_point_, threshold_arrive_des_, distance_head_,
        distance_tail_);
}

bool TaskManager::UpdateParamsSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    nh_.getParam("/task_manager/derail_limit_x", derail_limit_x_);
    nh_.getParam("/task_manager/derail_limit_y", derail_limit_y_);
    nh_.getParam("/task_manager/derail_limit_theta", derail_limit_theta_);
    nh_.getParam("/task_manager/landmark_thred", landmark_thred_);
    nh_.getParam("/task_manager/static_work_thred", static_work_thred_);
    res.success = true;
    res.message = "Update params";
    return true;
}

void TaskManager::FillStepTaskInfo(const RCSNodeStru& cur_node, uint16_t* data_step_task)
{
    switch (static_cast<RCSNodeTypeEnum>(cur_node.type)) {
        case RCS_NODE_AUTORUN_BY_CAMERA:
        case RCS_NODE_LINE:
        case RCS_NODE_AUTORUN_LINE: {
            RCSLineNodeStru line_node;
            line_node.Reset(cur_node);
            memcpy(data_step_task, (uint16_t*)(&line_node), sizeof(RCSLineNodeStru));
            break;
        }
        case RCS_NODE_CURVE: {
            RCSCurveNodeStru curve_node;
            curve_node.Reset(cur_node);
            memcpy(data_step_task, (uint16_t*)(&curve_node), sizeof(RCSCurveNodeStru));
            break;
        }
        case RCS_NODE_WORK: {
            RCSWorkNodeStru work_node;
            work_node.Reset(cur_node);
            memcpy(data_step_task, (uint16_t*)(&work_node), sizeof(RCSWorkNodeStru));
            break;
        }
        case RCS_NODE_CHARGE: {
            RCSChargeNodeStru charge_node;
            charge_node.Reset(cur_node);
            memcpy(data_step_task, (uint16_t*)(&charge_node), sizeof(RCSChargeNodeStru));
            break;
        }
        case RCS_NODE_TRANSFER: {
            RCSTransferNodeStru transfer_node;
            transfer_node.Reset(cur_node);
            memcpy(data_step_task, (uint16_t*)(&transfer_node), sizeof(RCSTransferNodeStru));
            break;
        }
        case RCS_NODE_STANDBY_OPE: {
            RCSStandbyNodeStru standby_node;
            standby_node.Reset(cur_node);
            memcpy(data_step_task, (uint16_t*)(&standby_node), sizeof(RCSStandbyNodeStru));
            break;
        }
        case RCS_NODE_TASK_FINISH: {
            RCSFinishNodeStru finish_node;
            finish_node.Reset(cur_node);
            memcpy(data_step_task, (uint16_t*)(&finish_node), sizeof(RCSFinishNodeStru));
            break;
        }
        default: {
            //无对应类型，清空
            memset(data_step_task, 0, sizeof(RCSLineNodeStru));
            ROS_ERROR("tm_err_8: no this type ");
            break;
        }
    }
}

void TaskManager::GetCurrentStepTaskRemainDis(double& dis_remain)
{
    geometry_msgs::Point32 pose_current, pose_target;
    pose_current.x = pose_current_.position.x;
    pose_current.y = pose_current_.position.y;
    pose_target.x = mm_to_m_ * vec_step_task_stru_global_[index_current_step_task_].end_pt_x;
    pose_target.y = mm_to_m_ * vec_step_task_stru_global_[index_current_step_task_].end_pt_y;
    dis_remain = CalculateDistance(pose_current, pose_target);

    // ROS_INFO("test_1: dis_remain: %f, pose_target: [%f, %f] ", dis_remain, pose_target.x, pose_target.y);
}

// 需要注意自旋的id_a 与 id_b
// 一致，如果这些动作连接其他停车类型任务，可能瞬间完成任务，因为PLC一直上报完成在接收到下一个任务之前
bool TaskManager::FinishedCurrentStepTask()
{
    if (IPC_FOLLOW_PATH == task_type_curr_) {
        bool flag_two_line = (static_cast<int16_t>(RCS_NODE_LINE) == type_curr_task_) &&
                             (static_cast<int16_t>(RCS_NODE_LINE) == type_next_task_);
        double dis_remain = 0;
        double threshold_arrive = threshold_arrive_des_;
        if (flag_two_line) {
            threshold_arrive = threshold_arrive_des_;
        }
        GetCurrentStepTaskRemainDis(dis_remain);
        if (dis_remain < threshold_arrive) {
            if (CheckNextTaskIsStopType()) {
                if (g_id_stop_node_ == vec_step_task_stru_global_[index_current_step_task_].id_a &&
                    2 == g_status_stop_) {
                    ROS_INFO("tm_step_20: finish ipc_follow_path task, next task is stop type");
                    return true;
                } else {
                    return false;
                }
            } else {
                ROS_INFO("tm_step_15: finish ipc_follow_path task ");
                return true;
            }
        } else {
            return false;
        }
    } else if (IPC_AROUND_SPIN == task_type_curr_) {
        if (g_id_stop_node_ == vec_step_task_stru_global_[index_current_step_task_].id_a && 2 == g_status_stop_) {
            ROS_INFO("tm_step_16: finish ipc_around_spin task ");
            return true;
        } else {
            return false;
        }
    } else if (IPC_PICK_CARGO == task_type_curr_) {
        if (g_id_stop_node_ == vec_step_task_stru_global_[index_current_step_task_].id_a && 2 == g_status_stop_) {
            ROS_INFO("tm_step_19: finish pick_cargo task ");
            return true;
        } else {
            return false;
        }
    } else if (IPC_WORK == task_type_curr_) {
        bool flag_finish_action =
            ((ACUATOR_LIFT_DOWN_DONE == g_status_action_1) || (ACUATOR_LIFT_UP_DONE == g_status_action_1));
        if (flag_finish_action && (g_id_finish_action_1 == id_action_current_)) {
            ROS_INFO("tm_step_17: finish work task, lift type is: %d, id_finish_action: %d, id_action_current: %d",
                     g_status_action_1, g_id_finish_action_1, id_action_current_);
            id_action_current_++;
            return true;
        } else {
            return false;
        }
    } else if (IPC_CHARGE == task_type_curr_) {
        if (static_cast<int8_t>(STATUS_CHARGE_FINISHED) == g_status_charge_) {
            ROS_INFO("tm_step_22: finish charge task ");
            return true;
        } else {
            return false;
        }
    } else if (IPC_TASK_FINISH == task_type_curr_) {
        ROS_INFO("tm_step_21: finish ipc_task_finish task !!!!!! ");
        return true;
    } else {
        ROS_ERROR("tm_err_10: not exist this type ");
    }
}

void TaskManager::UpdataCurrentStepTaskIndex()
{
    if (FinishedCurrentStepTask()) {
        index_current_step_task_++;
        if (index_current_step_task_ == vec_step_task_stru_global_.size()) {
            status_ipc_ = STATUS_COMPLETED_TASK;
            compelete_percent_ = 100;
            ROS_INFO("tm_step_13: finish current all task ");
        } else {
            RCSNodeStru cur_step_task = vec_step_task_stru_global_[index_current_step_task_];
            start_point_curr_task_.x = cur_step_task.start_pt_x * mm_to_m_;
            start_point_curr_task_.y = cur_step_task.start_pt_y * mm_to_m_;
            center_point_curr_task_.x = cur_step_task.center_pt_x * mm_to_m_;
            center_point_curr_task_.y = cur_step_task.center_pt_y * mm_to_m_;
            end_point_curr_task_.x = cur_step_task.end_pt_x * mm_to_m_;
            end_point_curr_task_.y = cur_step_task.end_pt_y * mm_to_m_;
            GetCurrAndNextTaskInfo(index_current_step_task_, id_curr_task_, type_curr_task_, speed_curr_task_,
                                   radius_sign_curr_task_, radius_length_curr_task, type_next_task_, speed_next_task_,
                                   radius_sign_next_task_);
            FillStepTaskInfo(cur_step_task, (uint16_t*)(&data_res_ipc_.node_info_1));
            ROS_INFO("tm_step_14: switch to next task -> index_current_step_task: %d ", index_current_step_task_);
        }
    }
}

void TaskManager::PubIpcResData()
{
    common_msgs::ipc_res_data res_data;
    res_data.tr_cmd = data_res_ipc_.tr_cmd;
    res_data.tr_id_index = data_res_ipc_.tr_id_index;
    res_data.tr_id_hour = data_res_ipc_.tr_id_hour;
    res_data.tr_id_day = data_res_ipc_.tr_id_day;
    res_data.tr_id_month = data_res_ipc_.tr_id_month;
    res_data.tr_year = data_res_ipc_.tr_year;
    res_data.tr_sender = data_res_ipc_.tr_sender;
    res_data.tr_type = data_res_ipc_.tr_type;
    res_data.tr_step_size = data_res_ipc_.tr_step_size;
    res_data.tr_start_delay = data_res_ipc_.tr_start_delay;

    res_data.tr_check_state = data_res_ipc_.tr_check_state;
    // res_data.tr_check_error  = data_res_ipc_.tr_check_error;
    // res_data.tr_total_odom   = data_res_ipc_.tr_total_odom;
    // res_data.tr_execute_time = data_res_ipc_.tr_execute_time;
    // res_data.tr_complete_percent = data_res_ipc_.tr_complete_percent;
    res_data.tr_node_index = index_current_step_task_ + 1;
    // res_data.tr_pause_info       = data_res_ipc_.tr_pause_info;
    res_data.node_compelete_percent = compelete_percent_;
    res_data.node_type = type_curr_task_;
    res_data.node_next_type = type_next_task_;
    // res_data.node_stop_state  = data_res_ipc_.node_stop_state;
    // res_data.node_is_end_stop = data_res_ipc_.node_is_end_stop;

    res_data.node_info_1 = data_res_ipc_.node_info_1;
    res_data.node_info_2 = data_res_ipc_.node_info_2;
    res_data.node_info_3 = data_res_ipc_.node_info_3;
    res_data.node_info_4 = data_res_ipc_.node_info_4;
    res_data.node_info_5 = data_res_ipc_.node_info_5;
    res_data.node_info_6 = data_res_ipc_.node_info_6;
    res_data.node_info_7 = data_res_ipc_.node_info_7;
    res_data.node_info_8 = data_res_ipc_.node_info_8;
    res_data.node_info_9 = data_res_ipc_.node_info_9;
    res_data.node_info_10 = data_res_ipc_.node_info_10;
    res_data.node_info_11 = data_res_ipc_.node_info_11;
    res_data.node_info_12 = data_res_ipc_.node_info_12;
    res_data.node_info_13 = data_res_ipc_.node_info_13;
    res_data.node_info_14 = data_res_ipc_.node_info_14;
    res_data.node_info_15 = data_res_ipc_.node_info_15;
    res_data.node_info_16 = data_res_ipc_.node_info_16;
    res_data.node_info_17 = data_res_ipc_.node_info_17;
    res_data.node_info_18 = data_res_ipc_.node_info_18;
    res_data.node_info_19 = data_res_ipc_.node_info_19;
    res_data.node_info_20 = data_res_ipc_.node_info_20;

    res_data.poweron_time = data_res_ipc_.poweron_time;

    res_data.agv_pose_x = static_cast<int32_t>(1000. * pose_current_.position.x);
    res_data.agv_pose_y = static_cast<int32_t>(1000. * pose_current_.position.y);

    double yaw_to_rcs = yaw_current_;  // fit rcs show
    if (yaw_to_rcs < 0) {
        yaw_to_rcs += 2 * M_PI;
    }
    res_data.agv_pose_theta = static_cast<int16_t>(10. * 180.0 * yaw_to_rcs / M_PI);

    res_data.lift_height = data_res_ipc_.lift_height;
    res_data.lift_speed = data_res_ipc_.lift_speed;
    res_data.agv_vel = static_cast<int16_t>(1000. * g_vel_linear_);
    res_data.is_parked = data_res_ipc_.is_parked;
    res_data.steering_angle = data_res_ipc_.steering_angle;
    res_data.battery_soc = g_capacity_remain_;
    res_data.com_quantity = data_res_ipc_.com_quantity;
    res_data.nav_quantity = data_res_ipc_.nav_quantity;
    // res_data.task_status = data_res_ipc_.task_status;
    res_data.task_status = static_cast<int16_t>(status_ipc_);
    res_data.mode_status = data_res_ipc_.mode_status;
    res_data.auto_status = data_res_ipc_.auto_status;
    res_data.version_hardware = 01;
    res_data.version_software = 22;
    // res_data.error_1 = data_res_ipc_.error_1;
    // res_data.error_2 = data_res_ipc_.error_2;
    // res_data.error_3 = data_res_ipc_.error_3;
    // res_data.error_4 = data_res_ipc_.error_4;
    // res_data.error_5 = data_res_ipc_.error_5;
    res_data.warning = data_res_ipc_.warning;
    res_data.cargo_loaded = g_status_sensor_signal_;

    // ROS_ERROR(" node info : [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
    //     res_data.node_info_1, res_data.node_info_2, res_data.node_info_3, res_data.node_info_4, res_data.node_info_5,
    //     res_data.node_info_6, res_data.node_info_7, res_data.node_info_8, res_data.node_info_9,
    //     res_data.node_info_10, res_data.node_info_11, res_data.node_info_12, res_data.node_info_13,
    //     res_data.node_info_14, res_data.node_info_15, res_data.node_info_16, res_data.node_info_17,
    //     res_data.node_info_18, res_data.node_info_19, res_data.node_info_20);

    pub_ipc_res_data_.publish(res_data);
}

// 下一子任务的起点需要是上一个子任务的终点，如果校验错误，需要返回错误码
bool TaskManager::CheckRcsPathTaskValidity(const std::vector<RCSNodeStru>& vec_step_task_stru)
{
    if (vec_step_task_stru.size() < 2) {
        ROS_ERROR("tm_warn_1: The task list is too short to validate ");
        return true;
    }
    for (size_t i = 0; i < vec_step_task_stru.size() - 1; ++i) {
        int32_t current_end_pt_x = vec_step_task_stru[i].end_pt_x;
        int32_t current_end_pt_y = vec_step_task_stru[i].end_pt_y;
        int32_t next_start_pt_x = vec_step_task_stru[i + 1].start_pt_x;
        int32_t next_start_pt_y = vec_step_task_stru[i + 1].start_pt_y;

        // Check if the current line's end point matches the next line's start point
        if (current_end_pt_x != next_start_pt_x || current_end_pt_y != next_start_pt_y) {
            ROS_ERROR("tm_err_7: Invalid path: step task %d end point does not match %d start point. ", i, i + 1);
            return false;
        }
    }
    ROS_INFO("tm_step_12: all tasks in the list are connected properly.");
    return true;
}

void TaskManager::GenerateLinePoints(const geometry_msgs::Point32& point_start, const geometry_msgs::Point32& point_end,
                                     std::vector<geometry_msgs::Point32>& points_generate)
{
    double distance = std::sqrt(std::pow(point_end.x - point_start.x, 2) + std::pow(point_end.y - point_start.y, 2));
    size_t num_points = static_cast<size_t>(std::ceil(distance / step_generate_path_));  // 按步长插值
    if (distance <= 0 || 0 == num_points) {
        ROS_ERROR("tm_err_6: error distance or num_points ");
        return;
    }
    for (size_t j = 0; j <= num_points; ++j) {
        double t = static_cast<double>(j) / num_points;  // 插值比例
        double x_new = point_start.x + t * (point_end.x - point_start.x);
        double y_new = point_start.y + t * (point_end.y - point_start.y);
        double z_new = point_start.z + t * (point_end.z - point_start.z);
        geometry_msgs::Point32 pose;
        pose.x = x_new;
        pose.y = y_new;
        pose.z = z_new;
        points_generate.emplace_back(pose);
    }
}

void TaskManager::GenerateArcPoints(const geometry_msgs::Point32& point_start, const geometry_msgs::Point32& point_end,
                                    const geometry_msgs::Point32& center, double desired_arc_length,
                                    std::vector<geometry_msgs::Point32>& points_generate)
{
    double radius_start = std::sqrt(std::pow(point_start.x - center.x, 2) + std::pow(point_start.y - center.y, 2));
    double radius_end = std::sqrt(std::pow(point_end.x - center.x, 2) + std::pow(point_end.y - center.y, 2));
    if (std::fabs(radius_start - radius_end) > 1e-3) {
        ROS_ERROR("tm_err_9: Start and end points are not on the same circle.");
        return;
    }
    double radius = radius_start;
    double angle_start = std::atan2(point_start.y - center.y, point_start.x - center.x);
    double angle_end = std::atan2(point_end.y - center.y, point_end.x - center.x);
    // Normalize angles to range [0, 2*PI)
    if (angle_start < 0) {
        angle_start += 2 * M_PI;
    }
    if (angle_end < 0) {
        angle_end += 2 * M_PI;
    }
    // Compute arc lengths in both directions
    double arc_length_ccw =
        (angle_end >= angle_start) ? (angle_end - angle_start) * radius : (2 * M_PI - angle_start + angle_end) * radius;
    double arc_length_cw =
        (angle_start >= angle_end) ? (angle_start - angle_end) * radius : (2 * M_PI - angle_end + angle_start) * radius;
    // Select the direction based on the desired arc length
    bool is_ccw = std::fabs(arc_length_ccw - desired_arc_length) < std::fabs(arc_length_cw - desired_arc_length);
    double angle_increment = step_generate_path_ / radius;
    if (is_ccw) {
        if (angle_end < angle_start) {
            angle_end += 2 * M_PI;
        }
    } else {
        if (angle_start < angle_end) {
            angle_start += 2 * M_PI;
        }
        angle_increment = -angle_increment;
    }
    // Generate points along the arc
    for (double angle = angle_start; (is_ccw ? angle <= angle_end : angle >= angle_end); angle += angle_increment) {
        geometry_msgs::Point32 point;
        point.x = center.x + radius * std::cos(angle);
        point.y = center.y + radius * std::sin(angle);
        point.z = point_start.z;
        points_generate.emplace_back(point);
    }
    points_generate.emplace_back(point_end);  // Ensure the end point is included
}

void TaskManager::GenerateGlobalDensePath(const std::vector<RCSNodeStru>& vec_step_task_stru,
                                          std::vector<std::vector<geometry_msgs::Point32>>& points_generate)
{
    for (size_t i = 0; i < vec_step_task_stru.size(); ++i) {
        RCSNodeTypeEnum type_step_task = static_cast<RCSNodeTypeEnum>(vec_step_task_stru[i].type);
        std::vector<geometry_msgs::Point32> points_step_task;
        points_step_task.clear();
        switch (type_step_task) {
            case RCS_NODE_LINE: {
                geometry_msgs::Point32 point_start, point_end;
                point_start.x = mm_to_m_ * vec_step_task_stru[i].start_pt_x;
                point_start.y = mm_to_m_ * vec_step_task_stru[i].start_pt_y;
                point_end.x = mm_to_m_ * vec_step_task_stru[i].end_pt_x;
                point_end.y = mm_to_m_ * vec_step_task_stru[i].end_pt_y;
                GenerateLinePoints(point_start, point_end, points_step_task);
                points_generate.emplace_back(points_step_task);
                break;
            }
            case RCS_NODE_CURVE: {
                geometry_msgs::Point32 point_start, point_end, point_center;
                point_start.x = mm_to_m_ * vec_step_task_stru[i].start_pt_x;
                point_start.y = mm_to_m_ * vec_step_task_stru[i].start_pt_y;
                point_end.x = mm_to_m_ * vec_step_task_stru[i].end_pt_x;
                point_end.y = mm_to_m_ * vec_step_task_stru[i].end_pt_y;
                point_center.x = mm_to_m_ * vec_step_task_stru[i].center_pt_x;
                point_center.y = mm_to_m_ * vec_step_task_stru[i].center_pt_y;
                double length = mm_to_m_ * vec_step_task_stru[i].length;
                GenerateArcPoints(point_start, point_end, point_center, length, points_step_task);
                points_generate.emplace_back(points_step_task);
                break;
            }
            case RCS_NODE_WORK: {  // 确保 points_generate size 与 子任务 size 一致
                geometry_msgs::Point32 point_tmp;
                points_step_task.emplace_back(point_tmp);
                points_generate.emplace_back(points_step_task);
                break;
            }
            case RCS_NODE_TRANSFER: {
                /* code */
                break;
            }
            case RCS_NODE_AUTORUN_LINE: {
                /* code */
                break;
            }
            case RCS_NODE_AUTORUN_CURVE: {
                /* code */
                break;
            }
            case RCS_NODE_STANDBY_OPE: {
                /* code */
                break;
            }
            case RCS_NODE_CHARGE:
            case RCS_NODE_AUTORUN_BY_CAMERA:
            case RCS_NODE_TASK_FINISH: {
                geometry_msgs::Point32 point_tmp;
                points_step_task.emplace_back(point_tmp);
                points_generate.emplace_back(points_step_task);
                break;
            }
            default:
                break;
        }
    }
}

void TaskManager::ShowGlobalDensePath(const std::vector<std::vector<geometry_msgs::Point32>>& points_vec)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    for (size_t i = 0; i < points_vec.size(); ++i) {
        for (size_t j = 0; j < points_vec[i].size(); j++) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = points_vec[i][j].x;
            pose.pose.position.y = points_vec[i][j].y;
            pose.pose.position.z = points_vec[i][j].z;
            pose.pose.orientation.w = 1.0;  // 默认方向
            path.poses.emplace_back(pose);
        }
    }
    pub_dense_path_.publish(path);
}

void TaskManager::ShowRcsTaskPoints(const std::vector<RCSNodeStru>& tasks_vec)
{
    visualization_msgs::Marker pub_points;
    pub_points.id = 2;
    pub_points.header.frame_id = "map";
    pub_points.header.stamp = ros::Time::now();
    pub_points.type = visualization_msgs::Marker::POINTS;
    pub_points.scale.x = 1;
    pub_points.scale.y = 1;
    pub_points.color.g = 1.0f;
    pub_points.color.a = 1.0;
    for (int j = 0; j < tasks_vec.size(); j++) {
        geometry_msgs::Point point;
        if (0 == j) {
            point.x = tasks_vec[j].start_pt_x * mm_to_m_;
            point.y = tasks_vec[j].start_pt_y * mm_to_m_;
            pub_points.points.push_back(point);
        }
        point.x = tasks_vec[j].end_pt_x * mm_to_m_;
        point.y = tasks_vec[j].end_pt_y * mm_to_m_;
        pub_points.points.push_back(point);
    }
    pub_task_points_.publish(pub_points);
}

void TaskManager::GetLookaheadPoint(geometry_msgs::Point32& point_lookahead,
                                    const std::vector<geometry_msgs::Point32>& points_step_task)
{
    geometry_msgs::Point32 pose_current;
    pose_current.x = pose_current_.position.x;
    pose_current.y = pose_current_.position.y;
    size_t index_closest_point = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points_step_task.size(); ++i) {  // 遍历路径上的每一个点，找到距离最小的点
        double distance = CalculateDistance(points_step_task[i], pose_current);
        if (distance < min_distance) {
            min_distance = distance;
            index_closest_point = i;
        }
    }

    double dis_sum = 0;
    for (size_t i = index_closest_point; i < points_step_task.size() - 1; ++i) {
        dis_sum += CalculateDistance(points_step_task[i], points_step_task[i + 1]);
        if (dis_sum >= distance_lookahead_) {
            point_lookahead = points_step_task[i + 1];
            return;
        }
    }
    point_lookahead = points_step_task.back();
}

bool TaskManager::CheckNextTaskIsStopType()
{
    if (index_current_step_task_ + 1 >= vec_step_task_stru_global_.size()) {
        return true;
    }
    int16_t type_next_task = vec_step_task_stru_global_[index_current_step_task_ + 1].type;
    int16_t map_id_next_task = vec_step_task_stru_global_[index_current_step_task_ + 1].map_id;
    return CheckTaskIsStopType(type_next_task, map_id_next_task);
}

bool TaskManager::CheckTaskIsStopType(const int16_t type_task, const int16_t map_id)
{
    RCSNodeTypeEnum type = static_cast<RCSNodeTypeEnum>(type_task);
    bool is_stop_type = (1 == map_id) ||  // 表示自旋
                        (RCS_NODE_WORK == type) || (RCS_NODE_AUTORUN_LINE == type) || (RCS_NODE_CHARGE == type) ||
                        (RCS_NODE_AUTORUN_BY_CAMERA == type) || (RCS_NODE_TASK_FINISH == type);
    return is_stop_type;
}

// 获取当前任务以及下一个任务的信息: 当前节点ID、类型、速度、转弯半径以及转弯弧长，下一个节点类型、速度、转弯半径
void TaskManager::GetCurrAndNextTaskInfo(const size_t& index_step_task, int16_t& id_curr_task, int16_t& type_curr_task,
                                         float& speed_curr_task, float& radius_sign_curr_task,
                                         float& radius_length_curr_task, int16_t& type_next_task,
                                         float& speed_next_task, float& radius_sign_next_task)
{
    RCSNodeStru curr_task = vec_step_task_stru_global_[index_step_task];
    id_curr_task = curr_task.id_a;
    type_curr_task = curr_task.type;
    speed_curr_task = curr_task.target_vel * mm_to_m_;
    if (static_cast<int16_t>(RCS_NODE_CURVE) == type_curr_task) {
        radius_sign_curr_task = curr_task.cargo_size * mm_to_m_;
    } else {
        radius_sign_curr_task = 1000.0;  // PLC要求
    }
    radius_length_curr_task = std::fabs(1 / 2 * M_PI * radius_sign_curr_task);

    if (index_step_task < vec_step_task_stru_global_.size() - 1) {  // 当前任务非最后一个任务
        RCSNodeStru next_task = vec_step_task_stru_global_[index_step_task + 1];
        type_next_task = next_task.type;
        speed_next_task = next_task.target_vel * mm_to_m_;
        if (static_cast<int16_t>(RCS_NODE_CURVE) == type_next_task) {
            radius_sign_next_task = next_task.cargo_size * mm_to_m_;
        } else {
            radius_sign_next_task = 0;
        }
    } else {  // 当前任务为最后一个任务
        type_next_task = RCS_NODE_NONE;
        speed_next_task = 0;
        radius_sign_next_task = 0;
    }
}

void TaskManager::GetDisData(float& dis_to_stop_node, float& dis_to_type_switch_node, double delta_x)
{
    int16_t type_current_task = vec_step_task_stru_global_[index_current_step_task_].type;
    int16_t map_id_current_task = vec_step_task_stru_global_[index_current_step_task_].map_id;
    if (CheckTaskIsStopType(type_current_task, map_id_current_task)) {
        dis_to_stop_node = 0;
        dis_to_type_switch_node = 0;
        return;
    }
    dis_to_stop_node += -delta_x;
    dis_to_type_switch_node += -delta_x;
    for (size_t i = index_current_step_task_ + 1; i < vec_step_task_stru_global_.size(); ++i) {
        if (vec_step_task_stru_global_[i].type == type_current_task) {
            dis_to_type_switch_node += mm_to_m_ * vec_step_task_stru_global_[i].length;
        } else {
            break;
        }
    }

    for (size_t j = index_current_step_task_ + 1; j < vec_step_task_stru_global_.size(); ++j) {
        if (!CheckTaskIsStopType(vec_step_task_stru_global_[j].type, vec_step_task_stru_global_[j].map_id)) {
            dis_to_stop_node += mm_to_m_ * vec_step_task_stru_global_[j].length;
        } else {
            break;
        }
    }
}

double TaskManager::QuaternionToYaw(const geometry_msgs::Quaternion& q)
{
    tf::Quaternion quat(q.x, q.y, q.z, q.w);  // 使用 tf 库将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;  // 单位：弧度
}

void TaskManager::GetHeadTailPose(const geometry_msgs::Pose& pose_current, const double yaw,
                                  geometry_msgs::Point32& head_pose, geometry_msgs::Point32& tail_pose)
{
    // 车头沿着机器人的朝向方向
    head_pose.x = pose_current.position.x + distance_head_ * cos(yaw);
    head_pose.y = pose_current.position.y + distance_head_ * sin(yaw);
    head_pose.z = pose_current.position.z;
    // 车尾沿着机器人的朝向反方向
    tail_pose.x = pose_current.position.x - distance_tail_ * cos(yaw);
    tail_pose.y = pose_current.position.y - distance_tail_ * sin(yaw);
    tail_pose.z = pose_current.position.z;
}

double TaskManager::CalculateDistance(const geometry_msgs::Point32& point1, const geometry_msgs::Point32& point2)
{
    double dx = point1.x - point2.x;
    double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::Point32 TaskManager::FindClosestPoint(const geometry_msgs::Point32& target_point,
                                                     const std::vector<geometry_msgs::Point32>& points_step_task)
{
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::Point32 closest_point;
    for (const auto& path_point : points_step_task) {  // 遍历路径上的每一个点，找到距离最小的点
        double distance = CalculateDistance(path_point, target_point);
        if (distance < min_distance) {
            min_distance = distance;
            closest_point = path_point;
        }
    }
    return closest_point;
}

void TaskManager::UpdataLookAheadDis(const nav_msgs::Odometry& pose_current)
{
    distance_lookahead_ = min_dis_ahead_point_ + scale_ahead_point_ * g_vel_linear_;
}

void TaskManager::ShowPoint(const geometry_msgs::Point32& position, const float scale_marker, ros::Publisher& publisher)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_point";                          // 命名空间
    marker.id = id_marker_++;                          // 唯一标识
    marker.type = visualization_msgs::Marker::SPHERE;  // 显示球体
    marker.action = visualization_msgs::Marker::ADD;   // 添加 Marker
    // 位置
    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
    marker.pose.orientation.w = 1.0;
    // 大小
    marker.scale.x = scale_marker;  // 球体直径
    marker.scale.y = scale_marker;
    marker.scale.z = scale_marker;
    // 颜色
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // 不透明
    publisher.publish(marker);
    // ROS_INFO(" Visualized ahead point at: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
}

void TaskManager::DynamicReconfigureCallback(task_manager::task_manager_paramsConfig& config, uint32_t level)
{
    params_cfg_ = config;
    ROS_INFO("Update params");
}

void TaskManager::FillCommonNavData(common_msgs::nav_data& data_nav)
{
    data_nav.id_task = id_curr_task_;
    data_nav.type_current_task = type_curr_task_;
    data_nav.max_speed_current_task = speed_curr_task_;
    data_nav.radius_turn = radius_sign_curr_task_;
    data_nav.length_turn = radius_length_curr_task;
    data_nav.type_next_task = type_next_task_;
    data_nav.max_speed_next_task = speed_next_task_;
    data_nav.radius_sign_next_task = radius_sign_next_task_;
    data_nav.error_code = error_code_;
    data_nav.delta_center = 0;
    data_nav.delta_head = 0;
    data_nav.delta_tail = 0;
    data_nav.delta_angle = 0;
}

// 计算机器人中心、车头、车尾相对于直线[A点 -> B点]的偏差
void TaskManager::CalculateLinePathDeviation(const geometry_msgs::Point& point_start,
                                             const geometry_msgs::Point& point_end,
                                             const geometry_msgs::Pose& pose_current, PathDeviation& path_deviation)
{
    const double dx = point_end.x - point_start.x;
    const double dy = point_end.y - point_start.y;
    const double L = std::hypot(dx, dy);
    if (L < 1e-6) {
        path_deviation = {0, 0, 0, 0, 0};
        return;
    }
    const double px = pose_current.position.x;
    const double py = pose_current.position.y;
    // 横向偏差（沿路径方向）: 横向偏差的定义是基于路径方向的投影距离
    const double apx = px - point_start.x;
    const double apy = py - point_start.y;
    const double s = (apx * dx + apy * dy) / (L * L);
    path_deviation.delta_x = (s - 1.0) * L;
    // 只要横向x的偏差
    // 纵向偏差公式为左手坐标系
    path_deviation.delta_y = (apy * dx - apx * dy) / L;
    /*----- 角度偏差: 从路径朝向转向agv朝向，逆时针为正，顺时针为负 -----*/
    const double theta_path = std::atan2(dy, dx);
    const double theta_agv = QuaternionToYaw(pose_current.orientation);
    path_deviation.delta_theta = angles::normalize_angle(theta_agv - theta_path);
    /*----- 车体投影计算 -----*/
    const double agv_cos = std::cos(theta_agv);
    const double agv_sin = std::sin(theta_agv);
    // 车头坐标
    geometry_msgs::Point front_point;
    front_point.x = px + distance_head_ * agv_cos;
    front_point.y = py + distance_head_ * agv_sin;
    // 车尾坐标
    geometry_msgs::Point rear_point;
    rear_point.x = px - distance_tail_ * agv_cos;
    rear_point.y = py - distance_tail_ * agv_sin;
    // 车头投影
    const double front_apx = front_point.x - point_start.x;
    const double front_apy = front_point.y - point_start.y;
    path_deviation.front_projection = (front_apy * dx - front_apx * dy) / L;
    // 车尾投影
    const double rear_apx = rear_point.x - point_start.x;
    const double rear_apy = rear_point.y - point_start.y;
    path_deviation.rear_projection = (rear_apy * dx - rear_apx * dy) / L;
}

// 计算机器人实时位姿与1/4圆弧的偏差
void TaskManager::CalculateCurvePathDeviation(const geometry_msgs::Point& center,
                                              const geometry_msgs::Point& point_start,
                                              const geometry_msgs::Point& point_end,
                                              const geometry_msgs::Pose& pose_current, const float radius,
                                              PathDeviation& path_deviation)
{
    double robot_x = pose_current.position.x;
    double robot_y = pose_current.position.y;
    // 计算机器人到圆心的向量
    double cr_x = robot_x - center.x;
    double cr_y = robot_y - center.y;
    double r_robot = std::sqrt(cr_x * cr_x + cr_y * cr_y);  // 机器人到圆心的距离
                                                            // 计算机器人相对圆心的角度
    double theta_robot = std::atan2(cr_y, cr_x);
    // 计算径向偏差（带正负）
    double delta_y = r_robot - std::fabs(radius);
    if (radius < 0) {
        delta_y *= -1.0;
    }
    path_deviation.delta_y = delta_y;
    // 计算偏差 delta_x
    // 计算圆弧起点以及终点的角度
    double theta_start = std::atan2(point_start.y - center.y, point_start.x - center.x);
    double theta_end = std::atan2(point_end.y - center.y, point_end.x - center.x);
    // 标准化角度
    theta_robot = angles::normalize_angle(theta_robot);
    theta_start = angles::normalize_angle(theta_start);
    theta_end = angles::normalize_angle(theta_end);
    // 判断机器人是否超过圆弧终点
    double angle_min_diff = angles::shortest_angular_distance(theta_robot, theta_end);
    bool is_past_endpoint = false;
    if (radius * angle_min_diff > 0) {
        is_past_endpoint = true;
    }
    // 计算横向偏差，基于弧长差值（带符号）
    double arc_length = std::fabs(radius * angle_min_diff);
    path_deviation.delta_x = is_past_endpoint ? arc_length : -arc_length;

    // 计算航向角（从四元数转换）
    const double robot_theta = QuaternionToYaw(pose_current.orientation);
    // 计算圆弧切线方向（取决于圆弧方向）
    double theta_tangent = theta_robot - M_PI_2;  // 顺时针圆弧
    if (radius < 0) {
        theta_tangent = theta_robot + M_PI_2;  // 逆时针圆弧
    }
    /*----- 角度偏差: 从路径朝向转向agv朝向，逆时针为正，顺时针为负 -----*/
    path_deviation.delta_theta = angles::normalize_angle(robot_theta - theta_tangent);

    // === 计算车头与车尾的偏差 ===
    // 车头中心坐标
    double front_x = robot_x + distance_head_ * cos(robot_theta);
    double front_y = robot_y + distance_head_ * sin(robot_theta);
    // 车尾中心坐标
    double rear_x = robot_x - distance_tail_ * cos(robot_theta);
    double rear_y = robot_y - distance_tail_ * sin(robot_theta);
    // 计算车头到圆心的偏差
    double cf_x = front_x - center.x;
    double cf_y = front_y - center.y;
    double r_front = std::sqrt(cf_x * cf_x + cf_y * cf_y);
    path_deviation.front_projection = r_front - std::fabs(radius);
    // 计算车尾到圆心的偏差
    double c_rr_x = rear_x - center.x;
    double c_rr_y = rear_y - center.y;
    double r_rear = std::sqrt(c_rr_x * c_rr_x + c_rr_y * c_rr_y);
    path_deviation.rear_projection = r_rear - std::fabs(radius);
}

void TaskManager::SetErrorCode(const common_header::IPC_ERROR_CODE error_code)
{
    char index = static_cast<char>(error_code);
    (error_code_) |= (1 << (index));
}

void TaskManager::ClearErrorCode(const common_header::IPC_ERROR_CODE error_code)
{
    char index = static_cast<char>(error_code);
    (error_code_) &= ~(1 << (index));
}

void TaskManager::PubActionData(const ACTUATOR_CMD_ENUM cmd_action, const int32_t value, const int32_t id_action)
{
    common_msgs::action_data data_action;
    common_msgs::one_action_task one_action;
    one_action.action_cmd = static_cast<int32_t>(cmd_action);
    one_action.action_value = static_cast<int32_t>(value);
    one_action.action_id = id_action;

    data_action.type_execute_action = 0;
    data_action.action_tasks.push_back(one_action);
    pub_action_data_.publish(data_action);
}

void TaskManager::GenerateAndPubNoTaskData()
{
    common_msgs::nav_data data_nav;
    data_nav.point_current = pose_current_;
    data_nav.dis_remain_curr_node = 0;
    data_nav.dis_to_stop_node = 0;
    data_nav.dis_to_type_switch_node = 0;
    data_nav.angle_diff_spin_around = 0;
    data_nav.yaw_current_pose = static_cast<float>(yaw_current_);
    data_nav.flag_pick_task_valid = 0;
    data_nav.ctrl_charge = static_cast<int8_t>(CHARGE_CMD_STOP);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    data_nav.cmd_vel = cmd_vel;
    data_nav.x_manual_control = x_manual_control_;
    data_nav.w_manual_control = w_manual_control_;
    data_nav.ctrl_mode = static_cast<int8_t>(ctrl_mode_);
    data_nav.id_task = 0;
    data_nav.type_current_task = 0;
    data_nav.max_speed_current_task = 0;
    data_nav.radius_turn = 0;
    data_nav.length_turn = 0;
    data_nav.type_next_task = 0;
    data_nav.max_speed_next_task = 0;
    data_nav.radius_sign_next_task = 0;
    data_nav.delta_center = 0;
    data_nav.delta_head = 0;
    data_nav.delta_tail = 0;
    data_nav.delta_angle = 0;
    pub_nav_data_.publish(data_nav);

    PubActionData(ACTUATOR_CMD_STOP, 0, 0);
}

void TaskManager::GenerateAndPubFollowPathData()
{
    common_msgs::nav_data data_nav;
    geometry_msgs::Point32 point_lookahead;
    GetLookaheadPoint(point_lookahead, vec_path_point_global_[index_current_step_task_]);
    data_nav.point_lookahead = point_lookahead;
    data_nav.point_current = pose_current_;
    data_nav.angle_diff_spin_around = 0;
    data_nav.yaw_current_pose = static_cast<float>(yaw_current_);
    if (static_cast<int16_t>(RCS_NODE_LINE) == type_curr_task_) {
        CalculateLinePathDeviation(start_point_curr_task_, end_point_curr_task_, pose_current_, path_deviation_);
    } else {  // 只剩下弯道任务
        // sign_radius > 0 表示顺时针圆弧, < 0 表示逆时针圆弧
        double sign_radius = (radius_sign_curr_task_ / speed_curr_task_ < 0) ? std::fabs(radius_sign_curr_task_)
                                                                             : -std::fabs(radius_sign_curr_task_);
        CalculateCurvePathDeviation(center_point_curr_task_, start_point_curr_task_, end_point_curr_task_,
                                    pose_current_, sign_radius, path_deviation_);
    }
    if (std::fabs(path_deviation_.delta_y) > dis_derail_) {
        SetErrorCode(common_header::ERROR_NAVI_DERAILMENT);
    } else {
        ClearErrorCode(common_header::ERROR_NAVI_DERAILMENT);
    }
    FillCommonNavData(data_nav);
    float dis_to_stop_node = 0;
    float dis_to_type_switch_node = 0;
    GetDisData(dis_to_stop_node, dis_to_type_switch_node, path_deviation_.delta_x);
    data_nav.dis_remain_curr_node = -path_deviation_.delta_x;
    data_nav.dis_to_stop_node = dis_to_stop_node;
    data_nav.dis_to_type_switch_node = dis_to_type_switch_node;

    data_nav.delta_center = path_deviation_.delta_y;
    data_nav.delta_head = path_deviation_.front_projection;
    data_nav.delta_tail = path_deviation_.rear_projection;
    data_nav.delta_angle = angles::to_degrees(path_deviation_.delta_theta);

    geometry_msgs::Point32 head_pose, tail_pose, current_pose;
    geometry_msgs::Point32 base_project_pose, head_project_pose, tail_project_pose;
    GetHeadTailPose(pose_current_, yaw_current_, head_pose, tail_pose);  // 计算车头和车尾的位姿

    current_pose.x = pose_current_.position.x;
    current_pose.y = pose_current_.position.y;
    base_project_pose = FindClosestPoint(current_pose, vec_path_point_global_[index_current_step_task_]);
    head_project_pose = FindClosestPoint(head_pose, vec_path_point_global_[index_current_step_task_]);
    tail_project_pose = FindClosestPoint(tail_pose, vec_path_point_global_[index_current_step_task_]);

    data_nav.project_point_base = base_project_pose;
    data_nav.project_point_head = head_project_pose;
    data_nav.project_point_tail = tail_project_pose;
    data_nav.flag_pick_task_valid = 0;
    data_nav.ctrl_charge = static_cast<int8_t>(CHARGE_CMD_STOP);
    geometry_msgs::Twist cmd_vel;
    /*
    if (flag_enable_mpc_) {
        if (mpc_planner_->ComputeVelocityCommands(twist_current_, pose_current_,
                                                  cmd_vel)) {  // 计算失败，cmd_vel 是充填0
            data_nav.cmd_vel = cmd_vel;
        }
        pub_vel_.publish(cmd_vel);
    }
    */
    pub_nav_data_.publish(data_nav);
    PubActionData(ACTUATOR_CMD_STOP, 0, 0);

    ShowPoint(point_lookahead, scale_ahead_point_, pub_ahead_point_);
    if (flag_show_project_point_) {
        ShowPoint(base_project_pose, scale_project_point_, pub_base_project_point_);
        ShowPoint(head_project_pose, scale_project_point_, pub_head_project_point_);
        ShowPoint(tail_project_pose, scale_project_point_, pub_tail_project_point_);
    }
}

void TaskManager::GenerateAndPubAroundSpinData(const double angle_target)
{
    common_msgs::nav_data data_nav;
    float dis_to_stop_node = 0;
    float dis_to_type_switch_node = 0;
    GetDisData(dis_to_stop_node, dis_to_type_switch_node);
    data_nav.point_current = pose_current_;
    data_nav.dis_remain_curr_node = 0;
    data_nav.dis_to_stop_node = dis_to_stop_node;
    data_nav.dis_to_type_switch_node = dis_to_type_switch_node;
    data_nav.angle_diff_spin_around = NormalizeAngle(NormalizeAngle(angle_target) - NormalizeAngle(yaw_current_));
    data_nav.yaw_current_pose = static_cast<float>(yaw_current_);
    data_nav.flag_pick_task_valid = 0;
    data_nav.ctrl_charge = static_cast<int8_t>(CHARGE_CMD_STOP);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    data_nav.cmd_vel = cmd_vel;
    FillCommonNavData(data_nav);
    pub_nav_data_.publish(data_nav);

    PubActionData(ACTUATOR_CMD_STOP, 0, 0);
}

void TaskManager::GenerateAndPubWorkData(const int16_t static_work)
{
    common_msgs::nav_data data_nav;
    data_nav.point_current = pose_current_;
    data_nav.dis_remain_curr_node = 0;
    data_nav.dis_to_stop_node = 0;
    data_nav.dis_to_type_switch_node = 0;
    data_nav.angle_diff_spin_around = 0;
    data_nav.yaw_current_pose = static_cast<float>(yaw_current_);
    data_nav.flag_pick_task_valid = 0;
    data_nav.ctrl_charge = static_cast<int8_t>(CHARGE_CMD_STOP);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    data_nav.cmd_vel = cmd_vel;
    FillCommonNavData(data_nav);
    pub_nav_data_.publish(data_nav);

    // 随便给PLC一个上升或者下降指令，PLC自己根据叉尺实时高度判断是货叉上升还是下降
    PubActionData(ACTUATOR_CMD_FORK_DOWN, static_work, id_action_current_);
}

void TaskManager::GenerateAndPubPickCargoData()
{
    common_msgs::nav_data data_nav;
    data_nav.dis_remain_curr_node = 0;
    data_nav.point_current = pose_current_;
    data_nav.dis_to_stop_node = dis_fix_;  // 取货时托盘位置不固定，切换距离以及停止距离给固定值驱动PLC即可
    data_nav.dis_to_type_switch_node = dis_fix_;
    data_nav.angle_diff_spin_around = 0;
    data_nav.yaw_current_pose = static_cast<float>(yaw_current_);
    data_nav.flag_pick_task_valid = 1;
    data_nav.ctrl_charge = static_cast<int8_t>(CHARGE_CMD_STOP);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    data_nav.cmd_vel = cmd_vel;

    pallet_idenfity::PalletPose data_pallet_identify;
    bool ret = flag_enable_pallet_identify_ && p_pallet_identify_->GetPalletIdentifyData(data_pallet_identify);
    if (ret) {
        data_nav.flag_detect_pallet = 1;
        data_nav.x_detect = data_pallet_identify.x;
        data_nav.y_detect = data_pallet_identify.y;
        data_nav.theta_detect = data_pallet_identify.theta * 10.0;  // 放大10倍即0.1度作为单位给PLC
    } else {
        data_nav.flag_detect_pallet = 2;
        data_nav.x_detect = 0;
        data_nav.y_detect = 0;
        data_nav.theta_detect = 0;
    }
    FillCommonNavData(data_nav);
    pub_nav_data_.publish(data_nav);
    PubActionData(ACTUATOR_CMD_STOP, 0, 0);
}

void TaskManager::GenerateChargeData()
{
    common_msgs::nav_data data_nav;
    data_nav.point_current = pose_current_;
    data_nav.dis_remain_curr_node = 0;
    data_nav.dis_to_stop_node = 0;
    data_nav.dis_to_type_switch_node = 0;
    data_nav.angle_diff_spin_around = 0;
    data_nav.yaw_current_pose = static_cast<float>(yaw_current_);
    data_nav.flag_pick_task_valid = 0;
    data_nav.ctrl_charge = static_cast<int8_t>(CHARGE_CMD_GOING);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    data_nav.cmd_vel = cmd_vel;
    FillCommonNavData(data_nav);
    pub_nav_data_.publish(data_nav);
    PubActionData(ACTUATOR_CMD_STOP, 0, 0);
}

void TaskManager::InitTaskData()
{
    compelete_percent_ = 0;
    index_current_step_task_ = 0;
    RCSNodeStru cur_step_task = vec_step_task_stru_global_[index_current_step_task_];
    start_point_curr_task_.x = cur_step_task.start_pt_x * mm_to_m_;
    start_point_curr_task_.y = cur_step_task.start_pt_y * mm_to_m_;
    center_point_curr_task_.x = cur_step_task.center_pt_x * mm_to_m_;
    center_point_curr_task_.y = cur_step_task.center_pt_y * mm_to_m_;
    end_point_curr_task_.x = cur_step_task.end_pt_x * mm_to_m_;
    end_point_curr_task_.y = cur_step_task.end_pt_y * mm_to_m_;

    GetCurrAndNextTaskInfo(index_current_step_task_, id_curr_task_, type_curr_task_, speed_curr_task_,
                           radius_sign_curr_task_, radius_length_curr_task, type_next_task_, speed_next_task_,
                           radius_sign_next_task_);
    FillStepTaskInfo(cur_step_task, (uint16_t*)(&data_res_ipc_.node_info_1));

    // if (flag_enable_mpc_) {
    //     mpc_planner_->initialize();
    //     mpc_planner_->setPlan(vec_path_point_global_);
    // }
}

void TaskManager::ExecuteTask()
{
    RCSNodeStru cur_step_task = vec_step_task_stru_global_[index_current_step_task_];
    switch (static_cast<RCSNodeTypeEnum>(cur_step_task.type)) {
        case RCS_NODE_LINE: {
            task_type_curr_ = IPC_FOLLOW_PATH;
            GenerateAndPubFollowPathData();
            break;
        }
        case RCS_NODE_CURVE: {
            if (cur_step_task.map_id) {  // 原地自旋
                task_type_curr_ = IPC_AROUND_SPIN;
                double angle_target = NormalizeAngle(cur_step_task.heading / 180.0 * M_PI);  // RCS下发目标角度单位为度
                GenerateAndPubAroundSpinData(angle_target);
            } else {
                task_type_curr_ = IPC_FOLLOW_PATH;
                GenerateAndPubFollowPathData();
            }
            break;
        }
        case RCS_NODE_WORK: {
            task_type_curr_ = IPC_WORK;
            GenerateAndPubWorkData(cur_step_task.static_work);
            break;
        }
        case RCS_NODE_AUTORUN_BY_CAMERA: {  // 去取货
            task_type_curr_ = IPC_PICK_CARGO;
            GenerateAndPubPickCargoData();
            break;
        }
        case RCS_NODE_CHARGE: {
            task_type_curr_ = IPC_CHARGE;
            GenerateChargeData();
            break;
        }
        case RCS_NODE_TASK_FINISH: {
            task_type_curr_ = IPC_TASK_FINISH;
            GenerateAndPubNoTaskData();
            break;
        }
        default:
            break;
    }

    UpdataCurrentStepTaskIndex();

    static int count_send_path = 0;
    count_send_path++;
    if (count_send_path > 500) {
        ShowGlobalDensePath(vec_path_point_global_);  // 定时发布全局轨迹
        count_send_path = 0;
    }
}

void TaskManager::RunCycle()
{
    PubIpcResData();
    switch (status_ipc_) {
        case STATUS_NONE: {
            status_ipc_ = STATUS_NO_TASK;
            break;
        }
        case STATUS_NO_TASK: {
            if (common_header::TASK_CMD_RECV == cmd_rcs_) {
                vec_step_task_stru_global_.assign(vec_step_task_stru_.begin(), vec_step_task_stru_.end());
                if (!CheckRcsPathTaskValidity(vec_step_task_stru_global_)) {
                    data_res_ipc_.tr_check_state = 3;  // RCS 下发的路线检查失败
                    // common_header::ErrorManager::GetInstance().AddError(common_header::ERROR_RCS_PATH_CHECK_FAILED);
                } else {
                    data_res_ipc_.tr_check_state = 2;
                }
                vec_path_point_global_.clear();
                GenerateGlobalDensePath(vec_step_task_stru_global_, vec_path_point_global_);
                ShowRcsTaskPoints(vec_step_task_stru_global_);
                ShowGlobalDensePath(vec_path_point_global_);
                status_ipc_ = STATUS_RECVING_TASK;
                ROS_ERROR(" received rcs cmd:  TASK_CMD_RECV ");
            } else {
                GenerateAndPubNoTaskData();
            }
            break;
        }
        case STATUS_RECVING_TASK: {
            if (common_header::TASK_CMD_EXECUTE == cmd_rcs_) {
                status_ipc_ = STATUS_EXECUTING_TASK;
                InitTaskData();
                ROS_ERROR(" received rcs cmd:  TASK_CMD_EXECUTE ");
            }
            break;
        }
        case STATUS_EXECUTING_TASK: {
            ExecuteTask();
            if (common_header::TASK_CMD_PAUSE == cmd_rcs_) {
                status_ipc_ = STATUS_PAUSE_TASK_BY_RCS;
                ROS_ERROR(" received rcs cmd:  TASK_CMD_PAUSE ");
            }
            break;
        }
        case STATUS_COMPLETED_TASK: {
            // ClearTaskData();
            // 当任务完成后，上报RCS数据不用改变，需要保持上报任务完成状态
            status_ipc_ = STATUS_NO_TASK;  // 当任务都完成后，自动转到无任务状态 STATUS_NO_TASK
            break;
        }
        case STATUS_PAUSE_TASK_BY_IPC: {
            break;
        }
        case STATUS_PAUSE_TASK_BY_RCS: {
            if (common_header::TASK_CMD_OBORT == cmd_rcs_) {
                status_ipc_ = STATUS_ABORT_TASK;
                ROS_ERROR(" received rcs cmd:  TASK_CMD_OBORT ");
            }
            break;
        }
        case STATUS_ABORT_TASK: {
            status_ipc_ = STATUS_NO_TASK;  // 当任务被终止后，自动转到无任务状态 STATUS_NO_TASK
            break;
        }
        default: {
            ROS_ERROR("tm_err_1: do not exist this status");
            break;
        }
    }
}

std::string TaskManager::IpcStatusToString(const IpcStatusEnum status)
{
    switch (status) {
        case STATUS_NONE:
            return "STATUS_NONE";
        case STATUS_NO_TASK:
            return "STATUS_NO_TASK";
        case STATUS_RECVING_TASK:
            return "STATUS_RECVING_TASK";
        case STATUS_EXECUTING_TASK:
            return "STATUS_EXECUTING_TASK";
        case STATUS_COMPLETED_TASK:
            return "STATUS_COMPLETED_TASK";
        case STATUS_PAUSE_TASK_BY_IPC:
            return "STATUS_PAUSE_TASK_BY_IPC";
        case STATUS_PAUSE_TASK_BY_RCS:
            return "STATUS_PAUSE_TASK_BY_RCS";
        case STATUS_ABORT_TASK:
            return "STATUS_ABORT_TASK";
        default:
            return "UNKNOWN_STATUS";
    }
}

void TaskManager::TaskProcess(const common_msgs::rcs_cmd_data& data_task)
{
    ROS_ERROR("tm_step_0: received one rcs cmd ...... ");
    cmd_rcs_ = static_cast<common_header::TaskCmdEnum>(data_task.cmd);
    data_res_ipc_.tr_cmd = data_task.cmd;
    data_res_ipc_.tr_id_index = data_task.id_index;
    data_res_ipc_.tr_id_hour = data_task.id_hour;
    data_res_ipc_.tr_id_day = data_task.id_day;
    data_res_ipc_.tr_id_month = data_task.id_month;
    data_res_ipc_.tr_year = data_task.year;
    data_res_ipc_.tr_sender = data_task.sender;
    data_res_ipc_.tr_type = data_task.type;
    data_res_ipc_.tr_step_size = data_task.step_size;
    data_res_ipc_.tr_start_delay = data_task.start_delay;

    std::vector<uint16_t> step_task;
    step_task.clear();
    for (size_t i = 0; i < data_task.data_step_tasks.size(); ++i) {
        step_task.push_back(data_task.data_step_tasks[i]);
    }
    ROS_INFO("tm_step_1: task_info: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]", data_res_ipc_.tr_cmd,
             data_res_ipc_.tr_id_index, data_res_ipc_.tr_id_hour, data_res_ipc_.tr_id_day, data_res_ipc_.tr_id_month,
             data_res_ipc_.tr_year, data_res_ipc_.tr_sender, data_res_ipc_.tr_type, data_res_ipc_.tr_step_size,
             data_res_ipc_.tr_start_delay);

    const int size_one_node = static_cast<int>(sizeof(RCSLineNodeStru));
    if (step_task.size() < data_res_ipc_.tr_step_size * size_one_node / 2) {
        ROS_ERROR("tm_err_4: step_task size not match tr_step_size: %d, %d", step_task.size(),
                  data_res_ipc_.tr_step_size * size_one_node / 2);
        return;
    }

    vec_step_task_stru_.clear();
    vec_step_task_stru_.shrink_to_fit();
    RCSNodeStru one_node;
    for (size_t i = 0; i < data_res_ipc_.tr_step_size; ++i) {
        int addr = static_cast<int>(i * size_one_node) / 2;
        if (!ReadNodeFromRegister(step_task, addr, one_node)) {
            ROS_ERROR("tm_err_3: ReadNodeFromRegister failed ...... ");
        } else {
            ROS_INFO(
                "tm_step_10: node[%d]: type: %d, cmd: %d, tar_vel: %d, heading: %d, len: %d,"
                " static_work: %d, id: %d >> %d, start_to_end: [%d, %d] >> [%d, %d], center: [%d, %d],"
                " is_load: %d, operator_id: %d, position: [%d, %d, %d], obj_speed: %d,"
                " wait_time: %d, charge_mode: %d, charge_limit: %d,"
                " cargo_size: %d, stop_before_done: %d, map_id: %d, standby_wait_time: %d, task_id_index: %d",
                i, one_node.type, one_node.cmd, one_node.target_vel, one_node.heading, one_node.length,
                one_node.static_work, one_node.id_a, one_node.id_b, one_node.start_pt_x, one_node.start_pt_y,
                one_node.end_pt_x, one_node.end_pt_y, one_node.center_pt_x, one_node.center_pt_y,
                one_node.is_loading_cargo, one_node.operator_id, one_node.position1, one_node.position2,
                one_node.position3, one_node.obj_speed, one_node.done_wait_time, one_node.charge_mode,
                one_node.charge_limit, one_node.cargo_size, one_node.stop_before_done, one_node.map_id,
                one_node.WaitTime, one_node.task_id_index);
        }
        vec_step_task_stru_.emplace_back(one_node);
    }
    ROS_INFO("tm_step_11: step_size: %d, vec_step_task_stru_ size: %d, vector size of step_task: %d",
             data_res_ipc_.tr_step_size, vec_step_task_stru_.size(), step_task.size());
}

void TaskManager::ProcessPlcResData(const common_msgs::plc_res_nav_data& data_res)
{
    g_capacity_remain_ = data_res.capacity_remain;
    g_vel_linear_ = data_res.vel_linear;
    g_status_sensor_signal_ = data_res.status_sensor_signal;
    g_status_stop_ = data_res.status_stop;
    g_status_charge_ = data_res.status_charge;
    g_id_stop_node_ = data_res.id_stop_node;
}

void TaskManager::ProcessPlcResActionData(const common_msgs::plc_res_action_data& data_res_action)
{
    g_status_action_1 = data_res_action.status_action;
    g_error_code_action_1 = data_res_action.error_code_action;
    g_value_action_1 = data_res_action.value_action;
    g_id_finish_action_1 = data_res_action.id_finish_action;

    // ROS_INFO("tm_step_18: rec action res is: [%d, %d, %d, %d]", g_status_action_1, g_error_code_action_1,
    // g_value_action_1, g_id_finish_action_1);
}

// 计算圆弧任务的转弯半径, 正为顺时针圆弧，负为逆时针圆弧
void TaskManager::CalCurveSignRadius(const int16_t heading, const int32_t start_x, const int32_t start_y,
                                     const int32_t end_x, const int32_t end_y, const int32_t center_x,
                                     const int32_t center_y, double& radius_sign)
{
    double center_xx = mm_to_m_ * center_x;
    double center_yy = mm_to_m_ * center_y;
    double start_xx = mm_to_m_ * start_x;
    double start_yy = mm_to_m_ * start_y;
    double end_xx = mm_to_m_ * end_x;
    double end_yy = mm_to_m_ * end_y;
    double mutiple = (end_yy - center_yy) * (start_xx - center_xx) - (end_xx - center_xx) * (start_yy - center_yy);
    double sign_w = (mutiple > 0) ? 1.0 : -1.0;  // 左转的话w是正值, 右转的话w是负值
    double sign_v = (-2 == heading) ? -1.0 : 1.0;
    // v 跟 w 都带正负号，体现在转弯半径上
    double radius = std::hypot(center_xx - start_xx, center_yy - start_yy);
    radius_sign = radius * (sign_v / sign_w);
    ROS_INFO(
        "curve_info: start_point:[%f, %f], end_point:[%f, %f], center_point:[%f, %f], sign_w: %f, sign_v: "
        "%f, radius_sign: %f",
        start_xx, start_yy, end_xx, end_yy, center_xx, center_yy, sign_w, sign_v, radius_sign);
}

bool TaskManager::ReadNodeFromRegister(const std::vector<uint16_t>& one_step_task, const int addr,
                                       RCSNodeStru& rcs_node)
{
    int index = addr;
    switch ((RCSNodeTypeEnum)one_step_task[index]) {
        case RCS_NODE_LINE: {
            ROS_INFO("tm_step_2: RCS_NODE_LINE: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSLineNodeStru rcs_line_node;
            rcs_line_node.type = one_step_task[index++];
            rcs_line_node.target_vel = one_step_task[index++];
            rcs_line_node.heading = one_step_task[index++];
            if (-2 == rcs_line_node.heading) {
                rcs_line_node.target_vel *= -1.0;
            }
            rcs_line_node.length = one_step_task[index++];
            rcs_line_node.cargo_size = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            rcs_line_node.start_pt_x = temp_start_x;
            rcs_line_node.start_pt_y = temp_start_y;
            rcs_line_node.end_pt_x = temp_end_x;
            rcs_line_node.end_pt_y = temp_end_y;
            index += 4;
            rcs_line_node.map_id = one_step_task[index++];
            rcs_line_node.id_a = one_step_task[index++];
            rcs_line_node.id_b = one_step_task[index++];
            rcs_node.Reset(rcs_line_node);
            break;
        }
        case RCS_NODE_AUTORUN_LINE: {
            ROS_INFO("tm_step_2: RCS_NODE_AUTORUN_LINE: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSLineNodeStru rcs_line_node;
            rcs_line_node.type = one_step_task[index++];
            rcs_line_node.target_vel = one_step_task[index++];
            rcs_line_node.heading = one_step_task[index++];
            if (-2 == rcs_line_node.heading) {
                rcs_line_node.target_vel *= -1.0;
            }
            rcs_line_node.length = one_step_task[index++];
            rcs_line_node.cargo_size = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            rcs_line_node.start_pt_x = temp_start_x;
            rcs_line_node.start_pt_y = temp_start_y;
            rcs_line_node.end_pt_x = temp_end_x;
            rcs_line_node.end_pt_y = temp_end_y;
            rcs_line_node.depth = one_step_task[index++];
            rcs_line_node.x_map_box_frame = one_step_task[index++];
            rcs_line_node.y_map_box_frame = one_step_task[index++];
            rcs_line_node.remain17 = one_step_task[index++];
            rcs_line_node.map_id = one_step_task[index++];
            rcs_line_node.id_a = one_step_task[index++];
            rcs_line_node.id_b = one_step_task[index++];
            rcs_node.Reset(rcs_line_node);
            break;
        }
        case RCS_NODE_CURVE: {
            ROS_INFO("tm_step_2: RCS_NODE_CURVE: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSCurveNodeStru tcs_curve_node;
            tcs_curve_node.type = one_step_task[index++];
            tcs_curve_node.target_vel = one_step_task[index++];
            tcs_curve_node.heading = one_step_task[index++];
            if (-2 == tcs_curve_node.heading) {
                tcs_curve_node.target_vel *= -1.0;
            }
            tcs_curve_node.length = one_step_task[index++];
            tcs_curve_node.cargo_size = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_center_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_center_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            tcs_curve_node.start_pt_x = temp_start_x;
            tcs_curve_node.start_pt_y = temp_start_y;
            tcs_curve_node.end_pt_x = temp_end_x;
            tcs_curve_node.end_pt_y = temp_end_y;
            tcs_curve_node.center_pt_x = temp_center_x;
            tcs_curve_node.center_pt_y = temp_center_y;
            // 转弯任务,复用 map_id, 1 表示原地自旋任务
            tcs_curve_node.map_id = one_step_task[index++];
            double radius_sign = 0;
            if ((temp_start_x == temp_end_x) && (temp_start_y == temp_end_y)) {
                tcs_curve_node.map_id = 1;
            } else {
                tcs_curve_node.map_id = 0;
                CalCurveSignRadius(tcs_curve_node.heading, temp_start_x, temp_start_y, temp_end_x, temp_end_y,
                                   temp_center_x, temp_center_y, radius_sign);
            }
            tcs_curve_node.cargo_size = radius_sign * m_to_mm_;  // 转弯任务 复用 cargo_size 表示带正负号的转弯半径
            tcs_curve_node.id_a = one_step_task[index++];
            tcs_curve_node.id_b = one_step_task[index++];
            rcs_node.Reset(tcs_curve_node);
            break;
        }
        case RCS_NODE_AUTORUN_CURVE: {
            ROS_INFO("tm_step_2: RCS_NODE_AUTORUN_CURVE: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSCurveNodeStru tcs_curve_node;
            tcs_curve_node.type = one_step_task[index++];
            tcs_curve_node.target_vel = one_step_task[index++];
            tcs_curve_node.heading = one_step_task[index++];
            if (-2 == tcs_curve_node.heading) {
                tcs_curve_node.target_vel *= -1.0;
            }
            tcs_curve_node.length = one_step_task[index++];
            tcs_curve_node.cargo_size = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_center_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_center_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            tcs_curve_node.start_pt_x = temp_start_x;
            tcs_curve_node.start_pt_y = temp_start_y;
            tcs_curve_node.end_pt_x = temp_end_x;
            tcs_curve_node.end_pt_y = temp_end_y;
            tcs_curve_node.center_pt_x = temp_center_x;
            tcs_curve_node.center_pt_y = temp_center_y;
            tcs_curve_node.map_id = one_step_task[index++];
            tcs_curve_node.id_a = one_step_task[index++];
            tcs_curve_node.id_b = one_step_task[index++];
            rcs_node.Reset(tcs_curve_node);
            break;
        }
        case RCS_NODE_WORK: {
            ROS_INFO("tm_step_2: RCS_NODE_WORK: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSWorkNodeStru tcs_work_node;
            tcs_work_node.type = one_step_task[index++];
            tcs_work_node.static_work = one_step_task[index++];
            tcs_work_node.target_vel = one_step_task[index++];
            tcs_work_node.stop_before_done = one_step_task[index++];
            tcs_work_node.done_wait_time = one_step_task[index++];
            tcs_work_node.cargo_size = one_step_task[index++];
            tcs_work_node.is_loading_cargo = one_step_task[index++];
            tcs_work_node.heading = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            tcs_work_node.start_pt_x = temp_start_x;
            tcs_work_node.start_pt_y = temp_start_y;
            tcs_work_node.position2 = one_step_task[index++];
            tcs_work_node.position3 = one_step_task[index++];
            index += 3;
            tcs_work_node.map_id = one_step_task[index++];
            tcs_work_node.id_a = one_step_task[index++];
            tcs_work_node.id_b = one_step_task[index++];
            rcs_node.Reset(tcs_work_node);
            break;
        }
        case RCS_NODE_CHARGE: {
            ROS_INFO("tm_step_2: RCS_NODE_CHARGE: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSChargeNodeStru tcs_charge_node;
            tcs_charge_node.type = one_step_task[index++];
            tcs_charge_node.charge_mode = one_step_task[index++];
            tcs_charge_node.charge_limit = one_step_task[index++];
            tcs_charge_node.cargo_size = one_step_task[index++];
            tcs_charge_node.heading = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            tcs_charge_node.start_pt_x = temp_start_x;
            tcs_charge_node.start_pt_y = temp_start_y;
            index += 8;
            tcs_charge_node.map_id = one_step_task[index++];
            tcs_charge_node.id_a = one_step_task[index++];
            rcs_node.Reset(tcs_charge_node);
            break;
        }
        case RCS_NODE_TRANSFER: {
            ROS_INFO("tm_step_2: RCS_NODE_TRANSFER: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSTransferNodeStru tcs_transfer_node;
            tcs_transfer_node.type = one_step_task[index++];
            tcs_transfer_node.heading = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            tcs_transfer_node.start_pt_x = temp_start_x;
            tcs_transfer_node.start_pt_y = temp_start_y;
            tcs_transfer_node.is_where = one_step_task[index++];
            tcs_transfer_node.operator_id = one_step_task[index++];
            tcs_transfer_node.position = one_step_task[index++];
            tcs_transfer_node.obj_speed = one_step_task[index++];
            tcs_transfer_node.cmd = one_step_task[index++];
            tcs_transfer_node.wait_time = one_step_task[index++];
            tcs_transfer_node.cargo_size = one_step_task[index++];
            index += 4;
            tcs_transfer_node.map_id = one_step_task[index++];
            tcs_transfer_node.id_a = one_step_task[index++];
            rcs_node.Reset(tcs_transfer_node);
            break;
        }
        case RCS_NODE_STANDBY_OPE: {
            ROS_INFO("tm_step_2: RCS_NODE_STANDBY_OPE: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSStandbyNodeStru tcs_standby_ope_node;
            tcs_standby_ope_node.type = one_step_task[index++];
            tcs_standby_ope_node.Ope_type = one_step_task[index++];
            tcs_standby_ope_node.Sen_type = one_step_task[index++];
            tcs_standby_ope_node.WaitTime = tcs_standby_ope_node.Sen_type;
            tcs_standby_ope_node.CodeID = one_step_task[index++];
            tcs_standby_ope_node.cargo_size = one_step_task[index++];
            int32_t temp_target_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_target_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            tcs_standby_ope_node.tar_point_x = temp_target_x;
            tcs_standby_ope_node.tar_point_y = temp_target_y;
            int32_t temp_target_theta = GET_INT32_FROM_INT16(one_step_task, index);
            index += 2;
            tcs_standby_ope_node.tar_point_theta = temp_target_theta;
            tcs_standby_ope_node.adj_type = one_step_task[index++];
            tcs_standby_ope_node.motion_mehtod = one_step_task[index++];
            tcs_standby_ope_node.limit_x = one_step_task[index++];
            tcs_standby_ope_node.limit_y = one_step_task[index++];
            index += 2;
            tcs_standby_ope_node.map_id = one_step_task[index++];
            tcs_standby_ope_node.id_a = one_step_task[index++];
            tcs_standby_ope_node.id_b = one_step_task[index++];
            rcs_node.Reset(tcs_standby_ope_node);
            break;
        }
        case RCS_NODE_AUTORUN_BY_CAMERA: {
            ROS_INFO("tm_step_2: RCS_NODE_AUTORUN_BY_CAMERA: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSLineNodeStru rcs_line_node;
            rcs_line_node.type = one_step_task[index++];
            rcs_line_node.target_vel = one_step_task[index++];
            rcs_line_node.heading = one_step_task[index++];
            if (-2 == rcs_line_node.heading) {
                rcs_line_node.target_vel *= -1.0;
            }
            rcs_line_node.length = one_step_task[index++];
            rcs_line_node.cargo_size = one_step_task[index++];
            int32_t temp_start_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_start_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_x = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            int32_t temp_end_y = (GET_INT32_FROM_INT16(one_step_task, index));
            index += 2;
            rcs_line_node.depth = one_step_task[index];
            rcs_line_node.start_pt_x = temp_start_x;
            rcs_line_node.start_pt_y = temp_start_y;
            rcs_line_node.end_pt_x = temp_end_x;
            rcs_line_node.end_pt_y = temp_end_y;
            index += 4;
            rcs_line_node.map_id = one_step_task[index++];
            rcs_line_node.id_a = one_step_task[index++];
            rcs_line_node.id_b = one_step_task[index++];
            rcs_node.Reset(rcs_line_node);
            break;
        }
        case RCS_NODE_TASK_FINISH: {
            ROS_INFO("tm_step_2: RCS_NODE_TASK_FINISH: %d", (RCSNodeTypeEnum)one_step_task[index]);
            RCSFinishNodeStru rcs_finish_node;
            rcs_finish_node.type = one_step_task[index++];
            index += 15;
            rcs_finish_node.task_id_index = one_step_task[index++];
            rcs_finish_node.map_id = one_step_task[index++];
            rcs_finish_node.id_a = one_step_task[index++];
            rcs_finish_node.id_b = one_step_task[index++];
            rcs_node.Reset(rcs_finish_node);
            break;
        }
        default: {
            ROS_ERROR("tm_err_2: modbus recv error node type: %d", one_step_task[addr]);
            return false;
        }
    }
    return true;
}

void TaskManager::ManualControlProcess(const geometry_msgs::Twist& vel_manual_control)
{
    if (0 == vel_manual_control.linear.x && 0 == vel_manual_control.angular.z) {
        ctrl_mode_ = CTRL_MODE_AUTO;
    } else {
        ctrl_mode_ = CTRL_MODE_MANUAL;
    }
    x_manual_control_ = vel_manual_control.linear.x;
    w_manual_control_ = vel_manual_control.angular.z;
}

void TaskManager::OdomProcess(const nav_msgs::Odometry& odom)
{
    UpdataLookAheadDis(odom);
    twist_current_ = odom.twist.twist;
}

void TaskManager::Current2dPoseProcess(const geometry_msgs::PoseStamped& pose_current)
{
    pose_current_.position.x = pose_current.pose.position.x;
    pose_current_.position.y = pose_current.pose.position.y;
    pose_current_.position.z = pose_current.pose.position.z;
    pose_current_.orientation.x = pose_current.pose.orientation.x;
    pose_current_.orientation.y = pose_current.pose.orientation.y;
    pose_current_.orientation.z = pose_current.pose.orientation.z;
    pose_current_.orientation.w = pose_current.pose.orientation.w;

    geometry_msgs::Quaternion quaternion;
    quaternion.x = pose_current_.orientation.x;
    quaternion.y = pose_current_.orientation.y;
    quaternion.z = pose_current_.orientation.z;
    quaternion.w = pose_current_.orientation.w;
    yaw_current_ = QuaternionToYaw(quaternion);
}

void TaskManager::Current3dPoseProcess(const nav_msgs::Odometry& pose_current)
{
    UpdataLookAheadDis(pose_current);
    pose_current_.position.x = pose_current.pose.pose.position.x;
    pose_current_.position.y = pose_current.pose.pose.position.y;
    pose_current_.position.z = pose_current.pose.pose.position.z;
    pose_current_.orientation.x = pose_current.pose.pose.orientation.x;
    pose_current_.orientation.y = pose_current.pose.pose.orientation.y;
    pose_current_.orientation.z = pose_current.pose.pose.orientation.z;
    pose_current_.orientation.w = pose_current.pose.pose.orientation.w;

    geometry_msgs::Quaternion quaternion;
    quaternion.x = pose_current_.orientation.x;
    quaternion.y = pose_current_.orientation.y;
    quaternion.z = pose_current_.orientation.z;
    quaternion.w = pose_current_.orientation.w;
    yaw_current_ = QuaternionToYaw(quaternion);

    twist_current_ = pose_current.twist.twist;
}

TaskManager::~TaskManager()
{
}

}  // namespace task
