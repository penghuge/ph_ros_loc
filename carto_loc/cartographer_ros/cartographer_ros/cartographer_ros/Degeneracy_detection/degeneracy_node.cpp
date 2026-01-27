#include <cmath>

#include <angles/angles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "laser_line_extraction/degeneracy_detector.h"

// Forward declare degeneracy flag setters from Cartographer core so this node
// can write the flag directly without ROS relay through cartographer_node.
namespace cartographer
{
namespace mapping
{
namespace scan_matching
{
void SetDegeneracyDetected(bool v);
bool GetDegeneracyDetected();
// Localization/odom fusion interfaces from core (only UseExternalOdom + pose setter kept)
void SetExternalOdometryPose2D(double x, double y, double theta);
bool GetUseExternalOdom();
void SetUseExternalOdom(bool v);
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

/**
 * DegeneracyNode penghu
 * - 典型使用：在 launch 中启动该节点，订阅激光话题（默认 /scan），
 *   并发布布尔话题 /degeneracy_detected 表示是否检测到退化场景。
 * - 支持通过私有参数配置检测阈值，方便在不同场景下调参。
 */
class DegeneracyNode
{
   public:
    DegeneracyNode()
    {
        // 私有命名空间读取参数，允许在 launch 中覆盖
        ros::NodeHandle pnh("~");
        std::string scan_topic;
        pnh.param<std::string>("scan_topic", scan_topic, std::string("/scan"));
        pnh.param<std::string>("odom_topic", odom_topic_, std::string("/odom"));
        pnh.param<std::string>("odom_forward_topic", odom_forward_topic_, std::string("/odom_forward"));
        pnh.param<std::string>("odom_path_topic", odom_path_topic_, std::string("/odom_forward_path"));
        pnh.param<std::string>("loc_path_topic", loc_path_topic_, std::string("/localization_path"));
        pnh.param<std::string>("frame_id", frame_id_, std::string("laser"));
        pnh.param<std::string>("odom_frame", odom_frame_, std::string("map"));
        pnh.param<std::string>("base_frame", base_frame_, std::string("base_footprint"));
        pnh.param<std::string>("loc_frame", loc_frame_, std::string("map"));
        pnh.param<int>("odom_traj_max_len", odom_traj_max_len_, 1000);
        pnh.param<int>("loc_traj_max_len", loc_traj_max_len_, 1000);
        pnh.param<double>("min_long_line_length", params_.min_long_line_length, 1.0);
        pnh.param<double>("long_line_ratio", params_.long_line_ratio, 0.6);
        pnh.param<double>("angle_consistency_thresh", params_.angle_consistency_thresh, 0.1);
        pnh.param<double>("coverage_ratio", params_.coverage_ratio, 0.5);
        pnh.param<bool>("publish_tf", publish_tf_, false);
        // 文本/标注三类 Marker 的显示位置（默认在 x 轴错开，避免重叠）
        pnh.param<double>("deg_marker_x", deg_marker_x_, 0.0);
        pnh.param<double>("deg_marker_y", deg_marker_y_, 2.0);
        pnh.param<double>("deg_marker_z", deg_marker_z_, 0.0);
        pnh.param<double>("status_marker_x", status_marker_x_, 8.0);
        pnh.param<double>("status_marker_y", status_marker_y_, 2.0);
        pnh.param<double>("status_marker_z", status_marker_z_, 0.0);
        pnh.param<double>("score_marker_x", score_marker_x_, -8.0);
        pnh.param<double>("score_marker_y", score_marker_y_, 3.0);
        pnh.param<double>("score_marker_z", score_marker_z_, 0.0);
        //使用读取的参数构造直线检测器
        detector_ = std::make_shared<laser_line_extraction::DegeneracyDetector>(params_);
        detector_->SetLidarParam(3.1415927410125732, -3.1415927410125732,
                                 0.0026179938577115536);  // 720 点  1) 构建 bearings/cos/sin/index 缓存
        detector_->SetLineExtractionFirstParam(1e-5, 0.012, 0.0001, 0.0001, 0.5, 1.0);  // 放宽参数
        detector_->SetLineExtractionSecondParam(20, 0.1, 50, 0.1, 0.1);                 // 进一步放宽参数

        //发布与订阅 里程计订阅/发布（50Hz），与激光（24Hz）并行运行
        pub_ = nh_.advertise<std_msgs::Bool>("/degeneracy_detected", 5);
        line_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("/line_markers", 1);
        deg_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/degeneracy_marker", 1);
        odom_forward_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_forward_topic_, 10);
        odom_path_pub_ = nh_.advertise<nav_msgs::Path>(odom_path_topic_, 1, true);
        odom_text_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/odom_forward_text", 1, true);
        odom_reset_flag_pub_ = nh_.advertise<std_msgs::Int32>("/odom_reset_flag", 10);
        localization_status_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/localization_status_marker", 1, true);
        localization_status_pub_ = nh_.advertise<std_msgs::String>("/localization_status", 5);
        localization_score_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/localization_score_marker", 1, true);
        loc_path_pub_ = nh_.advertise<nav_msgs::Path>(loc_path_topic_, 1, true);
        localization_text_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/localization_text", 1, true);        // 定位状态文本单独的 Marker 话题，便于在 RViz 单独订阅与放置
        use_external_odom_pub_ = nh_.advertise<std_msgs::Bool>("/carto/use_external_odom", 1, true);        // 对外广播是否使用外部里程计（供 cartographer_ros 订阅），latched 以便后到达者也能拿到最新值

        // 为不同频率/来源的回调配置独立的队列与订阅（严格解耦 24Hz/50Hz/跨进程定位话题）
        nh_laser_.setCallbackQueue(&laser_queue_);
        nh_odom_.setCallbackQueue(&odom_queue_);
        nh_loc_.setCallbackQueue(&loc_queue_);

        sub_ = nh_laser_.subscribe(scan_topic, 10, &DegeneracyNode::scanCb, this);
        odom_sub_ = nh_odom_.subscribe(odom_topic_, 1000, &DegeneracyNode::odomCb, this);
        // 订阅 cartographer_ros 输出的定位 Pose 和 Score（跨进程）绑定到独立 loc
        latest_loc_sub_ = nh_loc_.subscribe<geometry_msgs::PoseStamped>(
            "/carto_latest_localization", 10, [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                latest_loc_stamp_ = msg->header.stamp;
                latest_loc_x_ = msg->pose.position.x;
                latest_loc_y_ = msg->pose.position.y;
                latest_loc_th_ = tf::getYaw(msg->pose.orientation);
                have_latest_loc_ = true;
                if (!first_loc_logged_) {
                    first_loc_logged_ = true;
                    ROS_INFO_STREAM("[degeneracy1_node] first /carto_latest_localization t="
                                    << msg->header.stamp.toSec() << " x=" << latest_loc_x_ << " y=" << latest_loc_y_
                                    << " th=" << latest_loc_th_);
                }
                ROS_INFO_STREAM_THROTTLE(1.0, "[degeneracy1_node] recv /carto_latest_localization t="
                                                  << msg->header.stamp.toSec() << " x=" << latest_loc_x_
                                                  << " y=" << latest_loc_y_ << " th=" << latest_loc_th_);
            });
        loc_score_sub_ = nh_loc_.subscribe<std_msgs::Float32>(
            "/carto_localization_score", 10, [this](const std_msgs::Float32::ConstPtr& msg) {
                latest_loc_score_ = msg->data;
                have_latest_score_ = true;
                if (!first_score_logged_) {
                    first_score_logged_ = true;
                    ROS_INFO_STREAM("[degeneracy1_node] first /carto_localization_score score=" << latest_loc_score_);
                }
                ROS_INFO_STREAM_THROTTLE(
                    1.0, "[degeneracy1_node] recv /carto_localization_score score=" << latest_loc_score_);
            });

        // 独立线程并发处理三个回调队列
        spinner_laser_.reset(new ros::AsyncSpinner(1, &laser_queue_));
        spinner_odom_.reset(new ros::AsyncSpinner(1, &odom_queue_));
        spinner_loc_.reset(new ros::AsyncSpinner(1, &loc_queue_));
        spinner_laser_->start();
        spinner_odom_->start();
        spinner_loc_->start();
    }

    // 激光回调：调用检测器并发布结果
    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        bool deg = detector_->detect(*msg);
        // 直接写入 Cartographer 核心退化标志，避免在 cartographer_node 再订阅转发（暂时没用到，因为属于双进程）
        cartographer::mapping::scan_matching::SetDegeneracyDetected(deg);
        std_msgs::Bool out;
        out.data = deg;
        pub_.publish(out);
        // type·1 发布每帧提取到的直线/line_markers
        const auto& lines = detector_->getLines();
        visualization_msgs::Marker marker_msg;
        populateMarkerMsg(lines, marker_msg);
        line_markers_pub_.publish(marker_msg);
        // type.2 发布处理后线段到 /degeneracy_line（蓝色），与原始线区分
        const auto& plines = detector_->getProcessedLines();
        visualization_msgs::Marker processed_msg;
        processed_msg.ns = "degeneracy_processed";
        processed_msg.id = 1;
        processed_msg.type = visualization_msgs::Marker::LINE_LIST;
        processed_msg.scale.x = 0.1;
        processed_msg.pose.position.x = 0.0;
        processed_msg.pose.position.y = 0.0;
        processed_msg.pose.position.z = 0.0;
        processed_msg.pose.orientation.x = 0.0;
        processed_msg.pose.orientation.y = 0.0;
        processed_msg.pose.orientation.z = 0.0;
        processed_msg.pose.orientation.w = 1.0;
        processed_msg.color.r = 0.0;
        processed_msg.color.g = 0.0;
        processed_msg.color.b = 1.0;
        processed_msg.color.a = 1.0;
        for (const auto& l : plines) {
            geometry_msgs::Point p_start;
            p_start.x = l.start[0];
            p_start.y = l.start[1];
            p_start.z = 0;
            geometry_msgs::Point p_end;
            p_end.x = l.end[0];
            p_end.y = l.end[1];
            p_end.z = 0;
            processed_msg.points.push_back(p_start);
            processed_msg.points.push_back(p_end);
        }
        processed_msg.header.frame_id = frame_id_;
        processed_msg.header.stamp = ros::Time::now();
        static ros::Publisher processed_pub = nh_.advertise<visualization_msgs::Marker>("/degeneracy_line", 1);
        processed_pub.publish(processed_msg);
        // type.3 发布退化状态 marker（红=退化, 绿=正常），TEXT_VIEW_FACING 显示文本
        visualization_msgs::Marker deg_mark;
        deg_mark.header.frame_id = frame_id_;
        deg_mark.header.stamp = ros::Time();
        deg_mark.ns = "degeneracy_status";
        deg_mark.id = 0;
        deg_mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        deg_mark.action = visualization_msgs::Marker::ADD;
        deg_mark.pose.position.x = deg_marker_x_;
        deg_mark.pose.position.y = deg_marker_y_;
        deg_mark.pose.position.z = deg_marker_z_;  // 可配置，默认 1m
        deg_mark.pose.orientation.x = 0.0;
        deg_mark.pose.orientation.y = 0.0;
        deg_mark.pose.orientation.z = 0.0;
        deg_mark.pose.orientation.w = 1.0;
        deg_mark.scale.z = 0.8;  // 字体大小（扩大 2 倍）
        if (deg) {
            deg_mark.color.r = 1.0;
            deg_mark.color.g = 0.0;
            deg_mark.color.b = 0.0;
            deg_mark.color.a = 1.0;
            deg_mark.text = "DEGENERATE";
        } else {
            deg_mark.color.r = 0.0;
            deg_mark.color.g = 1.0;
            deg_mark.color.b = 0.0;
            deg_mark.color.a = 1.0;
            deg_mark.text = "OK";
        }
        deg_marker_pub_.publish(deg_mark);
    }

    void populateMarkerMsg(const std::vector<line_extraction::Line>& lines, visualization_msgs::Marker& marker_msg)
    {
        marker_msg.ns = "line_extraction";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::Marker::LINE_LIST;
        marker_msg.scale.x = 0.1;  // 线宽0.1米
        marker_msg.pose.position.x = 0.0;
        marker_msg.pose.position.y = 0.0;
        marker_msg.pose.position.z = 0.0;
        marker_msg.pose.orientation.x = 0.0;
        marker_msg.pose.orientation.y = 0.0;
        marker_msg.pose.orientation.z = 0.0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.color.r = 1.0;  // 纯红色; 2.0 表示纯绿; 3.0 表示纯蓝;4.0 表示纯黄; 5.0 表示纯紫; 6.0 表示纯青; 7.0 表示黑色; 8.0 表示白色; 9.0 表示橙色; 10.0 表示粉色
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 0.0;
        marker_msg.color.a = 1.0;  // 纯红，不透明, 1.0 表示不透明; 0.0 表示完全透明;
        for (std::vector<line_extraction::Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit) {
            geometry_msgs::Point p_start;
            p_start.x = cit->getStart()[0];
            p_start.y = cit->getStart()[1];
            p_start.z = 0;
            marker_msg.points.push_back(p_start);
            geometry_msgs::Point p_end;
            p_end.x = cit->getEnd()[0];
            p_end.y = cit->getEnd()[1];
            p_end.z = 0;
            marker_msg.points.push_back(p_end);
        }
        marker_msg.header.frame_id = frame_id_;
        marker_msg.header.stamp = ros::Time::now();
    }

    // 里程计回调：将 /odom 的速度积分为 /odom_forward，并发布 TF
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 回调触发心跳（1Hz），以及每帧DEBUG日志
        ROS_INFO_THROTTLE(1.0, "[degeneracy1_node] odomCb alive. last_time_set=%d x=%.3f y=%.3f th=%.3f",
                          have_last_time_, x_, y_, theta_);
        ROS_DEBUG_STREAM("[degeneracy1_node] odomCb called: stamp="
                         << msg->header.stamp.toSec() << " frame=" << msg->header.frame_id
                         << " child=" << msg->child_frame_id << " vx=" << msg->twist.twist.linear.x
                         << " vy=" << msg->twist.twist.linear.y << " wz=" << msg->twist.twist.angular.z);
        // type.1 航迹推算
        const ros::Time current_time = msg->header.stamp;
        if (!have_last_time_) {
            ROS_INFO("[degeneracy1_node] odomCb first message, align time only. stamp=%.3f", current_time.toSec());
            last_time_ = current_time;
            have_last_time_ = true;
            // 初次不积分，仅对齐时间
            return;
        }
        const double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;
        if (dt <= 0.0)
            return;
        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        const double wz = msg->twist.twist.angular.z;
        const double delta_x = vx * dt;
        const double delta_y = vy * dt;
        const double delta_theta = wz * dt;
        theta_ += delta_theta;
        // normalize theta_ to [-pi, pi]
        if (theta_ > M_PI) {
            theta_ -= 2.0 * M_PI;
        } else if (theta_ < -M_PI) {
            theta_ += 2.0 * M_PI;
        }
        x_ += delta_x * std::cos(theta_) - delta_y * std::sin(theta_);
        y_ += delta_x * std::sin(theta_) + delta_y * std::cos(theta_);
        ROS_DEBUG_STREAM("[degeneracy_node] odom integrated pose -> x=" << x_ << " y=" << y_ << " th=" << theta_);

        // type.2 里程计发布，暂时不显示在 RViz，仅供内部使用
        nav_msgs::Odometry out = *msg;
        out.header.frame_id = odom_frame_;
        out.child_frame_id = base_frame_;
        out.pose.pose.position.x = x_;
        out.pose.pose.position.y = y_;
        out.pose.pose.position.z = 0.0;
        out.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
        out.header.stamp = current_time;  // 保持与输入一致
        odom_forward_pub_.publish(out);   // 不显示

        // type.3 发布 TF: odom -> base_footprint（默认关闭）
        if (publish_tf_) {
            tf::StampedTransform transform;
            transform.setOrigin(tf::Vector3(x_, y_, 0.0));
            const tf::Quaternion q = tf::createQuaternionFromYaw(theta_);
            transform.setRotation(q);
            transform.stamp_ = current_time;
            transform.frame_id_ = odom_frame_;
            transform.child_frame_id_ = base_frame_;
            tf_broadcaster_.sendTransform(transform);
        }

        // type.4 发布 Odom_Path（nav_msgs/Path）
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = current_time;
        ps.header.frame_id = odom_frame_;
        ps.pose.position.x = x_;
        ps.pose.position.y = y_;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = out.pose.pose.orientation;
        odom_path_.header.stamp = current_time;
        odom_path_.header.frame_id = odom_frame_;
        odom_path_.poses.push_back(ps);
        if (odom_traj_max_len_ > 0 && static_cast<int>(odom_path_.poses.size()) > odom_traj_max_len_) {
            odom_path_.poses.erase(odom_path_.poses.begin(),
                                   odom_path_.poses.begin() + (odom_path_.poses.size() - odom_traj_max_len_));
        }
        odom_path_pub_.publish(odom_path_);

        // type.5 发布Odom_Path文本odom_pose：本帧末尾会再次带重置标志发布一次，先占位
        visualization_msgs::Marker text_msg;
        text_msg.header.frame_id = odom_frame_;
        text_msg.header.stamp = current_time;
        text_msg.ns = "odom_forward_text";
        text_msg.id = 0;
        text_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_msg.action = visualization_msgs::Marker::ADD;
        text_msg.pose.position.x = x_;
        text_msg.pose.position.y = y_;
        text_msg.pose.position.z = 1.0;  // Path 上方 1m
        text_msg.pose.orientation.x = 0.0;
        text_msg.pose.orientation.y = 0.0;
        text_msg.pose.orientation.z = 0.0;
        text_msg.pose.orientation.w = 1.0;
        text_msg.scale.z = 1.0;  // 字体大小（米）
        text_msg.color.r = 0.0;
        text_msg.color.g = 1.0;
        text_msg.color.b = 0.0;
        text_msg.color.a = 1.0;
        text_msg.text = "odom_pose";
        odom_text_marker_pub_.publish(text_msg);

        // 将里程计前向积分位姿上报给核心，供 TF/发布使用（暂时没用到，因为属于双进程）
        cartographer::mapping::scan_matching::SetExternalOdometryPose2D(x_, y_, theta_);
        double loc_x = latest_loc_x_, loc_y = latest_loc_y_, loc_th = latest_loc_th_;
        double loc_score = latest_loc_score_;
        bool have_loc = have_latest_loc_ && have_latest_score_;  // 订阅到
        ROS_DEBUG_STREAM("[degeneracy_node] local integrated pose -> x=" << loc_x << " y=" << loc_y
                                                                         << " th=" << loc_th);
        int reset_flag_this_frame = 0;
        if (have_loc) {
            // 条件一、连续高分统计
            if (loc_score > 0.6)
                high_score_count_++;
            else
                high_score_count_ = 0;
            // 条件二、与里程计的一致性判定
            const double dx = loc_x - x_;
            const double dy = loc_y - y_;
            const double d_trans = std::hypot(dx, dy);
            const double yaw_diff = std::atan2(std::sin(loc_th - theta_), std::cos(loc_th - theta_));
            const double d_yaw = std::fabs(yaw_diff);
            bool use_loc = (high_score_count_ >= 5) && ((d_trans > reset_trans_thresh_) || (d_yaw > reset_yaw_thresh_));
            bool deg = cartographer::mapping::scan_matching::GetDegeneracyDetected();
            bool use_odom = (loc_score < 0.3) || deg;
            {
                // 同步到核心开关并通过话题广播给 cartographer_ros 进程
                cartographer::mapping::scan_matching::SetUseExternalOdom(use_odom);  // 暂时没用到，保留
                std_msgs::Bool b;
                b.data = use_odom;
                use_external_odom_pub_.publish(b);
            }

            // type.6 发布定位状态文本，位于 degeneracy_marker 下方 1m，大小为其 2 倍
            visualization_msgs::Marker status_msg;
            status_msg.header.frame_id = frame_id_;
            status_msg.header.stamp = ros::Time();
            status_msg.ns = "localization_status";
            status_msg.id = 2;  // 与 id=0 的主文本区分
            status_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            status_msg.action = visualization_msgs::Marker::ADD;
            status_msg.pose.position.x = status_marker_x_;
            status_msg.pose.position.y = status_marker_y_;
            status_msg.pose.position.z = status_marker_z_;
            status_msg.pose.orientation.x = 0.0;
            status_msg.pose.orientation.y = 0.0;
            status_msg.pose.orientation.z = 0.0;
            status_msg.pose.orientation.w = 1.0;
            status_msg.scale.z = 1.4;  // 等于 degeneracy_marker(0.8) 的 2 倍
            status_msg.color.r = use_odom ? 0.0 : 1.0;
            status_msg.color.g = use_odom ? 1.0 : 1.0;
            status_msg.color.b = 0.0;
            status_msg.color.a = 1.0;
            status_msg.text = use_odom ? "odom_status" : "local_status";
            localization_status_marker_pub_.publish(status_msg);
            // 同步字符串话题
            std_msgs::String status_str;
            status_str.data = status_msg.text;
            localization_status_pub_.publish(status_str);

            // type.7 发布“定位得分”文本，在主 marker 上方 1m，大小为主 marker 的 3 倍
            visualization_msgs::Marker score_msg;
            score_msg.header.frame_id = frame_id_;
            score_msg.header.stamp = ros::Time();
            score_msg.ns = "localization_score";
            score_msg.id = 3;
            score_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            score_msg.action = visualization_msgs::Marker::ADD;
            score_msg.pose.position.x = score_marker_x_;
            score_msg.pose.position.y = score_marker_y_;
            score_msg.pose.position.z = score_marker_z_;
            score_msg.pose.orientation.x = 0.0;
            score_msg.pose.orientation.y = 0.0;
            score_msg.pose.orientation.z = 0.0;
            score_msg.pose.orientation.w = 1.0;
            score_msg.scale.z = 1.4;  // 等于 degeneracy_marker(0.8) 的 3 倍
            score_msg.color.r = 1.0;
            score_msg.color.g = 1.0;
            score_msg.color.b = 1.0;
            score_msg.color.a = 1.0;
            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss.precision(3);
            oss << "score=" << loc_score;
            score_msg.text = oss.str();
            localization_score_marker_pub_.publish(score_msg);

            // type.8 发布定位 Local_Path（nav_msgs/Path）
            {
                geometry_msgs::PoseStamped lps;
                lps.header.stamp = latest_loc_stamp_;
                lps.header.frame_id = loc_frame_;
                lps.pose.position.x = loc_x;
                lps.pose.position.y = loc_y;
                lps.pose.position.z = 0.0;
                lps.pose.orientation = tf::createQuaternionMsgFromYaw(loc_th);
                loc_path_.header.stamp = latest_loc_stamp_;
                loc_path_.header.frame_id = loc_frame_;
                loc_path_.poses.push_back(lps);
                if (loc_traj_max_len_ > 0 && static_cast<int>(loc_path_.poses.size()) > loc_traj_max_len_) {
                    loc_path_.poses.erase(loc_path_.poses.begin(),
                                          loc_path_.poses.begin() + (loc_path_.poses.size() - loc_traj_max_len_));
                }
                loc_path_pub_.publish(loc_path_);
            }
            // type.9 发布定位标签文本：在定位位姿上方 1m 显示标签（红色）
            {
                visualization_msgs::Marker ltext;
                ltext.header.frame_id = loc_frame_;
                ltext.header.stamp = ros::Time();
                ltext.ns = "localization_text";
                ltext.id = 0;
                ltext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                ltext.action = visualization_msgs::Marker::ADD;
                ltext.pose.position.x = loc_x;
                ltext.pose.position.y = loc_y + 1;
                ltext.pose.position.z = 1.0;
                ltext.pose.orientation.x = 0.0;
                ltext.pose.orientation.y = 0.0;
                ltext.pose.orientation.z = 0.0;
                ltext.pose.orientation.w = 1.0;
                ltext.scale.z = 1.0;  // 字体大小（米）
                ltext.color.r = 1.0;
                ltext.color.g = 0.0;
                ltext.color.b = 0.0;
                ltext.color.a = 1.0;
                ltext.text = "loc_pose";
                localization_text_marker_pub_.publish(ltext);
            }
            // type.10 重置条件：定位状态下满足条件则用定位坐标重置里程计坐标
            if (use_loc && !use_odom) {
                ResetOdom(loc_x, loc_y, loc_th);
                reset_flag_this_frame = 1;
                odom_reset_count_++;
                ROS_INFO_STREAM("[degeneracy_node] reset odom to localization pose: x="
                                << loc_x << " y=" << loc_y << " th=" << loc_th << " (score=" << loc_score << ")");
            }
        } else {
            // 即便当前没有定位结果，也发布一个占位文本，方便 RViz 订阅后立即可见
            visualization_msgs::Marker score_msg;
            score_msg.header.frame_id = frame_id_;
            score_msg.header.stamp = ros::Time();
            score_msg.ns = "localization_score";
            score_msg.id = 3;
            score_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            score_msg.action = visualization_msgs::Marker::ADD;
            score_msg.pose.position.x = score_marker_x_;
            score_msg.pose.position.y = score_marker_y_;
            score_msg.pose.position.z = score_marker_z_;
            score_msg.pose.orientation.x = 0.0;
            score_msg.pose.orientation.y = 0.0;
            score_msg.pose.orientation.z = 0.0;
            score_msg.pose.orientation.w = 1.0;
            score_msg.scale.z = 2.4;
            score_msg.color.r = 0.7;
            score_msg.color.g = 0.7;
            score_msg.color.b = 0.7;
            score_msg.color.a = 1.0;
            score_msg.text = "score=NA";
            localization_score_marker_pub_.publish(score_msg);
            // 无定位时关闭外部里程计介入
            {
                cartographer::mapping::scan_matching::SetUseExternalOdom(false);
                std_msgs::Bool b;
                b.data = false;
                use_external_odom_pub_.publish(b);
            }
        }

        // type.11 发布里程计重置标志（1/0），并更新文本为 "odom_pose <flag>"
        {
            std_msgs::Int32 flag_msg;
            flag_msg.data = reset_flag_this_frame;
            odom_reset_flag_pub_.publish(flag_msg);

            visualization_msgs::Marker text_msg2 = text_msg;
            // 保持与当前姿态一致（若重置发生，则 x_/y_/theta_ 已更新）
            text_msg2.header.stamp = current_time;
            text_msg2.pose.position.x = x_;
            text_msg2.pose.position.y = y_;
            // 确保为绿色
            text_msg2.color.r = 0.0;
            text_msg2.color.g = 1.0;
            text_msg2.color.b = 0.0;
            text_msg2.color.a = 1.0;
            std::ostringstream ot;
            ot << "odom_pose " << odom_reset_count_;
            text_msg2.text = ot.str();
            odom_text_marker_pub_.publish(text_msg2);
        }
    }

   private:
    ros::NodeHandle nh_;
    // 严格解耦用的回调队列与 NodeHandle
    ros::CallbackQueue laser_queue_;
    ros::CallbackQueue odom_queue_;
    ros::CallbackQueue loc_queue_;
    ros::NodeHandle nh_laser_;
    ros::NodeHandle nh_odom_;
    ros::NodeHandle nh_loc_;
    std::unique_ptr<ros::AsyncSpinner> spinner_laser_;
    std::unique_ptr<ros::AsyncSpinner> spinner_odom_;
    std::unique_ptr<ros::AsyncSpinner> spinner_loc_;
    ros::Publisher pub_;
    ros::Publisher line_markers_pub_;
    ros::Publisher deg_marker_pub_;
    ros::Subscriber sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher odom_forward_pub_;
    ros::Publisher odom_path_pub_;
    ros::Publisher odom_text_marker_pub_;
    ros::Publisher odom_reset_flag_pub_;
    ros::Publisher localization_status_marker_pub_;
    ros::Publisher localization_status_pub_;
    ros::Publisher localization_score_marker_pub_;
    ros::Publisher loc_path_pub_;
    ros::Publisher localization_text_marker_pub_;
    ros::Publisher use_external_odom_pub_;
    ros::Subscriber latest_loc_sub_;
    ros::Subscriber loc_score_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string odom_topic_;
    std::string odom_forward_topic_;
    std::string odom_path_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string loc_path_topic_;
    std::string loc_frame_;
    bool have_last_time_ = false;
    ros::Time last_time_;
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    int odom_traj_max_len_ = 1000;
    int loc_traj_max_len_ = 1000;
    nav_msgs::Path odom_path_;
    nav_msgs::Path loc_path_;
    bool publish_tf_ = true;
    // 融合参数与状态
    int high_score_count_ = 0;
    double reset_trans_thresh_ = 0.01;              // m
    double reset_yaw_thresh_ = 1.0 * M_PI / 180.0;  // rad
    // 里程计累计重置次数
    int odom_reset_count_ = 0;
    // 三类文本标注位置
    double deg_marker_x_ = 0.0, deg_marker_y_ = 0.0, deg_marker_z_ = 0.0;
    double status_marker_x_ = 12, status_marker_y_ = 12.0, status_marker_z_ = 12.0;
    double score_marker_x_ = -12, score_marker_y_ = -18.0, score_marker_z_ = -16;
    // 缓存的定位数据（来自 cartographer_ros 话题）
    bool have_latest_loc_ = false;
    bool have_latest_score_ = false;
    ros::Time latest_loc_stamp_;
    double latest_loc_x_ = 0.0, latest_loc_y_ = 0.0, latest_loc_th_ = 0.0;
    double latest_loc_score_ = 0.0;
    bool first_loc_logged_ = false;
    bool first_score_logged_ = false;

    // 对外暴露的里程计读/写接口
   public:
    void ResetOdom(double x, double y, double theta)
    {
        x_ = x;
        y_ = y;
        theta_ = theta;
        have_last_time_ = false;  // 重置时间，避免大 dt
    }
    void GetOdom(double& x, double& y, double& theta) const
    {
        x = x_;
        y = y_;
        theta = theta_;
    }

    laser_line_extraction::DegeneracyDetector::Params params_;
    std::shared_ptr<laser_line_extraction::DegeneracyDetector> detector_;
    std::string frame_id_;
};

int main(int argc, char** argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "degeneracy_node");
    ros::NodeHandle nh("~");
    DegeneracyNode node;
    double frequency;
    nh.param<double>("frequency", frequency, 24.0);
    ROS_INFO(
        "degeneracy_node initialized (laser~24Hz via dedicated queue, odom~50Hz via dedicated queue), param "
        "frequency=%.1f (informational)",
        frequency);
    ros::waitForShutdown();
    return 0;
}
