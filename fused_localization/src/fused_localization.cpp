#include "fused_localization/fused_localization.h"

namespace fused_localization
{
FusedLocalization::FusedLocalization()
{
    tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10.0));
    fused_localization_init_flag_ = false;
}

FusedLocalization::~FusedLocalization()
{
}

bool FusedLocalization::Init()
{
    nh_.param("fused_localization_type", fused_localization_type_, 0);  // 0: proportion fuse. 1: ekf fuse.
    nh_.param("filter_type", filter_type_, 0);                          // 0: median filtering. 1: mean filtering.
    nh_.param("flag_use_carto_pose", flag_use_carto_pose_, true);
    nh_.param("flag_use_ref_pose", flag_use_ref_pose_, true);
    nh_.param("slide_window_threshold", slide_window_threshold_, 3);
    nh_.param("file_pose_path", file_pose_path_, std::string("/root/agv_release/agv/docs/record_pose.txt"));

    nh_.param("fused_sys_model_nosie_x", sys_model_nosie_.x(), 0.005);
    nh_.param("fused_sys_model_nosie_y", sys_model_nosie_.y(), 0.005);
    nh_.param("fused_sys_model_nosie_theta", sys_model_nosie_.theta(), 0.0005);

    nh_.param("fused_reflector_measure_nosie_x", reflector_measure_nosie_.x(), 0.0135);
    nh_.param("fused_reflector_measure_nosie_y", reflector_measure_nosie_.y(), 0.0135);
    nh_.param("fused_reflector_measure_nosie_theta", reflector_measure_nosie_.theta(), 0.01);

    nh_.param("fused_carto_measure_nosie_x", carto_measure_nosie_.x(), 0.01);
    nh_.param("fused_carto_measure_nosie_y", carto_measure_nosie_.y(), 0.01);
    nh_.param("fused_carto_measure_nosie_theta", carto_measure_nosie_.theta(), 0.01);

    region01_ = GetParamRegion(nh_, "region01");
    region02_ = GetParamRegion(nh_, "region02");

    subscribers_.emplace_back(nh_.subscribe<geometry_msgs::PoseStamped>(
        CartoPoseTopicName, TopicReciveCacheSize, boost::bind(&FusedLocalization::CartoPoseCallback, this, _1)));
    subscribers_.emplace_back(nh_.subscribe<std_msgs::Float32MultiArray>(
        CartoScoreTopicName, TopicReciveCacheSize, boost::bind(&FusedLocalization::CartoScoreCallback, this, _1)));
    subscribers_.emplace_back(nh_.subscribe<nav_msgs::Odometry>(
        RefPoseTopicName, TopicReciveCacheSize, boost::bind(&FusedLocalization::RefPoseCallback, this, _1)));
    subscribers_.emplace_back(nh_.subscribe<visualization_msgs::Marker>(
        MatchedReflectorsTopicName, TopicReciveCacheSize, boost::bind(&FusedLocalization::MatchedReflectorsCallback, this, _1)));
    // subscribers_.emplace_back(nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    //     InitialPoseTopicName, TopicReciveCacheSize, boost::bind(&FusedLocalization::InitCallbackTest, this, _1)));

    subscribers_.emplace_back(nh_.subscribe<nav_msgs::Odometry>(
        OdomTopicName, TopicReciveCacheSize, boost::bind(&FusedLocalization::HandleOdomMessage, this, _1)));

    fused_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(FusedPoseTopicName, TopicPublishCacheSize);
    // filtered_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("filtered_pose_test", TopicPublishCacheSize);
    polygon_points_publishers_ = nh_.advertise<visualization_msgs::Marker>("polygon_points", 2);
    polygon_shape_publishers_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon_shape", 2);

    set_carto_initpose_client_ = nh_.serviceClient<common_msgs::SetPose>(SetCartoInitPoseServiceName);
    using_predict_pose_client_ = nh_.serviceClient<std_srvs::SetBool>(UsingPredictPoseServiceName);
    service_servers_.emplace_back(nh_.advertiseService(AcquireFilePoseServiceName, &FusedLocalization::HandleAcquireFilePoseRequest, this));
    carto_reloc_cnt_ = 0;

    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), quat);
    carto_pose_.pose.orientation = quat;
    last_carto_pose_.pose.orientation = quat;
    fused_pose_.pose.orientation = quat;
    ref_pose_.pose.pose.orientation = quat;
    last_ref_pose_.pose.pose.orientation = quat;

    State_ init_state;
    init_state << 0.0, 0.0, 0.0;
    ekf_.init(init_state);

    using_predict_pose_last_srv_.response.message = "no_response";
    last_predict_time_ = ros::Time::now();

    sys_model_.SetNoise(sys_model_nosie_.x(), sys_model_nosie_.y(), sys_model_nosie_.theta());
    reflector_measurement_model_.SetNoise(reflector_measure_nosie_.x(), reflector_measure_nosie_.y(), reflector_measure_nosie_.theta());
    carto_measurement_model_.SetNoise(carto_measure_nosie_.x(), carto_measure_nosie_.y(), carto_measure_nosie_.theta());

    // tf::StampedTransform tmp_transform;
    // State_ laser_install_offset;
    // tf_->waitForTransform(std::string("base_link"), std::string("base_scan"), ros::Time(0), ros::Duration(30.0));//block until transfor is possible or 30s time out
    // tf_->lookupTransform(std::string("base_link"), std::string("base_scan"), ros::Time(0), tmp_transform);
    // laser_install_offset << tmp_transform.getOrigin().x(), tmp_transform.getOrigin().y(), tf::getYaw(tmp_transform.getRotation());

    // reflector_measurement_model_.SetTransform(laser_install_offset);
    // carto_measurement_model_.SetTransform(laser_install_offset);

    fused_localization_init_flag_ = true;
    fused_localization_thread_ = new std::thread(&FusedLocalization::ProcessThread, this);

    return true;
}

void FusedLocalization::InitCallbackTest(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    if(msg != nullptr){
        geometry_msgs::Point32 current_point;
        current_point.x = msg->pose.pose.position.x;
        current_point.y = msg->pose.pose.position.y;

        bool is_in_zone = CheckPointInZone(current_point, region01_);
        if(is_in_zone){
            ROS_INFO("point: (%f, %f) in zone.", current_point.x, current_point.y);
        }else{
            ROS_INFO("point: (%f, %f) not in zone!", current_point.x, current_point.y);
        }
    }
}

void FusedLocalization::HandleOdomMessage(const nav_msgs::Odometry::ConstPtr& msg) {
    control_.v() = msg->twist.twist.linear.x;
    control_.w() = msg->twist.twist.angular.z;

    ros::Duration delta_time = ros::Time::now() - last_predict_time_;
    last_predict_time_ = ros::Time::now();
    control_.time() = delta_time.toSec();
    predict_pose_ = ekf_.predict(sys_model_, control_);
    // ROS_INFO("ekf_.predict: (%f, %f, %f), vwt: (%f, %f, %f)", predict_pose_.x(), predict_pose_.y(), predict_pose_.theta(), 
    //                                                                              control_.v(), control_.w(), control_.time());
}

void FusedLocalization::PolygonPosePublishersTest(geometry_msgs::Polygon region)
{
    ros::Time current_time = ros::Time::now();
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = current_time;
    points.ns = "polygon";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // 设置marker类型
    points.type = visualization_msgs::Marker::POINTS;
    // 设置宽高
    points.scale.x = 1.2;
    points.scale.y = 1.2;
    // 设置颜色
    points.color.r = 0.0f;
    points.color.g = 0.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    geometry_msgs::PolygonStamped polygon_stamped;
    geometry_msgs::Point32 points32;
    polygon_stamped.header.frame_id = "map";
    polygon_stamped.header.stamp = current_time;
    geometry_msgs::Point p;
    p.x = region.points[0].x; p.y = region.points[0].y; p.z = 0.0;
    points.points.emplace_back(p);
    points32.x = region.points[0].x; points32.y = region.points[0].y; points32.z = 0.0;
    polygon_stamped.polygon.points.emplace_back(points32);

    p.x = region.points[1].x; p.y = region.points[1].y; p.z = 0.0;
    points.points.emplace_back(p);
    points32.x = region.points[1].x; points32.y = region.points[1].y; points32.z = 0.0;
    polygon_stamped.polygon.points.emplace_back(points32);

    p.x = region.points[2].x; p.y = region.points[2].y; p.z = 0.0;
    points.points.emplace_back(p);
    points32.x = region.points[2].x; points32.y = region.points[2].y; points32.z = 0.0;
    polygon_stamped.polygon.points.emplace_back(points32);

    p.x = region.points[3].x; p.y = region.points[3].y; p.z = 0.0;
    points.points.emplace_back(p);
    points32.x = region.points[3].x; points32.y = region.points[3].y; points32.z = 0.0;
    polygon_stamped.polygon.points.emplace_back(points32);

    polygon_points_publishers_.publish(points);
    polygon_shape_publishers_.publish(polygon_stamped);
}

geometry_msgs::Polygon FusedLocalization::GetParamRegion(const ros::NodeHandle& nh, std::string region_name)
{
    XmlRpc::XmlRpcValue region;
    geometry_msgs::Polygon polygon;
    
    //set default 4 points
    polygon.points.clear();
    for (int i = 0; i < 4; ++i){
        polygon.points.push_back(geometry_msgs::Point32());
    }

    if (!nh.getParam(region_name, region)){
        ROS_ERROR_STREAM("'polygon' cannot be loaded, since param '" << nh.getNamespace()
                                                                        << region_name << "' does not exist.");
    }

    if (region.getType() == XmlRpc::XmlRpcValue::TypeArray){
      try{
        polygon = MakeRegionFromXMLRPC(region, region_name);
        ROS_INFO_STREAM("'polygon' loaded.");
      }catch (const std::exception &ex){
        ROS_ERROR_STREAM("'polygon' cannot be loaded: " << ex.what() << ".");
      }
    }else{
      ROS_ERROR_STREAM("'polygon' cannot be loaded, since param '" << nh.getNamespace()
                                                                        << region_name << "' does not define an array of coordinates.");
    }

    return polygon;
}

geometry_msgs::Polygon FusedLocalization::MakeRegionFromXMLRPC(XmlRpc::XmlRpcValue &region_xmlrpc, const std::string &full_param_name)
{
    // Make sure we have an array of at least 3 elements.
    if (region_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || region_xmlrpc.size() < 3){
        ROS_FATAL("The region must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(region_xmlrpc).c_str());
        throw std::runtime_error("The region must be specified as list of lists on the parameter server with at least "
                                "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    geometry_msgs::Polygon region;
    geometry_msgs::Point32 pt;

    for (int i = 0; i < region_xmlrpc.size(); ++i){
        // Make sure each element of the list is an array of size 2. (x and y coordinates)
        XmlRpc::XmlRpcValue point = region_xmlrpc[i];
        if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2){
            ROS_FATAL("The region (parameter %s) must be specified as list of lists on the parameter server eg: "
                        "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                        full_param_name.c_str());
            throw std::runtime_error("The region must be specified as list of lists on the parameter server eg: "
                                        "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
        }

        pt.x = GetNumberFromXMLRPC(point[0], full_param_name);
        pt.y = GetNumberFromXMLRPC(point[1], full_param_name);
        pt.z = 0.0;

        region.points.push_back(pt);
    }

    return region;
}

bool FusedLocalization::HandleAcquireFilePoseRequest(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
    geometry_msgs::Pose2D read_data;
    if(ReadFilePose(file_pose_path_, read_data)){
        ROS_INFO("ReadFilePose success!");
        response.success = true;
        response.message = std::to_string(read_data.x) + " " + std::to_string(read_data.y) + " " + std::to_string(read_data.theta);
        
        return true;
    }else{
        ROS_ERROR("ReadFilePose failed!");
        response.success = false;
        response.message = "failed!";
    }

    return false;
}

geometry_msgs::PoseStamped FusedLocalization::Process()
{
    double carto_pose_x = carto_pose_.pose.position.x;
    double carto_pose_y = carto_pose_.pose.position.y;
    double carto_pose_th = tf::getYaw(carto_pose_.pose.orientation);

    double ref_pose_x = ref_pose_.pose.pose.position.x;
    double ref_pose_y = ref_pose_.pose.pose.position.y;
    double ref_pose_th = tf::getYaw(ref_pose_.pose.pose.orientation);

    if(flag_use_carto_pose_ && !flag_use_ref_pose_){
        fused_pose_.pose = carto_pose_.pose;
    }else if(!flag_use_carto_pose_ && flag_use_ref_pose_){
        fused_pose_.pose = ref_pose_.pose.pose;
    }


    geometry_msgs::Pose2D write_data;
    write_data.x = fused_pose_.pose.position.x;
    write_data.y = fused_pose_.pose.position.y;
    write_data.theta = tf::getYaw(fused_pose_.pose.orientation);

    if(TimeOut(write_file_pose_time_, 0.5)){
        write_file_pose_time_ = ros::Time::now();

        if(carto_score_ > 0.4 || matched_ref_size_ > 4){
            if(WriteFilePose(file_pose_path_, write_data)){
                ROS_INFO_THROTTLE(1.0, "WriteFilePose success!");
            }else{
                ROS_ERROR("WriteFilePose failed!");
            }
        }
        region01_ = GetParamRegion(nh_, "region01");
        region02_ = GetParamRegion(nh_, "region02");

        //for test
        // PolygonPosePublishersTest(region02_);
    }

    //两者不一致时为true
    if(flag_use_carto_pose_ ^ flag_use_ref_pose_){
        return fused_pose_;
    }

    bool need_reloc = false;
    float pose_diff = std::hypot(carto_pose_x-ref_pose_x, carto_pose_y-ref_pose_y);
    float diff_thresh = 0.5f;

    geometry_msgs::Point32 current_point;
    current_point.x = ref_pose_x;
    current_point.y = ref_pose_y;

    bool is_in_region01 = false;
    is_in_region01 = CheckPointInZone(current_point, region01_);

    bool is_in_region02 = false;
    current_point.x = carto_pose_x;
    current_point.y = carto_pose_y;
    is_in_region02 = CheckPointInZone(current_point, region02_);
    if (is_in_region02){
        fused_pose_.pose = carto_pose_.pose;

        return fused_pose_;
    }

    if(is_in_region01){
        diff_thresh = 0.1f;
        using_predict_pose_srv_.request.data = true;
    }else{
        diff_thresh = 0.5f;
        using_predict_pose_srv_.request.data = false;
    }

    if (using_predict_pose_srv_.request.data != using_predict_pose_last_srv_.request.data ||
        using_predict_pose_srv_.response.message != using_predict_pose_last_srv_.response.message){
        using_predict_pose_client_.call(using_predict_pose_srv_);
        ROS_INFO("using_predict_pose_client_.call - data: %d, success: %d, msg: %s", using_predict_pose_srv_.request.data, using_predict_pose_srv_.response.success, 
                                                                                        using_predict_pose_srv_.response.message.c_str());
        using_predict_pose_last_srv_.request.data = using_predict_pose_srv_.request.data;
        using_predict_pose_last_srv_.response.message = using_predict_pose_srv_.response.message;
    }

    ROS_INFO_THROTTLE(1.0, "carto_pose_x: %f, carto_pose_y: %f, carto_pose_th: %f, pose_diff: %f, matched_ref_size_: %d, carto_score_: %f, is_in_region01: %d, carto_reloc_cnt_: %d", 
        carto_pose_x, carto_pose_y, carto_pose_th, pose_diff, matched_ref_size_, carto_score_, is_in_region01, carto_reloc_cnt_);

    if(matched_ref_size_ < 4 && pose_diff > 1.0) return carto_pose_;

    if(carto_pose_x != 0.0 && carto_pose_y != 0.0 && pose_diff > diff_thresh && matched_ref_size_ >= 4 &&
            (carto_score_ <= 0.4 || is_in_region01) && carto_reloc_cnt_ < 5) {
        need_reloc = true;
    } else {
        need_reloc = false;
    }

    if(carto_score_ > 0.4) {
        carto_reloc_cnt_ = 0;
    }
    
    if((carto_pose_x == 0.0 && carto_pose_y == 0.0 && carto_pose_th == 0.0) || need_reloc){
        if(matched_ref_size_ >= 4){
            if(TimeOut(carto_initpose_time_, 0.5)){
                common_msgs::SetPose srv;
                srv.request.x = ref_pose_x;
                srv.request.y = ref_pose_y;
                srv.request.theta = ref_pose_th;

                set_carto_initpose_client_.call(srv);
                ROS_INFO("========= set_carto_initpose_client_.call(srv) =========");
                
                if(need_reloc) carto_reloc_cnt_++;

                carto_initpose_time_ = ros::Time::now();
            }
        }
    }

    if (fused_localization_type_ == PROPORTION_FUSE) {
        if(carto_pose_x != 0.0 || carto_pose_y != 0.0 || carto_pose_th != 0.0){
            if(carto_score_ < 0.3 || is_in_region01){
                carto_score_ = 0.0;
            }
            fused_pose_.pose.position.x = carto_pose_x*carto_score_ + ref_pose_x*(1-carto_score_);
            fused_pose_.pose.position.y = carto_pose_y*carto_score_ + ref_pose_y*(1-carto_score_);
            fused_pose_.pose.position.z = 0.0;

            Eigen::Quaterniond carto_quat(Eigen::AngleAxisd(carto_pose_th, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond ref_quat(Eigen::AngleAxisd(ref_pose_th, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond fuse_quat = ref_quat.slerp(carto_score_, carto_quat);
            // double fused_pose_th = fuse_quat.normalized().toRotationMatrix().eulerAngles(0, 1, 2)[2];

            fused_pose_.pose.orientation.x = fuse_quat.x();
            fused_pose_.pose.orientation.y = fuse_quat.y();
            fused_pose_.pose.orientation.z = fuse_quat.z();
            fused_pose_.pose.orientation.w = fuse_quat.w();
        }
    } else if (fused_localization_type_ == EKF_FUSE) {
        if(carto_pose_x != 0.0 || carto_pose_y != 0.0 || carto_pose_th != 0.0){
            if(is_in_region01){
                fused_pose_.pose.position.x = ref_pose_x;
                fused_pose_.pose.position.y = ref_pose_y;
                fused_pose_.pose.position.z = 0.0;
    
                fused_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(ref_pose_th);
            }else {
                //判断ref_pose和carto_pose哪个跳变量更小，就更相信哪个位姿
                double delta_carto_x = carto_pose_x - last_carto_pose_.pose.position.x;
                double delta_carto_y = carto_pose_y - last_carto_pose_.pose.position.y;
                double delta_carto_th = angles::shortest_angular_distance(carto_pose_th, tf::getYaw(last_carto_pose_.pose.orientation));
                double delta_carto_xy = std::hypot(delta_carto_x, delta_carto_y);
                last_carto_pose_ = carto_pose_;

                double delta_ref_x = ref_pose_x - last_ref_pose_.pose.pose.position.x;
                double delta_ref_y = ref_pose_y - last_ref_pose_.pose.pose.position.y;
                double delta_ref_th = angles::shortest_angular_distance(ref_pose_th, tf::getYaw(last_ref_pose_.pose.pose.orientation));
                double delta_ref_xy = std::hypot(delta_ref_x, delta_ref_y);
                last_ref_pose_ = ref_pose_;

                State_ carto_measure_nosie_tmp = carto_measure_nosie_;
                State_ reflector_measure_nosie_tmp = reflector_measure_nosie_;
                double factor = 2.0;
                double lower_bound = 0.8;
                double upper_bound = 1.2;
                if(delta_carto_xy < lower_bound*delta_ref_xy){
                    carto_measure_nosie_tmp.x() *= 1.0/factor;
                    carto_measure_nosie_tmp.y() *= 1.0/factor;
                    reflector_measure_nosie_tmp.x() *= factor;
                    reflector_measure_nosie_tmp.y() *= factor;
                    ROS_INFO("lyy0417: ekf fuse more believe carto position.");
                } else if(delta_carto_xy > upper_bound*delta_ref_xy){
                    carto_measure_nosie_tmp.x() *= factor;
                    carto_measure_nosie_tmp.y() *= factor;
                    reflector_measure_nosie_tmp.x() *= 1.0/factor;
                    reflector_measure_nosie_tmp.y() *= 1.0/factor;
                    ROS_INFO("lyy0417: ekf fuse more believe ref position.");
                }

                if(delta_carto_th < lower_bound*delta_ref_th){
                    carto_measure_nosie_tmp.theta() *= 1.0/factor;
                    reflector_measure_nosie_tmp.theta() *= factor;
                    ROS_INFO("lyy0417: ekf fuse more believe carto theta.");
                } else if(delta_carto_th > upper_bound*delta_ref_th){
                    carto_measure_nosie_tmp.theta() *= factor;
                    reflector_measure_nosie_tmp.theta() *= 1.0/factor;
                    ROS_INFO("lyy0417: ekf fuse more believe ref theta.");
                }

                carto_measurement_.x() = carto_pose_x;
                carto_measurement_.y() = carto_pose_y;
                carto_measurement_.theta() = carto_pose_th;
                
                carto_measurement_model_.SetNoise(carto_measure_nosie_tmp.x(), carto_measure_nosie_tmp.y(), carto_measure_nosie_tmp.theta());
                predict_pose_ = ekf_.update(carto_measurement_model_, carto_measurement_);
                ROS_INFO("carto_pose (%f, %f, %f), ekf_.update1: (%f, %f, %f)", carto_pose_x, carto_pose_y, carto_pose_th, 
                    predict_pose_.x(), predict_pose_.y(), predict_pose_.theta());
    
                reflector_measurement_.x() = ref_pose_x;
                reflector_measurement_.y() = ref_pose_y;
                reflector_measurement_.theta() = ref_pose_th;
                reflector_measurement_model_.SetNoise(reflector_measure_nosie_tmp.x(), reflector_measure_nosie_tmp.y(), reflector_measure_nosie_tmp.theta());
                predict_pose_ = ekf_.update(reflector_measurement_model_, reflector_measurement_);
                ROS_INFO("ref_pose (%f, %f, %f), ekf_.update2: (%f, %f, %f)", ref_pose_x, ref_pose_y, ref_pose_th, 
                    predict_pose_.x(), predict_pose_.y(), predict_pose_.theta());
    
                fused_pose_.pose.position.x = predict_pose_.x();
                fused_pose_.pose.position.y = predict_pose_.y();
                fused_pose_.pose.position.z = 0.0;
    
                fused_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(predict_pose_.theta());
            }
        }
    }

    return fused_pose_;
}

bool FusedLocalization::ProcessThread()
{
    ros::Rate rate(100);
    while(ros::ok())
    {   
        if(fused_localization_init_flag_){
            geometry_msgs::PoseStamped fused_pose = Process();
            fused_pose.header.stamp = ros::Time::now();
            fused_pose.header.frame_id = "map";
    
            // geometry_msgs::Pose filtered_pose = PoseFilter(fused_pose.pose.position.x, fused_pose.pose.position.y, tf::getYaw(fused_pose.pose.orientation));
            // fused_pose.pose = filtered_pose;
            
            fused_pose_publisher_.publish(fused_pose);
            // filtered_pose_publisher_.publish(fused_pose);
        }
        rate.sleep();
    }
}

/*
功能：判断点是否在四边形内，该函数也适用于多边形，将点数改成你想要的边数就行。
参数：
    point：当前点
    rect：四边形，4个点要顺时针或逆时针相邻
返回值：如果在四边形内返回 true ，否则返回 false
*/
bool FusedLocalization::PointInRect(geometry_msgs::Point32 point, geometry_msgs::Polygon rect)
{
    int nCount = 4;  // 任意四边形有4个顶点
    if (rect.points.size() == nCount) {
        geometry_msgs::Point32 RectPoints[] = {rect.points[0], rect.points[1], rect.points[2], rect.points[3]};
        int nCross = 0;
        double lastPointX = -999999.999;
        for (int i = 0; i < nCount; i++) {
            // 依次取相邻的两个点
            geometry_msgs::Point32 pBegin = RectPoints[i];
            geometry_msgs::Point32 pEnd = RectPoints[(i + 1) % nCount];
            // 相邻的两个点是平行于x轴的,当前点和x轴的平行线要么重合,要么不相交,不算
            if (pBegin.y == pEnd.y)
                continue;
            // 交点在pBegin,pEnd的延长线上,不算
            if (point.y < std::min(pBegin.y, pEnd.y) || point.y > std::max(pBegin.y, pEnd.y))
                continue;
            // 当前点和x轴的平行线与pBegin,pEnd直线的交点的x坐标
            double x =
                (double)(point.y - pBegin.y) * (double)(pEnd.x - pBegin.x) / (double)(pEnd.y - pBegin.y) + pBegin.x;
            if (x > point.x)  // 只看pCur右边交点
            {
                if (x != lastPointX)  // 防止角点算两次
                {
                    nCross++;
                    lastPointX = x;
                }
            }
        }

        // 单方向交点为奇数，点在多边形之内。单方向交点为偶数，点在多边形之外。
        return (nCross % 2 == 1);
    } else {
        ROS_ERROR("PointInRect: rect.size: %d != 4", rect.points.size());
    }

    return false;
}

bool FusedLocalization::CheckPointInZone(geometry_msgs::Point32 point, geometry_msgs::Polygon rect)
{
    if(!rect.points.empty()){
        if (PointInRect(point, rect)) {
            ROS_DEBUG("CheckPointInZone: point(%f, %f) in zone", point.x, point.y);
            return true;
        }
        ROS_DEBUG("CheckPointInZone: point(%f, %f) not in zone", point.x, point.y);
    }
    
    return false;
}

geometry_msgs::Pose FusedLocalization::PoseFilter(double x, double y, double theta)
{
    geometry_msgs::Pose2D current_pose;
    current_pose.x = x;
    current_pose.y = y;
    current_pose.theta = theta;

    if (pose_filter_deque_.size() < slide_window_threshold_) {
        // pose_filter_deque_.emplace_back(std::move(current_pose));
        pose_filter_deque_.push_back(current_pose);
    } else {
        pose_filter_deque_.pop_front();
        // pose_filter_deque_.emplace_back(std::move(current_pose));
        pose_filter_deque_.push_back(current_pose);
        if (filter_type_ == MEDIAN_FILTERING) {
            std::vector<float> x_values, y_values, theta_values;

            // 收集窗口数据
            for (const auto& pos : pose_filter_deque_) {
                x_values.push_back(pos.x);
                y_values.push_back(pos.y);
                theta_values.push_back(pos.theta);
            }
            // 排序取中值
            auto median = [](std::vector<float>& values) {
                const size_t n = values.size();
                std::sort(values.begin(), values.end());
                return values[n / 2];  // 适用于奇数窗口
            };

            // 计算三维中值
            current_pose.x = median(x_values);
            current_pose.y = median(y_values);
            current_pose.theta = median(theta_values);

            ROS_DEBUG("MEDIAN_FILTERING - slide_window_threshold: %d, median(x,y,th): (%f, %f, %f)",
                      slide_window_threshold_, current_pose.x, current_pose.y, angles::to_degrees(current_pose.theta));
        } else if (filter_type_ == MEAN_FILTERING) {
            float sum_x = 0.f;
            float sum_y = 0.f;
            int count_tmp = 0;

            Eigen::Quaterniond avg_quat(Eigen::AngleAxisd(pose_filter_deque_.front().theta, Eigen::Vector3d::UnitZ()));
            for (auto pos : pose_filter_deque_) {
                sum_x += pos.x;
                sum_y += pos.y;

                Eigen::Quaterniond tmp_quat(Eigen::AngleAxisd(pos.theta, Eigen::Vector3d::UnitZ()));
                avg_quat = avg_quat.slerp(1.0 / pose_filter_deque_.size(), tmp_quat);
                count_tmp++;
            }
            current_pose.x = sum_x / float(pose_filter_deque_.size());
            current_pose.y = sum_y / float(pose_filter_deque_.size());
            current_pose.theta = avg_quat.normalized().toRotationMatrix().eulerAngles(0, 1, 2)[2];

            ROS_DEBUG("MEAN_FILTERING - slide_window_count: %d, avg (x,y,th): (%f, %f, %f)", count_tmp, current_pose.x,
                      current_pose.y, angles::to_degrees(current_pose.theta));
        }
    }

    geometry_msgs::Pose update_pose;
    update_pose.position.x = current_pose.x;
    update_pose.position.y = current_pose.y;
    update_pose.position.z = 0.0;

    update_pose.orientation.x = 0.0;
    update_pose.orientation.y = 0.0;
    update_pose.orientation.z = std::sin(current_pose.theta / 2.0);
    update_pose.orientation.w = std::cos(current_pose.theta / 2.0);

    return update_pose;
}

void FusedLocalization::CartoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    carto_pose_ = *msg;
}

void FusedLocalization::CartoScoreCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() > 0){
        carto_score_ = msg->data[0];
    }
}

void FusedLocalization::RefPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ref_pose_ = *msg;
}

void FusedLocalization::MatchedReflectorsCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    matched_ref_size_ = msg->points.size();
}

}  // namespace fused_localization