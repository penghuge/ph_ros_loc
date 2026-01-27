#ifndef FUSED_LOCALIZATION_H
#define FUSED_LOCALIZATION_H

#include <angles/angles.h>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <deque>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <common_msgs/SetPose.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <shared_mutex>
#include <iostream>
#include <stdio.h>
#include <string>
#include <time.h>
#include <std_msgs/Float32MultiArray.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <std_srvs/SetBool.h>

#include <kalman/ExtendedKalmanFilter.hpp>
#include "measure_model/PoseMeasurementModel.hpp"
#include "measure_model/PositionMeasurementModel.hpp"
#include "measure_model/system_model.h"

namespace fused_localization
{
typedef double T;

typedef location::State<T> State_;
typedef location::Control<T> Control_;
typedef location::SystemModel<T> SystemModel_;

typedef location::PositionMeasurement<T> PositionMeasurement_;
typedef location::PositionMeasurementModel<T> PositionMeasurementModel_;

typedef location::PoseMeasurement<T> PoseMeasurement_;
typedef location::PoseMeasurementModel<T> PoseMeasurementModel_;

constexpr char CartoPoseTopicName[] = "tracked_pose";
constexpr char CartoScoreTopicName[] = "best_score_submapid";
constexpr char RefPoseTopicName[] = "reflector_location";
constexpr char FusedPoseTopicName[] = "fused_pose";
constexpr char InitialPoseTopicName[] = "initialpose";
constexpr char OdomTopicName[] = "odom";
constexpr char SetCartoInitPoseServiceName[] = "set_carto_initpose";
constexpr char MatchedReflectorsTopicName[] = "matched_reflectors";
constexpr char AcquireFilePoseServiceName[] = "acquire_file_pose";
constexpr char UsingPredictPoseServiceName[] = "using_predict_pose";
constexpr int TopicReciveCacheSize = 1;
constexpr int TopicPublishCacheSize = 1;

enum FilterTypeEnum
{
    MEDIAN_FILTERING = 0,
    MEAN_FILTERING,
    FILTERING_NUM
};

enum FusedLocalizationEnum
{
    PROPORTION_FUSE = 0,
    EKF_FUSE,
    FUSE_NUM
};


class FusedLocalization
{
    public:
        FusedLocalization();
        ~FusedLocalization();

        void CartoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void CartoScoreCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void RefPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
        bool Init();
        bool ProcessThread();
        geometry_msgs::Pose PoseFilter(double x, double y, double theta);
        bool PointInRect(geometry_msgs::Point32 point, geometry_msgs::Polygon rect);
        bool CheckPointInZone(geometry_msgs::Point32 point, geometry_msgs::Polygon rect);
        geometry_msgs::PoseStamped Process();
        void InitCallbackTest(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void PolygonPosePublishersTest(geometry_msgs::Polygon region);
        void MatchedReflectorsCallback(const visualization_msgs::Marker::ConstPtr& msg);
        bool HandleAcquireFilePoseRequest(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
        geometry_msgs::Polygon MakeRegionFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name);
        geometry_msgs::Polygon GetParamRegion(const ros::NodeHandle& nh, std::string region_name);
        void HandleOdomMessage(const nav_msgs::Odometry::ConstPtr& msg);

        double GetNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name){
            // Make sure that the value we're looking at is either a double or an int.
            if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
                std::string &value_string = value;
                ROS_FATAL("Values in the region specification (param %s) must be numbers. Found value %s.",
                            full_param_name.c_str(), value_string.c_str());
                throw std::runtime_error("Values in the region specification must be numbers");
            }
            return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
        }

        std::string ConvertTimeStamp2TimeStr(time_t timeStamp){
            struct tm *timeinfo = nullptr;
            char buffer[80];
            timeinfo = localtime(&timeStamp);
            strftime(buffer, 80,"%Y-%m-%d-%H:%M:%S",timeinfo);

            return std::string(buffer);
        }

        bool TimeOut(ros::Time time, double duration_thresh){
            ros::Duration dur = ros::Duration(ros::Time::now() - time);
            
            if(dur.toSec() > duration_thresh){
                return true;
            }

            return false;
        }
        
        //读写锁同步文件写入和读取：允许多个线程同时读，但有一个线程写时，则阻止其他线程的读和写
        bool ReadFilePose(const std::string& filename, geometry_msgs::Pose2D& read_data) {
            std::shared_lock<std::shared_mutex> lock(record_pose_rw_mutex_);
            std::ifstream in(filename);

            if (!in.is_open()){
                ROS_ERROR("Read %s failed!", filename.c_str());
                return false;
            }
            
            std::stringstream ss;
            std::string one_line;
            std::string time;
            while (in.good() && !in.eof()) {
                getline(in, one_line, '\n');

                if(one_line.empty()) continue;
                ss.clear();
                time.clear();

                ss.str(one_line);
                ss >> time >> read_data.x >> read_data.y >> read_data.theta;

                if(ss.fail()){
                    ROS_WARN("read_fstream: ss read %s fail! continue...", filename.c_str());
                    continue;
                }
            }
            
            ROS_INFO("Read: %f, %f, %f from %s", read_data.x, read_data.y, read_data.theta, filename.c_str());

            in.close();

            return true;
        }

        bool WriteFilePose(const std::string& filename, const geometry_msgs::Pose2D& write_data) {
            std::unique_lock<std::shared_mutex> lock(record_pose_rw_mutex_);
            std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);

            if (!out.is_open()){
                ROS_ERROR("Write %s failed!", filename.c_str());
                return false;
            }
            std::string time = ConvertTimeStamp2TimeStr(ros::Time::now().toSec());
            std::string one_line = time + " " + std::to_string(write_data.x) + " " + std::to_string(write_data.y) + " " + std::to_string(write_data.theta);
            out << one_line << std::endl;

            ROS_INFO_THROTTLE(1.0, "Write: %s to %s", one_line.c_str(), filename.c_str());
            out.close();

            return true;
        }

    private:
        ros::NodeHandle nh_;
        int fused_localization_type_;//0: proportion fuse. 1: ekf fuse. 
        int filter_type_;// 0: median filtering. 1: mean filtering.
        bool flag_use_carto_pose_;
        bool flag_use_ref_pose_;
        ros::Publisher fused_pose_publisher_;
        ros::Publisher filtered_pose_publisher_;
        ros::Publisher polygon_points_publishers_;
        ros::Publisher polygon_shape_publishers_;
        std::vector<ros::Subscriber> subscribers_;
        geometry_msgs::PoseStamped carto_pose_, last_carto_pose_;
        geometry_msgs::PoseStamped fused_pose_;
        double carto_score_;
        nav_msgs::Odometry ref_pose_, last_ref_pose_;
        std::deque<geometry_msgs::Pose2D> pose_filter_deque_;
        int slide_window_threshold_;
        int matched_ref_size_;
        int carto_reloc_cnt_;

        ros::ServiceClient set_carto_initpose_client_;
        ros::ServiceClient using_predict_pose_client_;
        std::vector<ros::ServiceServer> service_servers_;

        std::thread* fused_localization_thread_;
        ros::Time carto_initpose_time_;
        ros::Time write_file_pose_time_;
        std::shared_mutex record_pose_rw_mutex_;
        std::string file_pose_path_;

        geometry_msgs::Polygon region01_, region02_;
        std_srvs::SetBool using_predict_pose_srv_;
        std_srvs::SetBool using_predict_pose_last_srv_;

        SystemModel_ sys_model_;
        PoseMeasurement_ reflector_measurement_;
        PoseMeasurement_ carto_measurement_;
        PoseMeasurementModel_ reflector_measurement_model_;
        PoseMeasurementModel_ carto_measurement_model_;
        Kalman::ExtendedKalmanFilter<State_> ekf_;
        Control_ control_;

        State_ sys_model_nosie_;
        State_ reflector_measure_nosie_;
        State_ carto_measure_nosie_;

        std::shared_ptr<tf::TransformListener> tf_;
        ros::Time last_predict_time_;
        bool fused_localization_init_flag_;
        State_ predict_pose_;
};
}  // namespace fused_localization

#endif