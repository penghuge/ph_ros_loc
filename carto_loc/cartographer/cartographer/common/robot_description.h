#ifndef ROBOT_DESCRIPTION_H_
#define ROBOT_DESCRIPTION_H_

#include "format_transform.h"
#include "config_file_operator.h"
#include "const_value.h"
#include "tools/template_class.h"
#include "tools/config_pool.h"


#include "cartographer/transform/rigid_transform.h"
#include "absl/memory/memory.h"

namespace SlamCommon
{
struct RobotMechanicalParam
{
    double wheel_base = 0.;             //front and back wheel base
    // steering wheel offset from the center line of robot
    // positive on the left, and negative on the right
    double steering_wheel_offset = 0.;
    double driving_wheel_offset = 0.;    //driver wheel offset from the center line of robot
    double steering_filter_rate = 0.;
    double driving_filter_rate = 0.;
};
enum SensorTypeEnum
{
    SENSORTYPE_LASER_SCAN = 0,
    SENSORTYPE_MULTI_ECHO_LASER_SCAN,
    SENSORTYPE_POINT_CLOUD_2,
    SENSORTYPE_IMU,
    SENSORTYPE_ODOMETRY,
    SENSORTYPE_SATELLITE,
    SENSORTYPE_2D_CODE,
    SENSORTYPE_LANDMARK,
    SENSORTYPE_NUM
};

struct SensorHeader
{
    SlamCommon::Time time = SlamCommon::Time::min();
    SlamCommon::SensorTypeEnum sensor_type = SlamCommon::SENSORTYPE_LASER_SCAN;

    bool operator <(const SensorHeader &other) const
    {
        return time > other.time;
    }
};

struct SensorCommonData
{
    SensorHeader header;
    SlamCommon::SlamLaserScanData laser_scan;
    SlamCommon::SlamMultiEchoLaserScanData echoes;
    SlamCommon::SlamPointCloud2Data point_cloud2;
    SlamCommon::SlamNavSatFixData nav_sat_fix;
    SlamCommon::SlamIMUData imu;
    SlamCommon::SlamOdometryPoseData odometry;
    bool last_msg = false;

    bool operator <(const SensorCommonData &other) const
    {
        return header.time > other.header.time;
    }
};

struct SensorCoordValue
{
    double x = 0.;
    double y = 0.;
    double z = 0.;
    double roll = 0.;
    double pitch = 0.;
    double yaw = 0.;
};
struct SensorInfoStru
{
    int type = 0;           //传感器型号
    std::string name = "";  //传感器名称
    SensorCoordValue coord; //传感器坐标
};

class CRobotDescription : public CSingletonTemp<CRobotDescription>
{
    //需要将基类声明成友员，以便访问子类的私有构造函数和析构函数
    friend class CSingletonTemp<CRobotDescription>;
public:
    struct ParamsRoboStru : public ParamBaseStru
    {
    #define PARAM_KEY_ROBOT_DES_ROBOT_MODEL_NAME            "robot_model"
    #define PARAM_KEY_ROBOT_DES_MECHANICAL_PARAM_NAME       "robot_mechanical_params"
    #define PARAM_KEY_ROBOT_DES_WHEEL_BASE_NAME             "wheel_base"
    #define PARAM_KEY_ROBOT_DES_STEERING_WHEEL_OFFSET_NAME  "steering_wheel_offset"
    #define PARAM_KEY_ROBOT_DES_DRIVING_WHEEL_OFFSET_NAME   "driving_wheel_offset"
    #define PARAM_KEY_ROBOT_DES_STEERING_FILTER_RATE_NAME   "steering_filter_rate"
    #define PARAM_KEY_ROBOT_DES_DRIVING_FILTER_RATE_NAME    "driving_filter_rate"
    #define PARAM_KEY_ROBOT_DES_SENSOR_TYPE_NAME            "type"
    #define PARAM_KEY_ROBOT_DES_SENSOR_NAME_NAME            "name"
    #define PARAM_KEY_ROBOT_DES_SENSOR_COORD_X_NAME         "x"
    #define PARAM_KEY_ROBOT_DES_SENSOR_COORD_Y_NAME         "y"
    #define PARAM_KEY_ROBOT_DES_SENSOR_COORD_Z_NAME         "z"
    #define PARAM_KEY_ROBOT_DES_SENSOR_COORD_ROLL_NAME      "roll"
    #define PARAM_KEY_ROBOT_DES_SENSOR_COORD_PITCH_NAME     "pitch"
    #define PARAM_KEY_ROBOT_DES_SENSOR_COORD_YAW_NAME       "yaw"

        RobotStructModel robot_model;
        std::vector<RobotMechanicalParam> mechanical_param;
        std::map<SensorTypeEnum, std::vector<SensorInfoStru>> sensor;
        bool operator <(const CRobotDescription::ParamsRoboStru &other) const
        {
            return robot_model < other.robot_model;
        }

        static unsigned int InsertToRoboMsgPool(ParamsRoboStru *param,
                                            const std::string &name = CFG_POOL_ROBOT_DES)
        {
            return CConfigPool::GetInstance().AddData<ParamsRoboStru>(name, param);
        }
        static ParamsRoboStru *GetRoboMsgPoolData(const std::string &name = CFG_POOL_ROBOT_DES)
        {
            return CConfigPool::GetInstance().GetData<ParamsRoboStru>(name);
        }
        static void UpdateRoboMsgPoolData(const std::string &str,
                                      const std::string &name = CFG_POOL_ROBOT_DES)
        {
            ParamsRoboStru *param = CConfigPool::GetInstance().GetData<ParamsRoboStru>(name);
            param->UpdateParams(str);
        }
        static void UpdateRoboXmlFile(const std::string &name = CFG_POOL_ROBOT_DES)
        {
            //获取消息池数据
            ParamsRoboStru *param = CConfigPool::GetInstance().GetData<ParamsRoboStru>(name);
            //找到robot_description参数文件
            std::string cur_path("");
            // LOG_PROJECT_PATH(cur_path);
//            LOG_INFO("cur_path = %s", cur_path.c_str());
            CConfigFileOperator *config_operator = new CConfigFileOperator(cur_path);
            config_operator->SetNodeDirectory(DIR_CONFIGURATION_FILE_ID + std::string(DIR_FLAG)
                                              + CFG_SLAM_COMMON_ID);
            config_operator->InitConfigFileOperator(XML_ROBOT_DESCRIPTION_ID,
                                                    XML_ROBOT_DESCRIPTION_ID);
            std::deque<std::string> root_queue;
            root_queue.push_back(XML_ROBOT_DESCRIPTION_ID);
            root_queue.push_back(PARAM_KEY_ROBOT_DES_ROBOT_MODEL_NAME);
            //set robot model node
            config_operator->UpdateParam(root_queue, param->robot_model);
            root_queue.pop_back();
            std::string leaf_pre(UNDERLINE_FLAG);
            root_queue.push_back(PARAM_KEY_ROBOT_DES_MECHANICAL_PARAM_NAME);
            config_operator->SetRootQueue(root_queue);
            //find the leaf node to set mechanical param
            int leaf_size = config_operator->GetSpecialNodeNumInOneLevel(root_queue, leaf_pre, true);
            for(int i = 0; i < leaf_size; ++i)
            {
//                LOG_INFO("root_queue = %s", (leaf_pre + NumToString(i) + leaf_pre).c_str());
                root_queue.push_back(leaf_pre + NumToString(i) + leaf_pre);
                root_queue.push_back(PARAM_KEY_ROBOT_DES_WHEEL_BASE_NAME);
                config_operator->UpdateParam(root_queue, param->mechanical_param[i].wheel_base);
                root_queue.pop_back();
                root_queue.push_back(PARAM_KEY_ROBOT_DES_STEERING_WHEEL_OFFSET_NAME);
                config_operator->UpdateParam(root_queue, param->mechanical_param[i].steering_wheel_offset);
                root_queue.pop_back();
                root_queue.push_back(PARAM_KEY_ROBOT_DES_DRIVING_WHEEL_OFFSET_NAME);
                config_operator->UpdateParam(root_queue, param->mechanical_param[i].driving_wheel_offset);
                root_queue.pop_back();
                root_queue.push_back(PARAM_KEY_ROBOT_DES_STEERING_FILTER_RATE_NAME);
                config_operator->UpdateParam(root_queue, param->mechanical_param[i].steering_filter_rate);
                root_queue.pop_back();
                root_queue.push_back(PARAM_KEY_ROBOT_DES_DRIVING_FILTER_RATE_NAME);
                config_operator->UpdateParam(root_queue, param->mechanical_param[i].driving_filter_rate);
                root_queue.pop_back();
                root_queue.pop_back();
            }
            root_queue.pop_back();
//            find the leaf node to set sensor info
            size_t sensor_num = (size_t)SENSORTYPE_NUM;
            for(size_t i = 0; i < sensor_num; ++i)
            {
                std::map<SlamCommon::SensorTypeEnum, std::vector<SensorInfoStru>>::iterator param_it;
                for(param_it = param->sensor.begin(); param_it != param->sensor.end(); ++param_it)
                {
                    if(param_it->first == (SensorTypeEnum)i)
                    {
                        //sensor_num->name
                        root_queue.push_back(param_it->second[0].name);
                        config_operator->SetRootQueue(root_queue);
                        leaf_size = config_operator->GetSpecialNodeNumInOneLevel(root_queue, leaf_pre, true);
                        for(int i = 0; i < leaf_size; ++i)
                        {
                            root_queue.push_back(leaf_pre + NumToString(i) + leaf_pre);
                            //sensor_param_name, type, name
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_TYPE_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].type);
                            root_queue.pop_back();
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_NAME_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].name);
                            root_queue.pop_back();
                            //pose
                            root_queue.push_back("pose");
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_COORD_X_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].coord.x);
                            root_queue.pop_back();
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_COORD_Y_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].coord.y);
                            root_queue.pop_back();
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_COORD_Z_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].coord.z);
                            root_queue.pop_back();
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_COORD_ROLL_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].coord.roll);
                            root_queue.pop_back();
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_COORD_PITCH_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].coord.pitch);
                            root_queue.pop_back();
                            root_queue.push_back(PARAM_KEY_ROBOT_DES_SENSOR_COORD_YAW_NAME);
                            config_operator->UpdateParam(root_queue, param_it->second[i].coord.yaw);
                            root_queue.pop_back();
                            root_queue.pop_back();
                            root_queue.pop_back();
                        }
                        root_queue.pop_back();
                    }
                }
            }
            return;
        }
        virtual std::string FormatParams() override
        {
            std::string msg_str("");
            //robot model
            msg_str += BuildParamMsg(std::string(PARAM_KEY_ROBOT_DES_ROBOT_MODEL_NAME), (robot_model));
            std::string temp_name = std::string(PARAM_KEY_ROBOT_DES_MECHANICAL_PARAM_NAME);
            //mechanicaln param
            for(size_t i = 0; i < mechanical_param.size(); ++i)
            {
                msg_str += BuildParamMsg(temp_name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                         std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                         + std::string(PARAM_KEY_ROBOT_DES_WHEEL_BASE_NAME),
                                         mechanical_param[i].wheel_base);
                msg_str += BuildParamMsg(temp_name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                         std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                         + std::string(PARAM_KEY_ROBOT_DES_STEERING_WHEEL_OFFSET_NAME),
                                         mechanical_param[i].steering_wheel_offset);
                msg_str += BuildParamMsg(temp_name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                         std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                         + std::string(PARAM_KEY_ROBOT_DES_DRIVING_WHEEL_OFFSET_NAME),
                                         mechanical_param[i].driving_wheel_offset);
                msg_str += BuildParamMsg(temp_name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                         std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                         + std::string(PARAM_KEY_ROBOT_DES_STEERING_FILTER_RATE_NAME),
                                         mechanical_param[i].steering_filter_rate);
                msg_str += BuildParamMsg(temp_name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                         std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                         + std::string(PARAM_KEY_ROBOT_DES_DRIVING_FILTER_RATE_NAME),
                                         mechanical_param[i].driving_filter_rate);
            }
            //sensor info
            for(std::map<SlamCommon::SensorTypeEnum, std::vector<SlamCommon::SensorInfoStru>>::iterator
                sensor_it = sensor.begin(); sensor_it != sensor.end(); ++sensor_it)
            {
                std::string name("");
                switch (sensor_it->first)
                {
                case SlamCommon::SENSORTYPE_LASER_SCAN:
                    name = LASER_SCAN_ID;
                    break;
                case SlamCommon::SENSORTYPE_MULTI_ECHO_LASER_SCAN:
                    name = MULTI_ECHO_LASER_SCAN_ID;
                    break;
                case SlamCommon::SENSORTYPE_POINT_CLOUD_2:
                    name = POINT_CLOUD_2_ID;
                    break;
                case SlamCommon::SENSORTYPE_IMU:
                    name = IMU_ID;
                    break;
                case SlamCommon::SENSORTYPE_ODOMETRY:
                    name = ODOMETRY_ID;
                    break;
                case SlamCommon::SENSORTYPE_SATELLITE:
                    name = SATELLITE_ID;
                    break;
                case SlamCommon::SENSORTYPE_LANDMARK:
                    name = LANDMARK_ID;
                    break;
                default:
                    break;
                }
                for(size_t i = 0; i < sensor_it->second.size(); ++i)
                {
                    SlamCommon::SensorInfoStru &info = sensor_it->second[i];
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_TYPE_NAME),
                                             info.type);
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_NAME_NAME),
                                             info.name);
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_X_NAME),
                                             info.coord.x);
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_Y_NAME),
                                             info.coord.y);
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_Z_NAME),
                                             info.coord.z);
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_ROLL_NAME),
                                             info.coord.roll);
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_PITCH_NAME),
                                             info.coord.pitch);
                    msg_str += BuildParamMsg(name + std::string(PARAM_BASE_PARAM_GROUP_SPLIT) +
                                             std::to_string(i) + std::string(PARAM_BASE_PARAM_GROUP_SPLIT)
                                             + std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_YAW_NAME),
                                             info.coord.yaw);
                }
            }
            return msg_str;
        }
        virtual void UpdateParams(const std::string &str) override
        {
            std::string parse_str(str);
            std::string param_root_name(""), param_leaf_name(""), value("");
            size_t group_index = 0;
            while(SplitParamMsg(parse_str, param_root_name, group_index, param_leaf_name, value))
            {
                //robot_model
                if(param_root_name == param_leaf_name &&
                        PARAM_KEY_ROBOT_DES_ROBOT_MODEL_NAME == param_root_name)
                {
                    robot_model = (RobotStructModel)SlamCommon::StringToNum<int>(value);
                    continue;
                }
                //mechanical_param
//                LOG_INFO("%s %d %s = %s", param_root_name.c_str(), group_index,
//                         param_leaf_name.c_str(), value.c_str());
                if(std::string(PARAM_KEY_ROBOT_DES_MECHANICAL_PARAM_NAME) == param_root_name)
                {
                    //===========To do==========================
//                    LOG_INFO("%s = %s", PARAM_KEY_ROBOT_DES_MECHANICAL_PARAM_NAME, param_root_name.c_str());
                    for(size_t i = 0; i < mechanical_param.size(); ++i)
                    {
//                        LOG_INFO("%d ?= %d", i, group_index);
                        if(i == group_index)
                        {
//                            LOG_INFO("%s ?= %s", PARAM_KEY_ROBOT_DES_WHEEL_BASE_NAME,
//                                     param_leaf_name.c_str());
                            if(std::string(PARAM_KEY_ROBOT_DES_WHEEL_BASE_NAME) == param_leaf_name)
                            {
                                mechanical_param[i].wheel_base = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
//                            LOG_INFO("%s ?= %s", PARAM_KEY_ROBOT_DES_STEERING_WHEEL_OFFSET_NAME,
//                                     param_leaf_name.c_str());
                            if(std::string(PARAM_KEY_ROBOT_DES_STEERING_WHEEL_OFFSET_NAME) == param_leaf_name)
                            {
                                mechanical_param[i].steering_wheel_offset = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
//                            LOG_INFO("%s ?= %s", PARAM_KEY_ROBOT_DES_DRIVING_WHEEL_OFFSET_NAME,
//                                     param_leaf_name.c_str());
                            if(std::string(PARAM_KEY_ROBOT_DES_DRIVING_WHEEL_OFFSET_NAME) == param_leaf_name)
                            {
                                mechanical_param[i].driving_wheel_offset = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
//                            LOG_INFO("%s ?= %s", PARAM_KEY_ROBOT_DES_STEERING_FILTER_RATE_NAME,
//                                     param_leaf_name.c_str());
                            if(std::string(PARAM_KEY_ROBOT_DES_STEERING_FILTER_RATE_NAME) == param_leaf_name)
                            {
                                mechanical_param[i].steering_filter_rate = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
//                            LOG_INFO("%s ?= %s", PARAM_KEY_ROBOT_DES_DRIVING_FILTER_RATE_NAME,
//                                     param_leaf_name.c_str());
                            if(std::string(PARAM_KEY_ROBOT_DES_DRIVING_FILTER_RATE_NAME) == param_leaf_name)
                            {
                                mechanical_param[i].driving_filter_rate = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
                            continue;
                        }
                    }
                    continue;
                }
                //sensor_info
                for(std::map<SlamCommon::SensorTypeEnum, std::vector<SlamCommon::SensorInfoStru>>::iterator
                                        sensor_it = sensor.begin();
                                        sensor_it != sensor.end(); ++sensor_it)
                {
                    //获取传感器名称
                    std::string name("");
                    switch (sensor_it->first)
                    {
                    case SlamCommon::SENSORTYPE_LASER_SCAN:
                        name = LASER_SCAN_ID;
                        break;
                    case SlamCommon::SENSORTYPE_MULTI_ECHO_LASER_SCAN:
                        name = MULTI_ECHO_LASER_SCAN_ID;
                        break;
                    case SlamCommon::SENSORTYPE_POINT_CLOUD_2:
                        name = POINT_CLOUD_2_ID;
                        break;
                    case SlamCommon::SENSORTYPE_IMU:
                        name = IMU_ID;
                        break;
                    case SlamCommon::SENSORTYPE_ODOMETRY:
                        name = ODOMETRY_ID;
                        break;
                    case SlamCommon::SENSORTYPE_SATELLITE:
                        name = SATELLITE_ID;
                        break;
                    case SlamCommon::SENSORTYPE_LANDMARK:
                        name = LANDMARK_ID;
                        break;
                    default:
                        break;
                    }
                    //sensor info
                    for(size_t i = 0; i < sensor_it->second.size(); ++i)
                    {
                        //相同分支进行更新
                        if(param_root_name == name && group_index == (i))
                        {
                            if(param_leaf_name == std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_X_NAME))
                            {
                                sensor_it->second[i].coord.x = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
                            if(param_leaf_name == std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_Y_NAME))
                            {
                                sensor_it->second[i].coord.y = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
                            if(param_leaf_name == std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_Z_NAME))
                            {
                                sensor_it->second[i].coord.z = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
                            if(param_leaf_name == std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_ROLL_NAME))
                            {
                                sensor_it->second[i].coord.roll = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
                            if(param_leaf_name == std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_PITCH_NAME))
                            {
                                sensor_it->second[i].coord.pitch = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
                            if(param_leaf_name == std::string(PARAM_KEY_ROBOT_DES_SENSOR_COORD_YAW_NAME))
                            {
                                sensor_it->second[i].coord.yaw = SlamCommon::StringToNum<double>(value);
                                continue;
                            }
                        }
                    }
                }
            }
        }
    };

   CRobotDescription(const CRobotDescription&)=delete;
   CRobotDescription& operator =(const CRobotDescription&)= delete;
   ~CRobotDescription() {}
private:
   CRobotDescription();

public:

    std::unique_ptr<::cartographer::transform::Rigid3d> GetSensorToTrackingTF(
            const std::string &sensor_id);

    int GetSensorInTrackingISO(const SensorTypeEnum &sensor_id,
                               Eigen::Isometry3d &sensor_in_tracking,
                               const std::string &sensor_name = "");
    int GetSensorInTrackingPose3D(const SensorTypeEnum &sensor_id,
                                  SlamCommon::Pose3D &sensor_in_tracking,
                                  const std::string &sensor_name = "");
    int GetSensorInTrackingISO(const SensorTypeEnum &sensor_id,
                               const SensorTypeEnum &tracking_id,
                               Eigen::Isometry3d &sensor_in_tracking,
                               const std::string &sensor_name = "",
                               const std::string &tracking_name = "");
    int GetTrackingInSensorISO(const SensorTypeEnum &sensor_id,
                               Eigen::Isometry3d &tracking_in_sensor)
    {
        Eigen::Isometry3d sensor_in_tracking = Eigen::Isometry3d::Identity();
        int ret = GetSensorInTrackingISO(sensor_id, sensor_in_tracking);
        if(0 != ret)
        {
            ret = 1 + RETURN_FALSE_MAX_NUM * ret;
            return ret;
        }
        tracking_in_sensor = sensor_in_tracking.inverse();
        return 0;
    }
    int GetTrackingInSensorPOSE3D(const SensorTypeEnum &sensor_id,
                                  SlamCommon::Pose3D &tracking_in_sensor)
    {
        SlamCommon::Pose3D sensor_in_tracking = { 0., 0., 0. };
        int ret = GetSensorInTrackingPose3D(sensor_id, sensor_in_tracking);
        if(0 != ret)
        {
            ret = 1 + RETURN_FALSE_MAX_NUM * ret;
            return ret;
        }
        tracking_in_sensor = InverseTransform(sensor_in_tracking);
        return 0;
    }

    //transform sensor pose to global by another sensor pose in global
    Pose3D GetTrackingPoseInGlobalFromSensorPose(const SensorTypeEnum &sensor_id,
                                                 const Pose3D &sensor_in_global)
    {
        Pose3D tracking_in_sensor = {0.0, 0.0, 0.0};
        GetTrackingInSensorPOSE3D(sensor_id, tracking_in_sensor);
        return TransformFromLocalToGlobal(tracking_in_sensor, sensor_in_global);
    }
    Eigen::Isometry3d GetTrackingPoseInGlobalFromSensorPose(
            const SensorTypeEnum &sensor_id,
            const Eigen::Vector3d &laser_in_global_trans,
            const Eigen::Vector3d &laser_in_global_rot)
    {
        Eigen::Isometry3d tracking_in_sensor = Eigen::Isometry3d::Identity();
        GetTrackingInSensorISO(sensor_id, tracking_in_sensor);
        return SlamCommon::TransformFromLocalToGlobal(tracking_in_sensor,
            laser_in_global_trans, laser_in_global_rot);
    }
    Eigen::Isometry3d GetTrackingPoseInGlobalFromSensorPose(
            const SensorTypeEnum &sensor_id,
            const Eigen::Vector3d &laser_in_global_trans,
            const Eigen::Quaterniond &laser_in_global_rot)
    {
        Eigen::Isometry3d tracking_in_sensor = Eigen::Isometry3d::Identity();
        GetTrackingInSensorISO(sensor_id, tracking_in_sensor);
        return SlamCommon::TransformFromLocalToGlobal(tracking_in_sensor,
            laser_in_global_trans, laser_in_global_rot);
    }
    Pose3D GetSensorPoseInGlobalFromTrackingPose(const SensorTypeEnum &sensor_id,
                                                 const Pose3D &tracking_in_global)
    {
        Pose3D sensor_in_tracking = {0.0, 0.0, 0.0};
        GetSensorInTrackingPose3D(sensor_id, sensor_in_tracking);
        return TransformFromLocalToGlobal(sensor_in_tracking, tracking_in_global);
    }
    Eigen::Isometry3d GetSensorPoseInGlobalFromTrackingPose(
            const SensorTypeEnum &sensor_id,
            const Eigen::Vector3d &tracking_in_global_trans,
            const Eigen::Vector3d &tracking_in_global_rot)
    {
        Eigen::Isometry3d sensor_in_tracking = Eigen::Isometry3d::Identity();
        GetSensorInTrackingISO(sensor_id, sensor_in_tracking);
        return SlamCommon::TransformFromLocalToGlobal(sensor_in_tracking,
            tracking_in_global_trans, tracking_in_global_rot);
    }
    Eigen::Isometry3d GetSensorPoseInGlobalFromTrackingPose(
            const SensorTypeEnum &sensor_id,
            const Eigen::Vector3d &tracking_in_global_trans,
            const Eigen::Quaterniond &tracking_in_global_rot)
    {
        Eigen::Isometry3d sensor_in_tracking = Eigen::Isometry3d::Identity();
        GetSensorInTrackingISO(sensor_id, sensor_in_tracking);
        return SlamCommon::TransformFromLocalToGlobal(sensor_in_tracking,
            tracking_in_global_trans, tracking_in_global_rot);
    }
    Eigen::Isometry3d GetSensorPoseInGlobalFromTrackingPose(
            const SensorTypeEnum &sensor_id,
            const SensorTypeEnum &tracking_id,
            const Eigen::Vector3d &tracking_in_global_trans,
            const Eigen::Quaterniond &tracking_in_global_rot)
    {
        Eigen::Isometry3d sensor_in_tracking = Eigen::Isometry3d::Identity();
        GetSensorInTrackingISO(sensor_id, tracking_id, sensor_in_tracking);
        return SlamCommon::TransformFromLocalToGlobal(sensor_in_tracking,
            tracking_in_global_trans, tracking_in_global_rot);
    }

    void GetRobotDescriptionParams(CRobotDescription::ParamsRoboStru &params)
    {
        params = *m_robot_description_params;
    }
    //获取机器人机械结构参数，未指定哪套机械结构参数时，默认取当前设定的那套
    void GetRobotMechanicalParam(RobotMechanicalParam &params, const int index = -1)
    {
        if(index >= 0)
        {
            params = m_robot_description_params->mechanical_param[index];
        }
        else
        {
            params = m_robot_description_params->mechanical_param[m_mechanical_param_index];
        }
    }
    std::string GetSensorDefaultId(const SensorTypeEnum &sensor_type)
    {
        return m_robot_description_params->sensor[sensor_type][0].name;
    }
    int GetSensorName(const SensorTypeEnum &sensor_type, const std::string &sensor_name = "")
    {
        std::vector<SensorInfoStru>::iterator iter
                = m_robot_description_params->sensor[sensor_type].begin();
        if(sensor_name.empty())
        {
            if(m_robot_description_params->sensor[sensor_type].empty())
            {
                // LOG_WARNING("Can't find %s to tracking transform!",
                //             m_sensor_type_map[sensor_type].c_str());
                return 1;
            }
        }
        else
        {
            for(; iter != m_robot_description_params->sensor[sensor_type].end(); ++iter)
            {
                if(sensor_name == iter->name)
                {
                    break;
                }
            }
            if (iter == m_robot_description_params->sensor[sensor_type].end())
            {
                // LOG_WARNING("Can't find %s to tracking transform!", sensor_name.c_str());
                return 2;
            }
        }
        return iter->type;
    }
    SlamCommon::ErrorType GetErrorCode()
    {
        return m_error_code;
    }
    void SetMechanicalParamIndex(const int index)
    {
        m_mechanical_param_index = index;
    }

private:
    Pose3D TransformFromIsoToPose3D(const Eigen::Isometry3d &tf_iso);
    void ReadMechanicalParam(CConfigFileOperator *config_operator);
    void ReadSensorInfo(const SensorTypeEnum &type, CConfigFileOperator *config_operator);
private:
    std::map<SensorTypeEnum, std::string> m_sensor_type_map;
    CRobotDescription::ParamsRoboStru *m_robot_description_params = nullptr;
    int m_mechanical_param_index = 0;

    //error code
    SlamCommon::ErrorType m_error_code = NO_ERROR_CODE;
};
} //namespace SlamCommon

#endif // !ROBOT_DESCRIPTION_H_
