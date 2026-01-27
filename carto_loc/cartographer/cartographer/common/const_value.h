#ifndef CONST_VALUE_H_
#define CONST_VALUE_H_

#include <climits>
#include <cstring>
#include <map>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <utility>  
#include <iomanip> 
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <set>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

#include "slam_time.h"
#include "cartographer/logging/log.h"

namespace SlamCommon
{
#define AGV_PROJECT_NAME                                    ("agv")

#define USE_VELOCITY_ANGLE_FLAG                             1

#define SENSOR_QUEUE_SIZE                                   (2)
#define TRAJECTORY_MAX_POINTS                               (20000)
#define UNKNOWN_VALUE                                       (128)
#define MAX_DB                                              (0xFFFFFFF)
#define VALID_PRECISION                                     (0.00001)
#define ZERO_VELOCITY                                       (0)

#define MAGNETIC_DATA_NUM                                   (2)
#define DRIVE_MOTOR_NUM                                     (4)
#define STEERING_MOTOR_NUM                                  (4)
#define TOTLE_MOTOR_NUM                                     (DRIVE_MOTOR_NUM+STEERING_MOTOR_NUM)

#define RETURN_FALSE_MAX_NUM                                10

#define OLEI_LASER_SCAN                                     ("olelidar")
#define LASER_SCAN_ID                                       ("scan")
#define LASER_SCAN_FRAME_ID                                 ("frame")
#define MULTI_ECHO_LASER_SCAN_ID                            ("echoes")
#define POINT_CLOUD_2_ID                                    ("points2")
#define IMU_ID                                              ("imu")
#define ODOMETRY_ID                                         ("odometry")
#define SATELLITE_ID                                        ("satellite")
#define _2D_CODE_ID                                         ("_2d_code_")
#define LOCALIZATION_MODULE_ID                              ("bosch_rexroth")
#define LANDMARK_ID                                         ("landmark")

#define SENSOR_FRAME_ID                                     ("_frame")

/*----------------------------configuration files----------------------------*/
#define TXT_ID                                              (".txt")
#define PGM_ID                                              (".pgm")
#define XML_ID                                              (".xml")
#define LUA_ID                                              (".lua")

//first level directory
#define DIR_CONFIGURATION_FILE_ID                           ("configuration_file")
#define DIR_DOCS_ID                                         ("docs")
#define DIR_RESOURCE_ID                                     ("res")
#define DIR_ALGORITHM_ID                                    ("algorithm")
#define DIR_ROOT_ID                                         ("root")
#define DIR_AGV_RELEASE_ID                                  ("agv_release") //penghu*
//second level directory
#define DOCS_LOG_ID                                         ("log")
#define DOCS_LASER_HIT_ID                                   ("laser_hit")
#define DOCS_LASER_RANGE_INTENSITY_ID                       ("laser_range_intensity")
#define DOCS_POINT_CLOUD_ID                                 ("point_cloud")
#define DOCS_OBSTACLE_ID                                    ("obstacle")
#define DOCS_RECORD_POSE_ID                                 ("record_pose")
///xcc 0402 code_pose_id
#define DOCS_CODE_POSE_MAP_ID                                 ("code_pose_map")
///
//penghu 24/5/09
#define DOCS_EXPAND_MAP_ID                                  ("expand_map")
#define DOCS_RECORD_SEEDPOSE_ID                             ("record_seedpose")
#define DOCS_MAP_ID                                         ("map")
#define DOCS_MAPPING_TRAJECTORY_ID                          ("mapping_trajectory")
#define DOCS_NAVIGATION_TRAJECTORY_ID                       ("navigation_trajectory")
#define DOCS_OFFLINE_MAPPING_DATA_ID                        ("offline_mapping_data")
#define DOCS_TRAJECTORY_POSE_ID                             ("trajectory_pose")
#define DOCS_DELTA_POSE_ID                                  ("delta_pose")
#define DOCS_TRAJECTORY_POSE_ID                             ("trajectory_pose")
#define DOCS_FIRMWARE_ID                                    ("firmware")
#define DOCS_CALIB_DATA_ID                                  ("calib_data")
#define CFG_LOCALIZATION_ID                                 ("localization")
#define CFG_INTERACTION_ID                                  ("interaction")
#define CFG_MAPPING_ID                                      ("mapping")
#define CFG_CARTO_ORIGINAL_ID                               ("carto_original")
#define CFG_CARTO_EDITED_ID                                 ("carto_edited")
#define CFG_SLAM_COMMON_ID                                  ("slam_common")
#define CFG_COSTMAP_ID                                      ("costmap")
#define CFG_NAVIGATION_ID                                   ("navigation")
#define CFG_DEVICE_ID                                       ("device")
#define CFG_DEVICE_BATTERY_ID                               ("device_battery")
#define CFG_MULTI_AGV_LINKAGE_ID                            ("multi_agv_linkage")
#define CFG_MAIN_TASK_ID                                    ("main_task")
#define CFG_INIT_POSE_ID                                    ("init_pose")
#define CFG_INIT_STATION_POSE_ID                            ("init_station_pose")
//third level directory
#define CFG_SENSORS_IMU_ID                                  ("imu")
#define CFG_SENSORS_LASER_ID                                ("laser")
#define CFG_SENSORS_ODOMETRY_ID                             ("odometry")
#define CFG_SENSORS_SATELLITE_ID                            ("satellite")
#define CFG_SENSORS_LOCALIZATION_MOUDLE_ID                  ("localization_moudle")
#define CFG_SENSORS_TAG_READER_ID                           ("tag_reader")
#define CFG_SENSORS_ROS_ID                                  ("ros")
//xml config name
#define XML_INTERACTIVE_MACHINE_ID                          ("interactive_machine")
#define XML_INTERNAL_SENSOR_ID                              ("internal_sensor")
#define XML_EXTERNAL_SENSOR_ID                              ("external_sensor")
#define XML_LOCALIZATION_INTERFACE_ID                       ("localization_interface")
#define XML_AMCL_ID                                         ("amcl")
#define XML_CARTO_LOCALIZER_ID                              ("carto_localizer")
//penghu*
#define XML_FAST_LOCALIZER_ID                              ("fast_localizer")
#define XML_CARTO_LOCALIZER_2D_ID                           ("carto_localizer_2d")
#define XML_REFLECTOR_LOCALIZER_ID                          ("reflector_localizer")
#define XML_SATELLITE_LOCALIZER_ID                          ("satellite_localizer")
#define XML_2D_CODE_LOCALIZER_ID                          ("2d_code_localizer")
#define XML_NAVIGATION_INTERFACE_ID                         ("navigation_interface")
#define XML_NAVIGATION_ALGORITHM_ID                         ("navigation_algorithm")
#define XML_NAVIGATION_LOCAL_PLANNER_ID                     ("local_planner")
#define XML_PATH_PLANNING_ID                                ("path_planning")
#define XML_A_STAR_PLANNING_ID                              ("a_star_planning")
#define XML_INTERNAL_LOCALIZER_ID                           ("internal_localizer")
#define XML_ODOMETRY_LOCALIZER_ID                           ("odometry_localizer")
#define XML_MAPPING_INTERFACE_ID                            ("mapping_interface")
#define XML_CARTO_MAPPING_INTERFACE_ID                      ("carto_mapping_interface")
#define XML_OFFLINE_DATA_HANDLER_ID                         ("offline_data_handler")
#define XML_MAPPING_BRIDGE_ID                               ("mapping_bridge")
#define XML_MAPPING_DATA_COLLECTOR_ID                       ("mapping_data_collector")
#define XML_MAP_BUILDER_ID                                  ("map_builder")
#define XML_SPARSE_POSE_GRAPH_ID                            ("sparse_pose_graph")
#define XML_TRAJECTORY_BUILDER_2D_ID                        ("trajectory_builder_2d")
#define XML_TRAJECTORY_BUILDER_3D_ID                        ("trajectory_builder_3d")
#define XML_REFLECTOR_MAPPING_ID                            ("reflector_mapping")
#define XML_IMU_INTERFACE_ID                                ("imu_interface")
#define XML_LASER_INTERFACE_ID                              ("laser_interface")
#define XML_ODOMETRY_INTERFACE_ID                           ("odometry_interface")
#define XML_ROBOT_DESCRIPTION_ID                            ("robot_description")
#define XML_REFLECTOR_GRID_POSE_ID                          ("reflector_grid_pose")
#define XML_PURE_REFLECTOR_POSE_ID                          ("pure_reflector_pose")
#define XML_TASK_LIST_ID                                    ("task_rfid_list")
#define XML_COSTMAP_2D_ID                                   ("costmap_2d")
#define XML_STATION_ID                                      ("statioin")
#define XML_AMCL_NAVIGATION_POSE_ID                         ("amcl_navigation_pose")
#define XML_MAPPING_BY_NAV350_ID                            ("mapping_by_nav350")
#define XML_MAINTENANCE_TOOL_ID                             ("maintenance_tool")
#define XML_VERSION_INFO_ID                                 ("version_info")
#define XML_TASK_RFID_LIST_ID                               ("task_rfid_list")
#define XML_ERROR_LIST_ID                                   ("error_list")
#define XML_LASER_LMS5XX_ID                                 ("lms5xx")
#define XML_LASER_LMS1XX_ID                                 ("lms1xx")
#define XML_LASER_TIM2XX_ID                                 ("tim2xx")
#define XML_LASER_OMD30M_R2000_ID                           ("omd30m_r2000")
#define XML_LASER_NAV350_ID                                 ("nav350")
#define XML_LASER_URG_ID                                    ("urg")
#define XML_LASER_LEIMOU_F10_ID                             ("leimou_f10")
#define XML_LASER_NANOSCAN3_ID                              ("nanoscan3")
#define XML_LASER_XINGSONG_ID                              ("xingsong")
#define XML_LASER_VIRTUAL_ID                                ("virtual_laser")
#define XML_LASER_OBS_AVOIDANCE_ID                          ("laser_obs_avoidance")
#define XML_IMU_MTIXXX_ID                                   ("mtixxx")
#define XML_IMU_VIRTUAL_ID                                  ("virtual_imu")
#define XML_ODOMETRY_VIRTUAL_ID                             ("virtual_odometry")
#define XML_BATTERY_INTERFACE_ID                            ("battery_interface")
#define XML_SATELLITE_BEIDOU_ID                             ("beidou")
#define XML_LOCALIZATION_MODULE_ID                          ("localization_module_interface")
#define XML_HIK_TAG_READER_ID                               ("hik_tag_reader")
#define XML_KINEMATICS_MIO_ID                               ("kinematics_mio")
#define NODE_OVERLAP_AREA_ID                                ("overlap_area")

//-----------------------error code------------------
#define ERR_COUNT_TO_REPORT									(5)
#define NO_ERROR_CODE										(0)
#define LOCAL_ERR_SIZE										(20)
#define CONTROLLER_ERR_SIZE									(10)
#define TOTLE_ERROR_SIZE                                    (LOCAL_ERR_SIZE+CONTROLLER_ERR_SIZE)

#define INVALID_STATIC_WORK                                 0xFFFF //无效的静态工作值

typedef unsigned char ErrorType;
enum ErrorCode
{
    //begin flag
    ERROR_BEGIN_FLAG                                      = 51  ,
    //common error
    ERROR_CFG_PARAM_INVALID                                     ,     //config params invalid
    //IMU
    ERROR_IMU_INIT_FAILED                                       ,		//init IMU class failed
    ERROR_IMU_INIT_COM_FAILED                                   ,		//init IMU serial failed
    ERROR_IMU_READ_COM_FAILED                                   ,		//read IMU serial failed
    ERROR_IMU_DATA_INVALID                                      ,		//IMU data invalid
    //laser
    ERROR_LASER_INIT_FAILED                                     ,       //init laser class failed
    ERROR_LASER_INIT_COM_FAILED                                 ,		//init laser socket failed
    ERROR_LASER_COM_DISCONNECT                                  ,       //laser socket disconnect
    ERROR_LASER_READ_COM_FAILED                                 ,		//read laser data from socket failed
    ERROR_LASER_DATA_INVALID                                    ,		//laser data invalid
    ERROR_LASER_OBS_IN_STOP_AREA                                ,		//exist obstacle in laser stop area
    ERROR_LASER_DEVICE_ERROR                                    ,       //laser device error

    ERROR_SATELLITE_INIT_FAILED                                 ,
    ERROR_SATELLITE_NO_DATA                                     ,

    ERROR_LOCALIZATIONMODULE_INIT_FAILED                        ,
    ERROR_LOCALIZATIONMODULE_NO_DATA                            ,

    //multi AGV linkage
    ERROR_LINKAGE_COM_ERROR                                     ,
    ERROR_LINKAGE_COM_DISCONNECT                                ,
    //remote server
    ERROR_REMOTE_SERVER_COM_ERROR                               ,		//remote server communication interrupt
    ERROR_REMOTE_SERVER_COM_DISCONNET                           ,       //remote server socket disconnet
    //controller
    ERROR_CONTROLLER_INIT_COM_FAILED                            ,		//setup controller socket failed
    ERROR_CONTROLLER_COM_DISCONNECT                             ,		//controller socket disconnect
    //localization algorithm
    ERROR_ALG_INIT_FAILED                                       ,
    ERROR_ALG_INIT_SCAN_MATCH_FAILED                            ,
    ERROR_ALG_MAIN_LOCALIZER_FAILED                             ,
    //interactive machine
    ERROR_INTERACTION_X_DERAIL                                  ,
    ERROR_INTERACTION_Y_DERAIL                                  ,
    //firmware
    ERROR_FIRMWARE_READ_FAILED                                  ,
    ERROR_FIRMWARE_UPDATE_FAILED                                ,
    //path planning
    ERROR_PATH_PLAN_GENERATE_FAILED                             ,
    ERROR_PATH_PLAN_TRANSFORM_FORMAT_FAILED                     ,
    //end flag
    ERROR_END_FLAG
};
enum WarningCode
{
    //begin flag
    WARNING_BEGIN_FLAG                                   = 192  ,
    //maintenance tool
    WARNING_MAINTENANCE_UPDATE_TIMEOUT                          ,
    WARNING_MAINTENANCE_DATA_INVALID                            ,
    //localization algorithm
    WARNING_ALG_LASER_SCAN_UPDATE_TIMEOUT                       ,      // laser scan data updating pose timeout
    WARNING_ALG_LASER_POSE_UPDATE_TIMEOUT                       ,      // laser pose data updating pose timeout
    WARNING_ALG_IMU_UPDATE_TIMEOUT                              ,      // imu data updating pose timeout
    WARNING_ALG_ODOMETRY_UPDATE_TIMEOUT                         ,      // odometry data updating pose timeout
    //localization algorithm
    WARNING_MAIN_LOCALIZER_TIMEOUT                              ,
    //laser sensor
    WARNING_LASER_POLLUTION                                     ,
    //multi AGV linkage
    WARNING_LINKAGE_DATA_INVALID                                ,
    WARNING_LINKAGE_UPDATE_TIMEOUT                              ,
    //path planning
    WARNING_PATH_PLAN_AGV_TRAJ_DIFF_ANGLE_TOO_LARGE             ,
    WARNING_PATH_PLAN_TARGET_IS_CURRENT                         ,
    //remote server
    WARNING_REMOTE_SERVER_DATA_INVALID                          ,
    //controller
    WARNING_CONTROLLER_UPDATE_TIMEOUT                           ,     //controller data update period is beyond limit
    WARNING_CONTROLLER_DATA_INVALID                             ,     //controller data invalid or abnormal
    //end flag
    WARNING_END_FLAG
};

enum AlgorithmNameEnum
{
    //--------------定位算法------------------
    ALGNAME_UNKNOWN                     = 0,//算法开始符
    ALGNAME_LOC_AMCL,
    ALGNAME_LOC_AMCL_REFLECTOR,
    ALGNAME_LOC_REFLECTOR_POSE,
    ALGNAME_LOC_REFLECTOR_POINT_CLOUD,
    ALGNAME_LOC_CARTO,
    ALGNAME_LOC_SATELLITE,
    ALGNAME_LOC_2DCODE,
    //以下两种仅是混合导航的两种类别，不是具体算法
    ALGNAME_LOC_HYBRID_AMCL,
    ALGNAME_LOC_HYBRID_REFLECTOR,
    ALGNAME_LOCALIZATION_END,               //算法截止符
    //--------------建图算法------------------
    ALGNAME_MAPPING_BEGIN               = 20,//算法开始符
    ALGNAME_MAPPING_CARTO,
    ALGNAME_MAPPING_REFLECTOR_CARTO,
    ALGNAME_MAPPING_REFLECTOR,
    ALGNAME_MAPPING_NAV350,
    ALGNAME_MAPPING_DATA_COLLECTOR,
    ALGNAME_MAPPING_END                     //算法截止符
};

enum LocalPlannerNameEnum
{
    LOCALPLANNER_PS,
    LOCALPLANNER_BACKSTEPPING,
    LOCALPLANNER_NUM
};

enum LaserSensorNameEnum
{
    LASERNAME_UNKNOWN,
    LASERNAME_LMS1XX,
    LASERNAME_LMS5XX,
    LASERNAME_OMD30M_R2000,
    LASERNAME_NAV350,
    LASERNAME_URG,
    LASERNAME_LEIMOU_F10,
    LASERNAME_NANOSCAN3,
    LASERNAME_TIM2XX,
    LASERNAME_VIRTUAL,
    LASERNAME_XINGSONG,
    LASERNAME_NUM
};

enum IMUSensorNameEnum
{
    IMUNAME_UNKNOWN,
    IMUNAME_MTIXXX,
    IMUNAME_VIRTUAL,
    IMUNAME_NUM
};

enum OdometrySensorNameEnum
{
    ODOMNAME_UNKNOWN,
    ODOMNAME_VIRTUAL,
    ODOMNAME_NUM
};

enum SatelliteEnum
{
    SATELLITENAME_UNKNOWN,
    SATELLITENAME_BEIDOU,
    SATELLITENAME_NUM
};

enum LocalizationModuleEnum
{
    LOCALIZATIONMODULE_UNKNOWN,
    LOCALIZATIONMODULE_BOSCH,
    LOCALIZATIONMODULE_DETOUR,
    LOCALIZATIONMODULE_NUM
};

///xcc 0328 add orientation
enum CodeReaderOri
{
    CodeReader_Down = 0,
    CodeReader_Up =1,
    CodeReader_Forward =2,
    CodeReader_Backward =3,
    CodeReader_Right =4,
    CodeReader_Left =5,
    CodeReader_Custom_1 =6,
    CodeReader_Custom_2 =7,
    CodeReader_Custom_3 =8,
    CodeReader_Custom_4 =9,
};

struct Header
{
    Header()
        :time(SlamCommon::Time::min()), frame_id(""), frequency(0)
    {}
    Header(const Header &other)
        :time(other.time), frame_id(other.frame_id), frequency(other.frequency)
    {}
    void Reset()
    {
        time = SlamCommon::Time::min();
        frame_id = "";
        frequency = 0.;
    }

    //为如果一次扫描周期中存在多次扫描，比如激光一个周期包括多帧激光束，则该时间戳应该为第一帧数据接收到的时间
    SlamCommon::Time time;
    SlamCommon::Time time1;
    //对应于robot_description中的坐标系
    std::string frame_id;
    //数据采集频率
    double frequency;
};

struct Point2D
{
    Point2D() : x(0.), y(0.) {}
    Point2D(const double x_in, const double y_in) : x(x_in), y(y_in) {}
    Point2D(const Point2D &other) : x(other.x), y(other.y) {}

    void Reset()
    {
        x = 0.;
        y = 0.;
    }
    void Reset(const double new_x, const double new_y)
    {
        x = new_x;
        y = new_y;
    }
    double x;
    double y;

    Point2D operator+(const Point2D &t) const
    {
        Point2D sum;
        sum.x = x + t.x;
        sum.y = y + t.y;
        return sum;
    }
    Point2D operator-(const Point2D &t) const
    {
        Point2D sub;
        sub.x = x - t.x;
        sub.y = y - t.y;
        return sub;
    }
    bool operator==(const Point2D &other)
    {
        if (x == other.x && y == other.y)
        {
            return true;
        }
        return false;
    }
};

struct PolarPoint
{
    PolarPoint() : length(0.), angle(0.) {}
    PolarPoint(const double length_in, const double angle_in) : length(length_in), angle(angle_in) {}
    PolarPoint(const PolarPoint &other) : length(other.length), angle(other.angle) {}

    void Reset()
    {
        length = 0.;
        angle = 0.;
    }
    void Reset(const double new_len, const double new_angle)
    {
        length = new_len;
        angle = new_angle;
    }
    void Reset(const PolarPoint &in)
    {
        length = in.length;
        angle = in.angle;
    }
    double length;
    double angle;
};

struct Pose3D
{
    Pose3D() : x(0.), y(0.), theta(0.) {}
    Pose3D(const double x_in, const double y_in, const double theta_in)
        : x(x_in), y(y_in), theta(theta_in) {}
    Pose3D(const Pose3D &other) : x(other.x), y(other.y), theta(other.theta) {}

    void Reset()
    {
        x = 0.;
        y = 0.;
        theta = 0.;
    }
    void Reset(const Pose3D &new_pt)
    {
        x = new_pt.x;
        y = new_pt.y;
        theta = new_pt.theta;
    }
    void Reset(const double new_x, const double new_y, const double new_theta)
    {
        x = new_x;
        y = new_y;
        theta = new_theta;
    }
    void ResetXY(const double new_x, const double new_y)
    {
        x = new_x;
        y = new_y;
    }

    double x;
    double y;
    double theta;

    Pose3D operator+(const Pose3D &t) const
    {
        Pose3D sum;
        sum.x = x + t.x;
        sum.y = y + t.y;
        sum.theta = theta + t.theta;
        sum.theta = atan2(sin(sum.theta), cos(sum.theta));//normalize
        return sum;
    }
    Pose3D operator-(const Pose3D &t) const
    {
        Pose3D sub;
        sub.x = x - t.x;
        sub.y = y - t.y;
        sub.theta = theta - t.theta;
        sub.theta = atan2(sin(sub.theta), cos(sub.theta));//normalize
        return sub;
    }
    bool operator==(const Pose3D &other)
    {
        if (x == other.x && y == other.y && theta == other.theta)
        {
            return true;
        }
        return false;
    }
    bool operator !=(const Pose3D &other)
    {
        if ( fabs(x - other.x) > VALID_PRECISION
           ||fabs(y - other.y) > VALID_PRECISION
           ||fabs(theta - other.theta) > VALID_PRECISION)
        {
            return true;
        }
        return false;
    }
};

struct Pose3DWithCov
{
    Pose3D pose;
    Pose3D covariance;
};

struct Pose6DVec2
{
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
};

struct Pose6DVec2WithCov
{
    Pose6DVec2 pose;
    Pose6DVec2 covariance;
};

struct Pose6DVecQua
{
    Pose6DVecQua()
        : translation(Eigen::Vector3d::Zero()), rotation(Eigen::Quaterniond::Identity())
    {}
    Pose6DVecQua(const Eigen::Vector3d &trans, const Eigen::Quaterniond &rot)
        : translation(trans), rotation(rot)
    {}
    Pose6DVecQua(const Pose6DVecQua &pose)
        : translation(pose.translation), rotation(pose.rotation)
    {}
    void Reset()
    {
        translation = Eigen::Vector3d::Zero();
        rotation = Eigen::Quaterniond::Identity();
    }
    void Reset(const Eigen::Vector3d &trans, const Eigen::Quaterniond &rot)
    {
        translation = trans;
        rotation = rot;
    }
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
};

struct Pose6DVecQuaWithCov
{
    Pose6DVecQua pose;
    Pose6DVec2 covariance;
};

struct Pose6DVecQuaStamped
{
    Pose6DVecQuaStamped(){}
    Pose6DVecQuaStamped(const Pose6DVecQuaStamped &p)
        : header(p.header), pose(p.pose)
    {}
    Pose6DVecQuaStamped(const Header &h, const Pose6DVecQua &p)
        : header(h), pose(p)
    {}
    void Reset()
    {
        header.Reset();
        pose.Reset();
    }
    void Reset(const Header &h, const Pose6DVecQua &p)
    {
        header = h;
        pose = p;
    }
    void Reset(const Header &h, const Eigen::Vector3d &trans, const Eigen::Quaterniond &rot)
    {
        header = h;
        pose.Reset(trans, rot);
    }
    Header header;
    Pose6DVecQua pose;
};

struct Pose6DVecQuaStampedWithCov
{
    Pose6DVecQuaStamped pos_stamp;
    Pose6DVec2 covariance;
};

struct SlamTransformStamped
{
    Header header;
    std::string child_frame_id; // the frame id of the child frame
    Pose6DVecQua transform;
};

//圆形
struct CircleStru
{
    void Reset()
    {
        center.Reset();
        radius = 0.;
    }
    Point2D center;
    double radius = 0.;
};

//扇形
struct SectorStru
{
    void Reset()
    {
        center.Reset();
        radius = 0.;
        angle = 0.;
    }
    Point2D center;
    double radius = 0.;
    double angle = 0.;
};

//平面激光测距仪的单次扫描
class SlamLaserScanData
{
public:
    SlamLaserScanData()
        : sensor_id(""), angle_min(0.), angle_max(0.), angle_increment(0.),
          time_increment(0.), scan_time(0.), range_min(0.), range_max(0.)
    {}
    SlamLaserScanData(const SlamLaserScanData &other)
        : header(other.header), sensor_id(other.sensor_id), angle_min(other.angle_min),
          angle_max(other.angle_max), angle_increment(other.angle_increment),
          time_increment(other.time_increment), scan_time(other.scan_time),
          range_min(other.range_min), range_max(other.range_max)
    {
        ranges.resize(other.ranges.size());
        ranges.assign(other.ranges.begin(), other.ranges.end());
        intensities.resize(other.intensities.size());
        intensities.assign(other.intensities.begin(), other.intensities.end());
    }

    void CopyBasicInfo(const SlamLaserScanData &other)
    {
        header = other.header;
        sensor_id = other.sensor_id;
        angle_min = other.angle_min;
        angle_max = other.angle_max;
        angle_increment = other.angle_increment;
        time_increment = other.time_increment;
        scan_time = other.scan_time;
        range_min = other.range_min;
        range_max = other.range_max;
    }
    void CopyRanges(const SlamLaserScanData &other)
    {
        ranges.resize(other.ranges.size());
        ranges.assign(other.ranges.begin(), other.ranges.end());
    }
    void CopyIntensities(const SlamLaserScanData &other)
    {
        intensities.resize(other.intensities.size());
        intensities.assign(other.intensities.begin(), other.intensities.end());
    }

public:
    Header header;
    std::string sensor_id;
    double angle_min;           //单位：rad
    double angle_max;           //单位：rad
    double angle_increment;     //单位：rad
    double time_increment;      //新增，每帧激光束之间的间隔，单位：秒；主要用于激光移动时，三维点的位置插值，目前未赋值
    double scan_time;           //新增，每次完整扫描之间的间隔，目前未赋值
    double range_min;           //单位：m
    double range_max;           //单位：m
    std::vector<double> ranges; //单位：m
    std::vector<unsigned int> intensities;
};

//一层激光回波扫描数据
struct SlamLaserEchoData
{
    std::vector<double> echoes = std::vector<double>();//单位：m
};

//多回波平面激光测距仪的单次扫描
struct SlamMultiEchoLaserScanData
{
    Header header;
    std::string sensor_id = "";
    double angle_min = 0.;//单位：rad
    double angle_max = 0.;//单位：rad
    double angle_increment = 0.;//单位：rad
    double time_increment = 0.;//新增，每帧激光束之间的间隔，单位：秒；主要用于激光移动时，三维点的位置插值，目前未赋值
    double scan_time = 0.;//新增，每次完整扫描之间的间隔，目前未赋值
    double range_min = 0.;//单位：m
    double range_max = 0.;//单位：m
    std::vector<SlamLaserEchoData> ranges = std::vector<SlamLaserEchoData>();//单位：m
    std::vector<SlamLaserEchoData> intensities = std::vector<SlamLaserEchoData>();
};

struct SlamPointFieldData
{
    enum DataType
    {
        INT8    = 1,
        UINT8   = 2,
        INT16   = 3,
        UINT16  = 4,
        INT32   = 5,
        UINT32  = 6,
        FLOAT32 = 7,
        FLOAT64 = 8
    };
    std::string name;
    uint32_t offset;//从点结构起点的偏移
    DataType datatype;//数据类型
    uint32_t count;//本区域元素的数量
};

//此消息包含n维点的集合，其中可能包含法线、强度等附加信息。点数据存储为二进制blob，其布局由“fields”数组的内容描述。
//点云数据可以组织为2d(类图)或1d(无序)。以2d图像组织的点云可以由立体或飞行时间等相机深度传感器生成。
struct SlamPointCloud2Data
{
    Header header;
    //点云的2D结构。如果云是无序的，则高度为1，宽度为点云的长度。
    uint32_t height = 0;
    uint32_t width = 0;
    //描述二进制数据blob中的通道及其布局。
    std::vector<SlamPointFieldData> fields;

    bool is_bigendian = false;
    uint32_t point_step = 0;    //以字节算，一个点所占的长度
    uint32_t row_step = 0;       //以字节算，一行的长度
    std::vector<unsigned char> data;  //实际点数据，大小=row_step*height

    bool is_dense = true;       //如果没有无效点，则为真
};

/**********************************
 * pose_error_code
 * 0 = no error
 * 1 = wrong operation mode
 * 2 = asynchrony method terminated
 * 3 = invalid data
 * 4 = no position available
 * 5 = timeout
 * 6 = method already active
 * 7 = general error
 * 8 = no response to sMA command
 * 9 = no response to sAN command
 * 10 = not enough reflectors
 * 11 = unknown error
 * *******************************/
struct SlamLaserPoseData
{
    SlamLaserPoseData()
        : sensor_id(""), pose_error_code(0)
    {}
    SlamLaserPoseData(const SlamLaserPoseData &other)
        : header(other.header), sensor_id(other.sensor_id),
          pose(other.pose), pose_error_code(other.pose_error_code)
    {
        reflectors_pose.resize(other.reflectors_pose.size());
        reflectors_pose.assign(other.reflectors_pose.begin(), other.reflectors_pose.end());
    }
    Header header;
    std::string sensor_id;
    Pose3D pose;
    std::vector<Pose3D> reflectors_pose;
    int pose_error_code;
};

struct SlamNavSatStatus
{
    //是否输出一个增强修正是由修正类型和最后收到的时间差分修正决定的。当status >= STATUS_FIX时，修复是有效的。
    enum Status
    {
        STATUS_NO_FIX =  -1,    //无法修正位置
        STATUS_FIX =      0,    //非增强修正
        STATUS_SBAS_FIX = 1,    //以卫星为基础的增强修正
        STATUS_GBAS_FIX = 2     //以地面为基础的增强修正
    };
    Status status;

    //定义接收机使用哪个全球导航卫星系统信号的位。
    enum Service
    {
        SERVICE_GPS =     1,
        SERVICE_GLONASS = 2,
        SERVICE_COMPASS = 4,    //包括北斗
        SERVICE_GALILEO = 8
    };
    Service service;
};

//针对任意全球导航卫星系统的导航卫星修正，指定使用WGS 84参考椭球面
struct SlamNavSatFixData
{
    //header.time指定本次测量的时间(并不是对应的卫星时间)。
    //header.frame_id是卫星接收器的参考坐标系，通常是天线的位置。这是一个相对于飞行器的欧几里德坐标系，而不是一个参考椭球体。
    Header header;
    //卫星定位状态信息
    SlamNavSatStatus status;

    //纬度，单位：度。赤道以北为正，以南为负
    double latitude = 0.;
    //经度，单位：度。本初子午线以东为正，以西为负
    double longitude = 0.;
    //高度，单位：m。WGS 84椭球面之上为正(如果没有可用高度，则为无穷大)。
    double altitude = 0.;
    //位置协方差[单位：m^2]定义为相对于一个过报告位置的切平面。组件是东、北和上(ENU)，按行为主维度。
    //注意:这个坐标系在极点处表现出奇异性。
    double position_covariance[9] = { 0. };

    //如果修正的协方差已知，则将其完全填充。
    //如果GPS接收器提供了每个测量值的方差，将它们沿对角线放置。
    //如果只有精度因子可用，估计一个近似协方差。
    enum CovType
    {
        COVARIANCE_TYPE_UNKNOWN = 0,
        COVARIANCE_TYPE_APPROXIMATED = 1,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
        COVARIANCE_TYPE_KNOWN = 3
    };
    CovType position_covariance_type = COVARIANCE_TYPE_UNKNOWN;
};

//IMU (Inertial Measurement Unit)数据
//加速度的单位应该是m/s^2(而不是g)，转速的单位应该是rad/s
//如果测量的协方差已知，则应填写(如果您只知道每个测量的方差，例如从数据表中，只需将其沿对角线填写即可)
//所有0的协方差矩阵将被解释为“协方差未知”，要使用协方差数据，就必须假设或从其他来源获得协方差
//如果对其中一个数据元素没有估计值(例如，IMU不输出朝向估计值)，请将相关协方差矩阵的元素0设置为-1
//如果您正在解释此消息，请检查每个协方差矩阵的第一个元素中的值-1，并忽略相关的估计。
struct SlamIMUData
{
    SlamIMUData()
        : sensor_id(""),
          orientation(Eigen::Quaterniond::Identity()),
          orientation_covariance(Eigen::Matrix3d::Zero()),
          euler_angle(Eigen::Vector3d::Zero()),
          angular_velocity(Eigen::Vector3d::Zero()),
          angular_velocity_covariance(Eigen::Matrix3d::Zero()),
          linear_acceleration(Eigen::Vector3d::Zero()),
          linear_acceleration_covariance(Eigen::Matrix3d::Zero())
    {}

    SlamIMUData(const SlamIMUData &data)
        : header(data.header),
          sensor_id(data.sensor_id),
          orientation(data.orientation),
          orientation_covariance(data.orientation_covariance),
          euler_angle(data.euler_angle),
          angular_velocity(data.angular_velocity),
          angular_velocity_covariance(data.angular_velocity_covariance),
          linear_acceleration(data.linear_acceleration),
          linear_acceleration_covariance(data.linear_acceleration_covariance)
    {}

    Header header;
    std::string sensor_id;
    Eigen::Quaterniond orientation;
    Eigen::Matrix3d orientation_covariance;
    Eigen::Vector3d euler_angle;
    Eigen::Vector3d angular_velocity;
    Eigen::Matrix3d angular_velocity_covariance;
    Eigen::Vector3d linear_acceleration;
    Eigen::Matrix3d linear_acceleration_covariance;
};

struct LaserScanPoseStru {
    SlamCommon::Time timestamp;
    SlamCommon::Time start_time;
    SlamCommon::Time end_time;
    double T;
    double x;
    double y;
    double yaw;
};

struct OdomPoseStru {
    SlamCommon::Time timestamp;
    double x;
    double y;
    double yaw;
    double v_l;
    double v_r;
    Eigen::Vector3d linear_v;
    Eigen::Vector3d rotate_v;
};

class SlamOdometryData
{
public:
    virtual ~SlamOdometryData() {}
    Header header;
    std::string sensor_id = "";
};

class SlamOdometryPoseData : public SlamOdometryData
{
public:
    Pose6DVec2 pose;
};

class SlamFrontOneSteeringDriverBackTwoFixedDiffWheelData : public SlamOdometryData
{
public:
    double steering_angle = 0.;
    double wheel_velocity = 0.;
};

class SlamFrontTwoSteeringWheelBackTwoDriverData : public SlamOdometryData
{
public:
    double steering_angle[2] = { 0. };
    double wheel_velocity[2] = { 0. };
};

class SlamFrontTwoSteeringDriverBackTwoFixedDiffWheelData : public SlamOdometryData
{
public:
    double steering_angle[2] = { 0. };
    double wheel_velocity[2] = { 0. };
};

class SlamTwoDiagonalSteeringDriverData : public SlamOdometryData
{
public:
    double steering_angle[2] = { 0. };
    double wheel_velocity[2] = { 0. };
};

class SlamFourDriverDiffDriverData : public SlamOdometryData
{
public:
    Pose6DVec2 pose;
    //penghu* Q
       double x11 = 0.;
       double x12 = 0.;
       double yaw1 = 0.;

       double x21 = 0.;
       double x22 = 0.;
       double yaw2 = 0.;

       double x31 = 0.;
       double x32 = 0.;
       double yaw3 = 0.;

       double x41 = 0.;
       double x42 = 0.;
       double yaw4 = 0.;

       double yaw = 0.;

};

class SlamFrontOneSteeringWheelBackOneDriverData : public SlamOdometryData
{
public:
    double steering_angle = 0.;
    double wheel_velocity = 0.;
};

class SlamFrontTwoMecanumWheelBackTwoMecanumWheel : public SlamOdometryData
{
public:
    double wheel_velocity[4] = { 0. };
};

class SlamFourDrivingWheel : public SlamOdometryData
{
public:
    double wheel_velocity[4] = { 0. };  //0 代表左前轮角速度 1 代表右前轮角速度 2 代表 右后角速度 3 左后角速度 
    double wheel_angle[4] = { 0. };//0 代表左前轮线速度 1 代表右前轮线速度 2 代表 右后线速度 3 左后线速度 
};

class SlamDoubleDiffWheel : public SlamOdometryData
{
public:
    double wheel_velocity[2] = { 0. };//0: 左轮；1：右轮
};

class SlamControlCenterVel : public SlamOdometryData
{
public:
    double line_velocity_x = 0.0;
    double line_velocity_y = 0.0;
    double angle_velocity = 0.0;
    //penghu* Q
    double acc_x = 0.0;
    double acc_y = 0.0;
    double w = 0.0;

    double l1 = 0.0;
    double r1 = 0.0;
    double angle1 = 0.0;

    double l2 = 0.0;
    double r2 = 0.0;
    double angle2 = 0.0;

    double l3 = 0.0;
    double r3 = 0.0;
    double angle3 = 0.0;

    double l4 = 0.0;
    double r4 = 0.0;
    double angle4 = 0.0;

    double yaw = 0.0;
};

enum MapTypeEnum
{
    MAPTYPE_OCCU_GRID,
    MAPTYPE_REFLECTOR,
    MAPTYPE_NUM
};

class MapData
{
public:
    virtual ~MapData() {}

    Header header;
    std::string map_name = "";
    Pose3D origin;
};

//二维网格地图，地图中每个栅格的灰度值表示占有的概率。
class OccupancyGrid : public MapData
{
public:
    OccupancyGrid(): cells(std::vector<char>()), resolution(0.), width(0), height(0),
        occupied_threash(0.), free_threash(0.), negate(false)
    {}
    virtual ~OccupancyGrid()
    {
        cells.clear();
    }
    //地图数据，以行为主维度的顺序，从(0,0)开始。栅格占有概率在[0,100]之间，未知为-1。
    std::vector<char> cells;
    double resolution;
    unsigned int width;
    unsigned int height;
    double occupied_threash;
    double free_threash;
    bool negate;
    SlamCommon::Time map_load_time;
};

enum MotionTypeEnum
{
    MTYPE_NONE,
    MTYPE_STOP,
    MTYPE_LINE,
    MTYPE_CURVE,
    MTYPE_SPIN,
    MTYPE_WORK,
    MTYPE_SHIFT,
    MTYPE_S_TURN,
    MTYPE_U_TURN,
    MTYPE_ADJUST,
    MTYPE_CHARGE,
    ///0821xcc 增加standby状态用于跳过这个motiontype的动作，应用于地图切换
    //MTYPE_STANDBY,
    MTYPE_AUTORUN_LINE,
    MTYPE_AUTORUN_CURVE,
    MTYPE_AUTORUN_SPIN,
    ///0323 xcc add MTYPE_SELF_ADJUST
    //MTYPE_SELF_ADJUST,
    ///
    MTYPE_WAIT,
    MTYPE_STEPWAIT,
    MTYPE_MAPCHANGE,
    MTYPE_RELOCAL,
    MTYPE_ALIGNMENT,
    MTYPE_NUM,
};

enum ServerMotionTypeEnum
{
    SMTYPE_STOP = 0,    //停车
    SMTYPE_LINE,        //直线
    SMTYPE_CURVE,       //转弯
    SMTYPE_SPIN,        //自旋
    SMTYPE_SHIFT,       //侧移
    SMTYPE_GET,         //取货
    SMTYPE_PUT,         //放货
    SMTYPE_S_TURN,      //S弯道
    SMTYPE_U_TURN,      //U弯道
    SMTYPE_ADJUST,      //调整
    SMTYPE_CHARGE,      //充电
    SMTYPE_PARKING,     //库位停车
};

enum StandbyOpeTpyeEnum
{
    SOTYPE_WAIT =0,
    SOTYPE_MAPCHANGE,
    SOTYPE_RELOCAL,
    SOTYPE_ALIGNMENT,
};

enum WaitEnum
{
    TIME_WAITING =0,
    STEP_WAITING,
};

enum SafeLaserSectionEnum
{
    SAFELASERSEC_DEFAULT,
    SAFELASERSEC_FORWARD,
    SAFELASERSEC_BACKWARD,
    SAFELASERSEC_LEFT_SHIFT,
    SAFELASERSEC_RIGHT_SHIFT,
    SAFELASERSEC_LEFT_SPIN,
    SAFELASERSEC_RIGHT_SPIN,
    SAFELASERSEC_LEFT_FRONT,
    SAFELASERSEC_RIGHT_FRONT,
    SAFELASERSEC_LEFT_BACK,
    SAFELASERSEC_RIGHT_BACK,
    SAFELASERSEC_DJUST,
    SAFELASERSEC_SMALL_AREA = 15,
    SAFELASERSEC_NUM
};

enum NaviAlgoTypeEnum
{
    NAVITYPE_TRAJECTORY,
    NAVITYPE_VISION,
    NAVITYPE_2D_CODE,
    NAVITYPE_NUM
};

enum LoadStatusEnum
{
    LOADSTATUS_NONE,
    LOADSTATUS_LOADED,
    LOADSTATUS_UNLOAD,
    LOADSTATUS_NUM
};

enum State_Code_reader
{
    NO_REQUIREMENT,
    NO_SUITABLE_CAMERA,
    CAMERA_ERR,
    FIND_NO_CODE,
    CODEID_FIT_NO_POSE,
    CODEID_FIT_POSE

};
struct AdjustMotionInfo
{
    int Sen_type =-1;
    uint32_t CodeID;
    int adj_type =0;
    int motion_method =0;
    float limit_x = 0.0;
    float limit_y = 0.0;
    Pose3D adj_pose;

    void Reset()
    {
        Sen_type =-1;
        CodeID = 0;
        adj_type =0;
        motion_method =0;
        limit_x = 0.0;
        limit_y = 0.0;
        adj_pose.Reset();
    }

};

struct PathNode
{
    //ph_spin0912
    int16_t heading_spin;
    int station_id = 0;
    int tcs_cmd = 0;
    int charge_cmd = 0;
    int id = 0;
    int node_cnt = 0;
    MotionTypeEnum motion_type = MTYPE_NONE;
    MotionTypeEnum next_motion_type = MTYPE_NONE;
    double static_work1 = 0.;
    double static_work2 = 0.;
    double target_vel = 0.;
    double next_vel = 0.;
    double vel_angle = 0.;//速度角度，从路线转向车身的夹角
    bool is_virtual = false;
    bool external_detect = false;
    LoadStatusEnum load_status = LOADSTATUS_NONE;
    int16_t done_wait_time = 0.;
    SafeLaserSectionEnum obs_area = SAFELASERSEC_NUM;
    NaviAlgoTypeEnum navi_type = NAVITYPE_TRAJECTORY;
    int map_name_index = -1;
    AlgorithmNameEnum localizer_type = SlamCommon::ALGNAME_UNKNOWN;
    bool valid_turn_sector = false;
    SectorStru turn_sector;
    //0618-----------
    double code_distance = 0.;
    int code_id = 0;
    int cargosize = 0;
    //1220 AUTORUN
    int autorunState =0;
    //1223 standby operate
    int waiting_time =0;//和int16的done_wait_time是否一致？？
    Pose3D Target_pose;
    //penghu* 2024/2/18
    double parking_distance_ = 0;
    double switching_distance_ = 0;
    double node_distance_ = 0;
    bool is_parking_node_ = false;
    //penghu 24/6/1
    int disc_angle_ = 1;

    //map Id 使用map_name_Index
    //自我调节使用单独结构体
    AdjustMotionInfo Adj_info;
    //
    bool operator !=(const PathNode &other) const
    {
        if(id != other.id || motion_type != other.motion_type
                || fabs(static_work1 - other.static_work1) > 0.005
                || fabs(static_work2 - other.static_work2) > 0.005
                || fabs(target_vel - other.target_vel) > VALID_PRECISION
                || fabs(next_vel - other.next_vel) > VALID_PRECISION
                || fabs(vel_angle - other.vel_angle) > VALID_PRECISION
                || fabs(turn_sector.angle - other.turn_sector.angle) > VALID_PRECISION
                || fabs(turn_sector.radius - other.turn_sector.radius) > VALID_PRECISION
                || obs_area != other.obs_area)
        {
            return true;
        }
        return false;
    }
    void Reset()
    {
        id = 0;
        node_cnt = 0;
        motion_type = MTYPE_NONE;
        next_motion_type = MTYPE_NONE;
        static_work1 = 0.;
        static_work2 = 0.;
        target_vel = 0.;
        next_vel = 0.;
        vel_angle = 0.;
        turn_sector.angle = 0.;
        turn_sector.radius = 0.;
        turn_sector.center.Reset();
        is_virtual = false;
        load_status = SlamCommon::LOADSTATUS_NONE;
        obs_area = SAFELASERSEC_NUM;
        navi_type = NAVITYPE_TRAJECTORY;
        map_name_index = 0;
        localizer_type = SlamCommon::ALGNAME_UNKNOWN;
        valid_turn_sector = false;
        //guoqiang 2021/12/8
        cargosize = 0;
        // 1220 autorun
        autorunState =0;
        waiting_time =0;
        Adj_info.Reset();
        Target_pose.Reset();
    }
};

enum LandmarkSpecialPart
{
    PartUnknown,
    PartPoseAndNode,
    PartOrigin,
    PartPoseAndNodeID,
    PartNode,
    PartPoseAndNodeIDAndType,
    PartNodeIDAndType
};

class ReflectorMapData : public MapData
{
public:
    std::vector<Pose3D> pose = std::vector<Pose3D>();	//reflector pose
    //special part 1
    int task_id = 0;
    std::vector<double> dis_bias = std::vector<double>();
    std::vector<PathNode> node = std::vector<PathNode>();
    //special part 2, from the base class
// 		Pose3D origin;
    //for localization algorithm
    std::vector<int> probability = std::vector<int>();
};

struct SlamLandmarkObservationData
{
public:
    std::string id = "";
    Pose6DVecQua landmark_to_tracking_transform;
    double translation_weight = 0.;
    double rotation_weight = 0.;
};

class SlamLandmarkData : public MapData
{
public:
    std::vector<SlamLandmarkObservationData> landmarks;
    std::string sensor_id = "";
};

union UNION_IntAndFloat
{
    int i;
    unsigned int ui;
    float f;
};

union UNION_CharAndErrorType
{
    unsigned char uc[sizeof(ErrorType)];
    ErrorType err_t;
};

union UNION_LongAndChar
{
    long l;
    char c[sizeof(long)];
    unsigned char uc[sizeof(long)];
};

union UNION_2Byte
{
    uint16_t ui16;
    int16_t i16;
    char c[sizeof(short)];
    unsigned char uc[sizeof(unsigned short)];
    short s;
    unsigned short us;
};

union UNION_4Byte
{
    uint8_t ui8[4];
    int8_t i8[4];
    uint16_t ui16[2];
    int16_t i16[2];
    uint32_t ui32;
    int32_t i32;
    float f;
    char c[4];
    unsigned char uc[4];
    short s[2];
    unsigned short us[2];
};

union UNION_8Byte
{
    uint8_t ui8[8];
    int8_t i8[8];
    uint16_t ui16[4];
    int16_t i16[4];
    uint32_t ui32[2];
    int32_t i32[2];
    uint64_t ui64;
    int64_t i64;
    char c[8];
    unsigned char uc[8];
    short s[4];
    unsigned short us[4];
};

struct ReflectorPoints
{
    //all laser hits
    std::vector<PolarPoint> polar_points = std::vector<PolarPoint>();
    std::vector<Point2D> cartesian_points = std::vector<Point2D>();
    double radius = 0.;
    //one laser hit standing for reflector
    Pose3D cartesian_pose;
    PolarPoint polar_pose;

    bool operator<(const ReflectorPoints& other) const
    {
        return polar_points[(unsigned int)(0.5 * polar_points.size())].length
            < other.polar_points[(unsigned int)(0.5 * other.polar_points.size())].length;
    }
};

struct ReflectorItem
{
    Pose3D pose;//accurate reflector cartesian poses in global coords
    PolarPoint scan_hit;//the polar coords pose of scan hits in laser coords

    bool operator<(const ReflectorItem &other) const
    {
        return scan_hit.length < other.scan_hit.length;
    }
};

struct ReflectorItems
{
    Pose3D pose;//accurate reflector cartesian poses in global coords
    PolarPoint scan_hit;//the polar coords pose of scan hits in laser coords

    double dist_bias = 0.0;

    bool operator<(const ReflectorItems &other) const
    {
        return scan_hit.length < other.scan_hit.length;
    }
};

struct MatchedReflectorItems
{
    ReflectorItem reflector;            //反光棒在地图中的准确位置
    SlamCommon::Pose3D origin_reflector; //实际采集的反光棒原始全局位置
    int matched_index = -1; // matching reflector index in map
    int matched_count = 0;
    double dist_bias = 0.0;//the distance between detected reflectors and reflectors in map
    bool operator<(const MatchedReflectorItems &other) const
    {
        //choose the reflector, which has minimun offset from existing reflectors
//			return dist_bias < other.dist_bias;
        //choose the reflector, which is neareast to AGV
        return reflector < other.reflector;
    }
};

//robot structure model
enum RobotStructModel
{
    RSModel_None,
    RSModel_FrontOneSteeringDriverBackTwoFixedDiffWheel,
    RSModel_FrontTwoSteeringWheelBackTwoDriver,
    RSModel_FrontTwoSteeringDriverBackTwoFixedDiffWheel,
    RSModel_TwoDiagonalSteeringDriver,
    RSModel_FourDriverDiffDriver,
    RSModel_FrontOneSteeringWheelBackOneDriver,
    RSModel_FrontTwoMecanumWheelBackTwoMecanumWheel,
    RSModel_FourDrivingWheel,
    RSModel_DoubleDiffWheel,
    RSModel_ControlCenterVel,
    RSModel_StructSize
};

///xcc 0524
enum MotionAbilityModel
{

    MAModel_FULLORI_SPIN,
    MAModel_FOURORI_SPIN,
    MAModel_TWOORI_SPIN,
    MAModel_FULLORI,
    MAModel_TWOORI,
    MAModel_FOURORI,
    MAModel_None
};
///

struct TaskScheduleStru
{
    int current = 0;
    int totle = 0;
};

struct SensorDeviceInfoStru
{
    int laser_pose_freq = 0;
    int laser_scan_freq = 0;
    int imu_freq = 0;
    int odometry_freq = 0;
    Pose3D euler_angle;
    Pose3D angular_v;
    Pose3D linear_v;
    //penghu*
    double image_width;
    double image_height;
    double data_size;
};

enum ProgramVersionEnum
{
    PROGRAMVERSION_NONE,
    PROGRAMVERSION_RELEASE,
    PROGRAMVERSION_MAPPING,
    PROGRAMVERSION_DEBUG,
    PROGRAMVERSION_NUM
};

enum ConnectStateEnum
{
    CONNSTATE_DISCONNECT,
    CONNSTATE_WAIT_FOR_CONNECT,
    CONNSTATE_CONNECTED,
};

enum NodeInsertTypeEnum
{
    NITYPE_KEEP,
    NITYPE_INSERT,
    NITYPE_REPLACE,
};

//保存附近相连的节点属性
struct PathPlanNodeInfoStru
{
    int id = 0;                                         //节点ID
    ServerMotionTypeEnum motion_type = SMTYPE_STOP;     //向该节点运行的运行类型
    double vel_angle = 2 * M_PI;                        //速度角度
    double turn_radius = 0.;                            //转弯半径，如果是自旋，设置为0
    int static_work_index = 0;                          //作动器标准值下标
    bool left_orientation = true;                       //取/放货出库后AGV朝向，朝向从库位由内向外方向的左侧还是右侧，true是左侧，false是右侧
    bool compensatory = false;                          //是否是补偿的新增节点
    double target_vel = 100.;                           //路段运行速度
};

struct NodeInsertToBaseStru
{
    int area_id = 0;        // 工位点所在的区域ID
    int station_id = 0;     // 工位点ID
    NodeInsertTypeEnum insert_type = NITYPE_INSERT;
    int insert_index = 0;  // 目标操作节点，在该节点前插入（如果插在最后，则该值设成-1），或者替换该节点
    PathPlanNodeInfoStru node_info;

    bool operator < (const NodeInsertToBaseStru &other) const
    {
        return area_id < other.area_id;
    }
};

enum AlgStatusEnum
{
    ALGSTATUS_UNKNOWN,
    ALGSTATUS_INIT,
    ALGSTATUS_RUNNING,
    ALGSTATUS_PAUSE,
    ALGSTATUS_STOP,
    ALGSTATUS_FAILED,
    ALGSTATUS_NUM
};

struct ColorRGBA
{
    double r = 0.;
    double g = 0.;
    double b = 0.;
    double a = 0.;
};

enum ObsAreaTypeEnum
{
    AREATYPE_POLYGON,
    AREATYPE_SECTOR,
    AREATYPE_NUM
};

enum LaserObsStatusEnum
{
    LASEROBS_STOP,
    LASEROBS_SLOWER,
    LASEROBS_SLOW,
    LASEROBS_NORMAL,
    LASEROBS_NUM
};

struct ObsAreaItemStru
{
    std::vector<SlamCommon::Pose3D> end_pts;                //检测区域的角点
    bool exist_obs = false;                                 //是否存在障碍物
    SlamCommon::Time exist_time = SlamCommon::Time::min();  //首次检测到障碍物的时间
    //guoqiang 12.22
    int state;                                              //area类型  0急挺 1缓停  2 缓速
};

struct ObsGroupItemStru
{
    bool active = true;                                     //是否激活该避障区域
    ObsAreaTypeEnum type = AREATYPE_POLYGON;                //检测区域的类型
    double min_detect_angle = 0.;                           //避障区域所有角点在激光坐标系下的最小角度
    double max_detect_angle = 0.;                           //避障区域所有角点在激光坐标系下的最大角度
    SlamCommon::Time detect_time = SlamCommon::Time::min(); //上一次检测时间
    std::vector<ObsAreaItemStru> areas;                     //避障区域队列
    LaserObsStatusEnum obs_status = LASEROBS_NORMAL;        //避障状态
};


template <class T>
T StringToNum(const std::string& str_in)
{
    std::istringstream iss(str_in);
    T num;
    iss >> num;
    return num;
}

//new class
struct ParamBaseStru
{
#define PARAM_BASE_DIR_PARAM_SPLIT      "$"
#define PARAM_BASE_PARAM_SPLIT          "|"
#define PARAM_BASE_PARAM_INTER_SPLIT    ":"
#define PARAM_BASE_PARAM_GROUP_SPLIT    "-"
#define PARAM_BASE_PARAM_DIR_SPLIT    "/"
    ParamBaseStru() {}
    virtual ~ParamBaseStru() {}

    static bool SplitDirAndParams(std::string &str, std::string &dir, std::string &param)
    {
        //提取name和value
        size_t index = str.find_first_of(PARAM_BASE_DIR_PARAM_SPLIT);
        if(static_cast<int>(index) < 0)
        {
            return false;
        }
        dir = str.substr(0, index);
        param = str.substr(index + 1, str.length());
        return true;
    }
    template<typename T>
    std::string BuildParamMsg(const std::string &name, T value)
    {
        std::stringstream ss;
        ss << name << PARAM_BASE_PARAM_INTER_SPLIT << value << PARAM_BASE_PARAM_SPLIT;
        return ss.str();
    }
    static bool SplitParamMsg(std::string &str, std::string &param_root_name, size_t &param_group,
                              std::string &param_leaf_name, std::string &value)
    {
        //提取name和value
        param_group = 0;
        param_root_name.clear();
        param_leaf_name.clear();
        value.clear();
        size_t index = str.find_first_of(PARAM_BASE_PARAM_SPLIT);
        if(static_cast<int>(index) < 0)
        {
            return false;
        }
        std::string str_temp = str.substr(0, index);
        //删除已解析的部分
        str = str.substr(index + 1, str.length());
        //分解name和value
        index = str_temp.find_first_of(PARAM_BASE_PARAM_INTER_SPLIT);
        if(static_cast<int>(index) < 0)
        {
            return false;
        }
        std::string temp_name = str_temp.substr(0, index);
        value = str_temp.substr(index + 1, str_temp.length());
        //temp name = "robot_mechanical_params-0-wheel_base", need to continue splitting
        index = temp_name.find_first_of(PARAM_BASE_PARAM_GROUP_SPLIT);
        //if there is no "-", for example "robot_model", let root = leaf = temp_name
        if(static_cast<int>(index) < 0)
        {
            param_root_name = temp_name;
            param_leaf_name = temp_name;
            param_group = 0;
            return true;
        }
        size_t group_num_start = index + 2;
        param_root_name = temp_name.substr(0, index);
        index = temp_name.find_last_of(PARAM_BASE_PARAM_GROUP_SPLIT);
        if(static_cast<int>(index) < 0)
        {
            return false;
        }
        size_t group_num_end = index;
        if(group_num_end - group_num_start > 2)
        {
            param_leaf_name = temp_name.substr((group_num_start + 1),
                                               (group_num_end)-(group_num_start + 1));
            param_group = SlamCommon::StringToNum<size_t>(temp_name.substr(group_num_end + 1,
                                                                           temp_name.length()));
//            LOG_INFO("param_group = %d, param_leaf_name = %s, group_num_start = %d, group_num_end = %d, "
//                     "temp_name.length() = %d, temp_name = %s",
//                     param_group, param_leaf_name.c_str(), group_num_start, group_num_end,
//                     temp_name.length(), temp_name.c_str());
            return true;
        }
        else
        {
            param_leaf_name = temp_name.substr(index + 1, temp_name.length());
            param_group = SlamCommon::StringToNum<size_t>(temp_name.substr(group_num_start,
                                                                           group_num_end-(group_num_start+1)));
            return true;
        }
    }
    virtual std::string FormatParams() = 0;
    virtual void UpdateParams(const std::string &str) = 0;
//    virtual void SetSensorParam(size_t sensor_num) = 0;
};

enum ConfigParamFileNameEnum
{
    CONFIG_PARAM_FILE_ROBOT_DESCRIPTION = 1,
    CONFIG_PARAM_FILE_NUM
};

//global variable
extern RobotStructModel g_robot_struct_model;
extern std::string g_task_list_name;
extern std::map<MotionTypeEnum, std::string> g_motion_type_map;
extern std::map<ServerMotionTypeEnum, std::string> g_server_motion_type_map;
extern SlamCommon::LaserSensorNameEnum g_laser_lidar_name;
extern std::map<AlgorithmNameEnum, std::string> g_alg_name_map;
extern std::map<AlgStatusEnum, std::string> g_alg_status_map;
extern std::map<ConnectStateEnum, std::string> g_connect_state_map;
extern std::map<NodeInsertTypeEnum, std::string> g_node_insert_type_map;



///xcc 0331 PoseMap Struct
struct CodePoseMap
{
    int list_size;
    int last_search_Id;
    //int Class_Id;//class of input id
    std::vector<int> HundId;
    std::vector<long> codeId;
    std::vector<Pose3D> agv_pose;

    void reset()
    {
        list_size=0;
        last_search_Id=0;
        std::vector<int>().swap(HundId);
        std::vector<long>().swap(codeId);
        std::vector<Pose3D>().swap(agv_pose);
    }
    int push_codePosePar(long ID, Pose3D agv_pose_input)
    {
        list_size++;
        int IdHund = int(ID/100%10);
        if (HundId.size()==0||HundId.back()!=IdHund)
        {
            // LOG_INFO("read pose_map file find IdHund %d",IdHund);
            HundId.push_back(IdHund);
        }
        codeId.push_back(ID);
        agv_pose.push_back(agv_pose_input);
        return list_size;
    }
    bool check_code(long ID)
    {
        for (int i=0;i<HundId.size();i++)
        {
            if (HundId[i]==int(ID/100%10))
            {
                return true;
            }
        }
        return false;
    }
    bool search_pose_byId(long ID,Pose3D &agv_pose_output)
    {
        if((last_search_Id < (codeId.size()-1))&&(last_search_Id >0))
        {
            if (ID==codeId[last_search_Id+1])
            {
                agv_pose_output = agv_pose[last_search_Id+1];
                last_search_Id = last_search_Id+1;
                return true;
            }
            if(ID==codeId[last_search_Id-1])
            {
                agv_pose_output = agv_pose[last_search_Id-1];
                last_search_Id = last_search_Id-1;
                return true;
            }
        }

        for (int i=0;i<list_size;i++)
        {
            if (codeId[i]==ID)
            {
                agv_pose_output = agv_pose[i];
                last_search_Id = i;
                return true;
            }
        }
        return false;
    }
};
///
} // namespace SlamCommon

#endif // !CONST_VALUE_H_
