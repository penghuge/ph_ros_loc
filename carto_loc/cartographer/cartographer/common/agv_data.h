#ifndef AGV_DATA_H
#define AGV_DATA_H

#include "stdint.h"
#include <vector>
#include <deque>
#include <string>
#include <map>
#include "agv_error_code.h"
#include "const_value.h"

namespace MainControl {

//#define __CNT_ADD__(n)  (n) = ((n)++) % 1000
#define __CNT_ADD__(n)  if((n)<1000){(n)++;}else{(n)=1;}

#define __MIO_PROTO_ACTUATOR_NUM__                5 //协议中执行机构数量
#define __MIO_PROTO_MOTOR_NUM__                   4 //协议中电机数量

enum AGV_STATUS_TYPE
{
    STATUSTYPE_NONE = 0,   //no any status, no connection with remote scheduling, or agv is power off
    STATUSTYPE_IDLE,       //agv is online but has no task
    STATUSTYPE_BUSY,       //agv is doing the task
    STATUSTYPE_CHARGE,     //agv charging
    STATUSTYPE_FAULT,      //agv is broken down
    STATUSTYPE_OFFLINE,    //agv is connected with remote scheduling, but no apply for online
};

enum AGV_STOP_TYPE
{
    STOPTYPE_NONE = 0,      //no stop
    STOPTYPE_WAIT_TASK,     //stop caused by no task                        ---->显示的优先级 8
    STOPTYPE_WAIT_START,    //stop caused by no start cmd                   ---->显示的优先级 7
    STOPTYPE_FAULT,         //stop caused by AGV error                      ---->显示的优先级 6
    STOPTYPE_MULTI_AGV,     //stop caused by multi-agv stop command         ---->显示的优先级 5
    STOPTYPE_CHARGE,        //stop caused by charge                         ---->显示的优先级 4
    STOPTYPE_BUTTON,        //stop caused by stop button                    ---->显示的优先级 3
    STOPTYPE_SCHE,          //stop caused by scheduling stop command        ---->显示的优先级 2
    STOPTYPE_MANUAL,        //stop caused by switching to manual-mode       ---->显示的优先级 1
};

enum AGV_MODE_TYPE
{
    MODETYPE_NONE = 0,
    MODETYPE_MANUAL,
    MODETYPE_AUTO,
};

enum BATTERY_STATUS
{
    BATTERY_STANDBY = 0,
    BATTERY_DISCHARGE = 1,
    BATTERY_CHARGE = 2,
};

struct GPIO_INPUT_STRU
{
    bool m_InitFinish = false;
    bool m_Start = false;
    bool m_Stop = true;
    bool m_Reset = false;
    bool m_Auto = false;
    bool m_PressStartBtn = false;

    std::vector<unsigned short> m_GPIOErrorCodeVector;
};

enum MIO_TOP_STATUS
{
    MIO_Init = 0,
    MIO_InitFailed,
    MIO_M_Mode,
    MIO_AutoRun
};

enum MIO_AUTO_STATUS
{
    MIO_Stopped,
    MIO_RUN,
    MIO_HardStop
};

enum TransferStatus
{
    UNLOAD_FINISH = 0,
    LEFT_ROLLING,
    RIGHT_ROLLING,
    LOAD_FINISH
};


struct GPIO_OUTPUT_STRU
{
    bool m_IndicatorRun = false;
    bool m_IndicatorError = false;
    bool m_IndicatorWarning = false;
    bool m_Buzzer = false;
};

enum MIO_OUTPUT_NAV_MODE_ENUM
{
    NAV_MODE_LOCALIZATION_INIT,
    NAV_MODE_LOCALIZATION_NORMAL,
    NAV_MODE_LOCALIZATION_VIRTUAL,
    NAV_MODE_LOCALIZATION_STOP,
    NAV_MODE_LOCALIZATION_FAIL,
};

enum MIO_OUTPUT_LOCALIZATION_ERROR_CODEP_ENUM
{
    ERROR_CODE_NORMAL,
    ERROR_CODE_FAIL,
};

enum MIO_OUTPUT_ALGORITHM_MODE_ENUM
{
    SYNCHRONOUS_MODE,
    ASYNCHRONOUS_MODE,
};

enum MIO_OUTPUT_SAFETY_INFRINGMENT_ENUM
{
    INVAD_NONE,
    INVAD_OUTERSPACE,
    INVAD_MIDDLESPACE,
    INVAD_INNERSPACE,
};


struct POSE_STRU
{
    POSE_STRU()
    {
        this->m_X = 0.;
        this->m_Y = 0.;
        this->m_Theta = 0.;
    }
    POSE_STRU(const double x, const double y, const double theta)
    {
        this->m_X = x;
        this->m_Y = y;
        this->m_Theta = theta;
    }
    POSE_STRU(const POSE_STRU &p)
    {
        this->m_X = p.m_X;
        this->m_Y = p.m_Y;
        this->m_Theta = p.m_Theta;
    }
    double m_X;
    double m_Y;
    double m_Theta;
};

struct MIO_CMD_STRU
{
    uint32_t cnt = 0;
    uint32_t code = 0;
    uint32_t param1 = 0;
    uint32_t param2 = 0;
    uint32_t param3 = 0;
};

struct MIO_SERIAL_STRU
{
    int localization_error_code;
    int algorithm_mode;
    int nav_mode;
    int safety_zone;
    int safety_infringment;
    uint32_t mean_dev = 0;
    uint32_t timestamp = 0;
    int reflector_nums = 0;
};

//for debug
struct TaskNodeStru
{
    uint32_t id = 0;
    float x = 0.;
    float y = 0.;
    uint32_t motion_type;
    float static_work1 = 0.f;
    float static_work2 = 0.f;
    float target_vel = 0.f;
    float vel_angle = 0.f;
    uint32_t navi_type = 0;
    int map_name_index = 0;
    uint32_t localizer_type = 0;
    int valid_turn_sector = 0;
    float turn_center_x = 0;
    float turn_center_y = 0;
    float turn_radius = 0;
    int reserve1 = 0;
    int reserve2 = 0;
    int reserve3 = 0;
    int reserve4 = 0;
    //0618----
    float code_distance = 0.f;
    int code_id = 0;
};

struct DebugTaskListStru
{
    int32_t map_id = -1;
    int32_t task_id = -1;
    TaskNodeStru *task_node = nullptr;	//node property
};

enum MultiAGVTypeEnum
{
    MULTIAGV_NONE,
    MULTIAGV_MASTER,
    MULTIAGV_SLAVE,
    MULTIAGV_NUM
};

struct BasicRunInfoToMaintenace_STRU
{
    struct
    {
        //算法类
        unsigned char alg_name = 0;
        unsigned char alg_status = 0;
        unsigned char alg_map = 0;
        unsigned char reflector_num = 0;

        unsigned char node_id = 0;
        unsigned char node_index = 0;
        unsigned char next_node_id = 0;
        unsigned char motion_type = 0;

        unsigned char laser_scan_frequecy = 0;
        unsigned char laser_pose_frequecy = 0;
        unsigned char imu_frequecy = 0;
        unsigned char odometry_frequecy = 0;

        unsigned char task_id = 0;
        unsigned char task_size = 0;
        unsigned char task_index = 0;
        //0: MIO; 1: laser; 2: IMU; 3: battery; 4: scheduling; 5: maintenance; 6: master; 7: slave
        unsigned char device_conn = 0;

        unsigned char mio_top_status = 0;
        unsigned char mio_auto_status = 0;
        unsigned char battery_status = 0;//电池状态, 0->休眠, 1->放电, 2->充电
        unsigned char obs_area = 0;

        unsigned char wait_confirm = 0;//需要确认的事项，0：无；1：新建/追加地图
        unsigned char multi_agv_type = 0;//0：未知；1：主AGV；2：从AGV
        unsigned char stop_type = 0;//停车类型
        unsigned char remain1 = 0;

        unsigned char remain2 = 0;
        unsigned char remain3 = 0;
        unsigned char remain4 = 0;
        unsigned char remain5 = 0;

        unsigned char remain6 = 0;
        unsigned char remain7 = 0;
        unsigned char remain8 = 0;
        unsigned char remain9 = 0;
    }flag;
    //AGV位姿
    float pose_x = 0.f;
    float pose_y = 0.f;
    float pose_theta = 0.f;
    //平均位姿
    float average_x = 0.f;
    float average_y = 0.f;
    float average_theta = 0.f;
    //位姿偏差
    float delta_x = 0.f;
    float delta_y = 0.f;
    float delta_theta = 0.f;
    //协同偏差
    float multi_agv_delta_x = 0.f;
    float multi_agv_delta_y = 0.f;
    float multi_agv_delta_theta = 0.f;
    //任务信息
    float node_x = 0.f;
    float node_y = 0.f;
    float target_v = 0.f;
    float next_v = 0.f;
    float vel_angle = 0.f;
    float static_work1 = 0.f;
    float static_work2 = 0.f;
    float turn_angle = 0.f;
    float turn_radius = 0.f;
    float run_time = 0.f;
    //电池
    float battery_vol = 0;//电池电压
    float battery_soc = 0;//电池电量
    struct
    {
        float v1 = 0.f;
        float a1 = 0.f;
        float v2 = 0.f;
        float a2 = 0.f;
        float v3 = 0.f;
        float a3 = 0.f;
        float v4 = 0.f;
        float a4 = 0.f;
    }odometry_value;
    struct
    {
        float vx = 0.f;
        float vy = 0.f;
        float w = 0.f;
    }imu_value;
    struct
    {
        float vx = 0.f;
        float vy = 0.f;
        float w = 0.f;
    }agv_vel;
    struct
    {
        float vx = 0.f;
        float vy = 0.f;
        float w = 0.f;
    }expect_ctrl_vel;
    struct
    {
        float vx = 0.f;
        float vy = 0.f;
        float w = 0.f;
    }real_ctrl_vel;
    //预留
    uint32_t remain1 = 0;
    uint32_t remain2 = 0;
    uint32_t remain3 = 0;
    uint32_t remain4 = 0;
    uint32_t remain5 = 0;
    uint32_t remain6 = 0;
    uint32_t remain7 = 0;
    uint32_t remain8 = 0;
    uint32_t remain9 = 0;
    uint32_t remain10 = 0;
};

enum DeviceList_ENUM
{
    DEV_MIO,
    DEV_LASER,
    DEV_IMU,
    DEV_BATTERY,
    DEV_SCHEDULE,
    DEV_MAINTENANCE,
    DEV_MASTER,
    DEV_SLAVE,
    DEV_NUM
};

enum KeyBoardCtrl_ENUM
{
    KEY_NONE = 0,                       //无效
    KEY_V = 1,                          //速度设置，单位：m/s
    KEY_LF = 50, KEY_F, KEY_RF,         //左前，前，右前
    KEY_L, KEY_S, KEY_R,                //左，停车，右
    KEY_LB, KEY_B, KEY_RB,              //左后，后，右后
    KEY_LS, KEY_RS,                     //左旋，右旋
    KEY_SLF, KEY_SRF, KEY_SLB, KEY_SRB, //左前转，右前转，左后转，右后转
    KEY_LIFT_UP = 80, KEY_LIFT_DOWN,    //上升、下降
    KEY_CLAMP_ON, KEY_CLAMP_OFF,        //抱夹，松夹
    KEY_ROLLER_F, KEY_ROLLER_B,         //前滚，后滚
    KEY_ROLLER_L, KEY_ROLLER_R,         //左滚，右滚
    KEY_ACT_STOP                        //执行机构停止
};

struct KeyBoardCtrl_STRU
{
    KeyBoardCtrl_ENUM key_id = KEY_NONE;
    unsigned char force_mannual = 0; //0: none; 1: true; 2: false
    float vel = 0.f;
    SlamCommon::LoadStatusEnum load_status = SlamCommon::LOADSTATUS_NONE;
    float turn_radius = 0.f;
    bool update = false;
};

//struct SERVER_INPUT_STRU
//{
//         unsigned char m_RunCmd = 0;   // 0: 未知; 1: 启动; 2: 停车
//         bool m_RunCmdUpdate = false;
//         unsigned char m_ChargeCmd = 0;  // 0: 未知; 1: 充电; 2: 结束充电
//         bool m_EnableMultiAGV = false;
//         bool m_RebootSystem = false;
//         std::vector<AGV_ERROR_CODE> m_ServerErrorVector;
//         SlamCommon::ConnectStateEnum m_ConnectStatus = SlamCommon::CONNSTATE_WAIT_FOR_CONNECT;
//         KeyBoardCtrl_STRU m_KeyCtrl;
//         unsigned char m_NewInitLocalization = 0;   //0：空，1：初始定位站点；2：初始定位坐标
//         int m_InitStationID = -1;       //初始定位站点
//            SlamCommon::Pose3D m_InitCoord; //初始定位坐标
//         int m_AgvId = 0;
//};

//struct SERVER_OUTPUT_STRU
//{
//    int m_CurNodeId = 0;
//    int m_NextNodeId = 0;
//    std::vector<AGV_ERROR_CODE> m_AGVErrorCodeVector;
//    char m_RunStatus = 0;   // 启停状态，0: 未知; 1: 启动; 2: 停车
//    AGV_STATUS_TYPE m_AGVStatus = STATUSTYPE_NONE; //AGV状态
//    float m_BatterySoc = 0.f;
//    float m_BatteryVol = 0.f;
//    POSE_STRU m_PoseStru;
//    POSE_STRU m_AGVVel = { 0., 0., 0. };
//    double m_TurnAngle = 0.;
//};

struct ServerOutputData
{
    //tcs and public server data
    int cur_node_id = 0;
    std::vector<AGV_ERROR_CODE> error_codes;
    AGV_STATUS_TYPE agv_status = STATUSTYPE_NONE; //AGV状态
    float battery_soc = 0.f;
    SlamCommon::Pose3D agv_pose;
    SlamCommon::LoadStatusEnum load_status = SlamCommon::LOADSTATUS_NONE;


    //tcs server data
    int task_id = -1;
    SlamCommon::Pose3D delta_pose;
    SlamCommon::Pose3D agv_vel;
    double static_work1 = 0.;
    double static_work2 = 0.;
    double task_cost_time = 0.;
    AGV_STOP_TYPE agv_stop_type = STOPTYPE_NONE;
    double steering_angle[__MIO_PROTO_MOTOR_NUM__] = { 0. };
    bool agv_start = false;
    bool task_done = false;
    double run_time = 0.;
    MIO_AUTO_STATUS mio_auto_status = MIO_Stopped;
    MIO_TOP_STATUS mio_top_status = MIO_Init;
    int com_quantity;                               //通讯质量
    int localization_quantity;                      //定位质量


    // public server data
    int next_node_id = 0;
    char run_status = 0;   // 启停状态，0: 未知; 1: 启动; 2: 停车
    float battery_vol = 0.f;
    int roller_status = 0;
    unsigned char release_cmd = 0;

    double turn_angle = 0.;
    //0728--add map_id data from mc to the struct
    int map_id = -1;
    bool WaitToTcs = false;
    //240615 kmy
    int lift_angle = 0;
};

struct ServerIntputData
{
    //tcs and public server data
    unsigned char run_cmd = 0;   // 0: 未知; 1: 启动; 2: 停车
    bool run_cmd_update = false;
    unsigned char charge_cmd = 0;  // 0: 未知; 1: 充电; 2: 结束充电
    //penghu* for ipc_plc
    unsigned char park_cmd = 0;

    std::vector<AGV_ERROR_CODE> error_codes;
    SlamCommon::ConnectStateEnum connect_status = SlamCommon::CONNSTATE_WAIT_FOR_CONNECT;
    int agv_id = 0;

    //public server data
    unsigned char new_init_localization = 0;   //0：空，1：初始定位站点；2：初始定位坐标
    int init_station_id = -1;       //初始定位站点
    SlamCommon::Pose3D init_coord; //初始定位坐标

    bool enable_multi_agv = false;
    bool reboot_system = false;
    KeyBoardCtrl_STRU key_ctrl;
    //0909----add tr_cmd to mc
    int16_t tr_cmd_to_mc = 0;
    int16_t task_status_to_loc = 0;

};

struct MAINTENANCE_OUTPUT_STRU
{
    BasicRunInfoToMaintenace_STRU m_BasicRunInfo;
};

struct NAVI_INPUT_STRU
{
    double                          m_DeltaX = 0.;
    double                          m_DeltaY = 0.;
    double                          m_DeltaT = 0.;
    int                             m_NodeIdx = 0;
    int                             m_CurNodeId = 0;
    int                             m_CurTaskId = 0;
    int                             m_LaserScanFreq = 0;
    SlamCommon::SectorStru          m_TurnSector;
    SlamCommon::NaviAlgoTypeEnum    m_NaviType = SlamCommon::NAVITYPE_TRAJECTORY;
    SlamCommon::Pose3D              m_NaviVel;
    bool                            m_TaskDone = true;

    std::vector<unsigned char>  m_ErrorVector;
    //0909----add navi_m_windshowerdoor_flag for set bosch renit
    ///xcc 1201
    //int                             m_windshowerdoor_flag = 0;
    ///
    /// \brief m_mechanism_flag
    ///xcc 1201
    //int                             m_mechanism_flag = 0;
    ///
    bool                            m_rexroth_flag = false;
    bool                            m_WaitToTcs = false;
    ///xcc 1206 taskEnd & reset Odom
    bool                            ResetOdom = false;
    ///
    /// xcc 0411
    int Code_Read_State = 0.;
    //penghu* Q
    double m_P_delta = 0.;
    double m_L_delta = 0.;
    double m_remain_angle = 0.;

};

struct LOCALIZER_INPUT_STRU
{
    double                              m_CurrentX = 0;
    double                              m_CurrentY = 0;
    double                              m_CurrentT = 0;
    ///xcc 1118 AUTORUN
    double                              m_OdomX = 0;
    double                              m_OdomY = 0;
    double                              m_OdomT = 0;
    ///
    int                                 m_map_id = -1;
    POSE_STRU                           m_AveragePose = { 0., 0., 0.};
    std::string                         m_MapName = "";
    SlamCommon::AlgorithmNameEnum       m_AlgoName = SlamCommon::ALGNAME_UNKNOWN;
    SlamCommon::AlgStatusEnum           m_AlgoStatus = SlamCommon::ALGSTATUS_UNKNOWN;
    SlamCommon::SensorDeviceInfoStru    m_SensorInfo;

    std::vector<unsigned char>          m_ErrorVector;
    bool                                m_reloc_reach = false;
    //penghu* Q
    double                              m_laser_x = 0;
    double                              m_laser_y = 0;
    double                              m_laser_t = 0;
    //penghu 24/5/22
    double                             localization_score = 0;
};

enum MANUAL_CTRL_ACTION
{
    MANUAL_STOP = 0,
    MANUAL_FORWARD,
    MANUAL_BACKWARD,
    MANUAL_LEFT,
    MANUAL_RIGHT,
    MANUAL_LEFTFORWARD,
    MANUAL_RIGHTFORWARD,
    MANUAL_LEFTBACKWARD,
    MANUAL_RIGHTBACKWARD,
};

struct MANUAL_CTRL_INPUT_STRU
{
    MANUAL_CTRL_ACTION m_ManualAction = MANUAL_STOP;
    double m_ManualSpeed = 0.1;

    std::vector<AGV_ERROR_CODE> m_ManualCtrlErrorVector;
};

struct HMI_RUN_DATA_STRU
{
    double m_ActAGVVelocity = 0;                                //AGV actual velocity
    double m_ActWheelAngle = 0;                                 //wheel actual angle
    double m_StaticWork1 = 0;                                    //static work
    double m_StaticWork2 = 0;                                    //static work
    double m_CurrPosX = 0;                                      //current pose x
    double m_CurrPosY = 0;                                      //current pose y
    double m_CurrPosAng = 0;                                    //current pose angle
    double m_DeltaPosX = 0;                                     //delta pose x
    double m_DeltaPosY = 0;                                     //delta pose y
    double m_DeltaPosAng = 0;                                   //delta pose angle
    double m_TurnAngle = 0.;                                    //turn angle
    double m_TurnRadius = 0.;                                   //turn radius
    SlamCommon::AlgorithmNameEnum m_AlgoName = SlamCommon::ALGNAME_UNKNOWN;//算法名
    SlamCommon::AlgStatusEnum m_AlgoStatus = SlamCommon::ALGSTATUS_UNKNOWN;//算法状态
    std::string m_MapName = "";
    bool m_Online = false;                                      //status of scheduling connection
    int  m_SignalLevel = 0;                                     //信号强度
    bool m_ExeStatus = false;                                   //status of ready to do the task
    AGV_STOP_TYPE m_StopType = STOPTYPE_WAIT_TASK;              //type of agv-stop-status
    int m_NodeIndex = 0;                                        //节点下标
    int m_TaskSize = 0;                                         //任务总量
    int m_TaskIndex = 0;                                        //任务进度，当任务完成时，与m_TaskSize相等
    AGV_STATUS_TYPE m_AGVStatus = STATUSTYPE_NONE;              //AGV任务状态
};
struct HMI_DEV_DATA_STRU
{
    float m_BatteryVolt = 0;                                    //voltage of battery
    float m_BatterySOC = 0;                                     //SOC of battery
    unsigned int m_BatteryStatus = 0;                           //status of battery, 0 for sleep, 1 for discharge, 2 for charge
    bool m_StartButton = false;                                 //signal of start button
    bool m_ResetButton = false;                                 //signal of reset button
    bool m_AutoButton = false;                                  //signal of auto button
    bool m_ImpactSignal = false;                                //signal of impact
    int m_SafeLaserSignal = 0;                                  //0 for running, 1 for stop or deceleration
};
struct HMI_MAPPING_DATA_STRU
{
    bool m_StartMapping = false;
    bool m_SaveMap = false;
    std::string m_MapName = "";
    bool m_DeleteMap = false;
};
struct HMI_OPERATION_DATA_STRU
{

};
struct HMI_CONFIG_DATA_STRU
{
    int m_AGVID = 0;
    int m_WarningLightBlock = 0;
    int m_BuzzerBlock = 0;
    int m_SafeLaserBlock = 0;
    int m_PhotoelectricityBlock = 0;
    int m_SafeSwitchForce = 0;
    int m_Charge = 0;
};

struct HMI_INPUT_STRU
{
    /*-- HMI to main_task --*/
//    HMI_OPERATION_DATA_STRU m_HMIOperationDataStru;
    HMI_CONFIG_DATA_STRU m_HMIConfigDataStru;
    HMI_MAPPING_DATA_STRU m_HMIMappingDataStru;
    bool m_NewConfigData = false;
    bool m_AGVOnlineCmd = false;
    bool m_ClearTaskCmd = false;
    char m_AGVRunCmd = 0;

    unsigned int    m_RxCnt = 0;
};

struct HMI_OUTPUT_STRU
{
    /*-- main_task to HMI --*/
    HMI_RUN_DATA_STRU m_HMIRunDataStru;
    HMI_DEV_DATA_STRU m_HMIDevDataStru;
    std::vector<unsigned short> m_AGVErrorCodeVector;
    int m_NodeCnt = 0;
};

enum COM_TYPE_ENUM
{
    COMTYPE_TCP,
    COMTYPE_UDP,
    COMTYPE_RS232,
    COMTYPE_RS485
};

//启用/禁用外设
struct HardwareEnableStatusStru
{
    int m_WarningLightEnable = 0;       //是否使能指示灯，0：关闭，1：使能
    int m_BuzzerEnable = 0;             //是否使能蜂鸣器
    int m_SafeLaserEnable = 0;          //是否屏蔽避障激光
    int m_PhotoelectricityEnable = 0;   //是否屏蔽避障光电
    int m_SafeSwitchEnable = 0;         //是否强制使能安全继电器
    int m_Charge = 0;                   //是否充电
};

//print-information map
extern std::map<MANUAL_CTRL_ACTION, std::string> g_ManualActionMap;
extern std::map<MIO_TOP_STATUS, std::string> g_MIOTopStatusMap;
extern std::map<MIO_AUTO_STATUS, std::string> g_MIOAutoStatusMap;
extern std::map<COM_TYPE_ENUM, std::string> g_ComTypeMap;
extern std::map<DeviceList_ENUM, std::string> g_DeviceListMap;

} // namespace MainControl

#endif  //AGV_DATA_H

