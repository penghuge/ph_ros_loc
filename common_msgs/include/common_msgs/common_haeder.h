#ifndef _COMMON_HEADER_
#define _COMMON_HEADER_

namespace common_header {

enum ConnectStateEnum
{
    CONNSTATE_DISCONNECT,
    CONNSTATE_WAIT_FOR_CONNECT,
    CONNSTATE_CONNECTED,
};

// 任务指令，RCS指令转换过后的，主要是转换 TASK_CMD_RECV， 是需要先传输再开始才认为是传输完成
enum TaskCmdEnum
{
    TASK_CMD_NONE,
    TASK_CMD_PAUSE,
    TASK_CMD_EXECUTE,
    TASK_CMD_OBORT,
    TASK_CMD_RECV,
    TASK_CMD_REMOVE_STEP_PAUSE,
    TASKCMD_NUM
};

enum IPC_ERROR_CODE
{
    // error_1
    ERROR_MIO_RECV_FAILED = 1,         //主控: MIO通讯接收异常
    ERROR_MIO_SEND_FAILED,             //主控: MIO通讯发送异常
    ERROR_SERVER_CONNECT_FAILED,       //主控: 远程通讯连接失败
    ERROR_SERVER_RECV_FAILED,          //主控: 远程通讯接收异常
    ERROR_SERVER_SEND_FAILED,          //主控: 远程通讯发送异常
    ERROR_MANUALCTRL_INIT_FAILED,      //主控: 手持器通讯初始化失败

    ERROR_NAVI_DERAILMENT,             //算法: 脱轨
    ERROR_NAVI_CFG_PARAM_INVALID,      //算法: 导航参数错误
    ERROR_NAVI_INIT_FAILED,            //算法: 导航初始化失败
    ERROR_NAVI_SCAN_MATCH_FAILED,      //算法: 导航初始定位失败
    ERROR_NAVI_MAIN_LOCALIZER_TIMEOUT, //算法: 导航定位失败
    ERROR_RCS_PATH_CHECK_FAILED,       //RCS: RCS下发鹅点不连续
    ERROR_NAVI_LASER_INIT_FAILED,      //算法(激光): 激光初始化失败
    ERROR_NAVI_LASER_INIT_COM_FAILED,  //算法(激光): 激光通讯端口打开失败
    ERROR_NAVI_LASER_COM_DISCONNECT,   //算法(激光): 激光通讯中断
    ERROR_NAVI_LASER_READ_COM_FAILED,  //算法(激光): 激光数据读取失败

    // error_2

};

// RCS下发的任务信息
typedef struct {
    int16_t tr_cmd = 0;                     // 任务指令
    int16_t tr_id_index = 0;
    int16_t tr_id_hour = 0;
    int16_t tr_id_day = 0;
    int16_t tr_id_month = 0;
    int16_t tr_year = 0;
    int16_t tr_sender = 0;                  // AGV IP
    int16_t tr_type = 0;                    // 任务类型，TCSTaskTypeEnum
    int16_t tr_step_size = 0;               // 任务总节点数，1~N
    int16_t tr_start_delay = 0;             // 任务启动延迟，单位：s
} Rcs2IpcTaskInfoStru;

typedef struct {
    int16_t tr_cmd = 0;                     // 任务指令，RCSTaskCmdEnum
    int16_t tr_id_index = 0;
    int16_t tr_id_hour = 0;
    int16_t tr_id_day = 0;
    int16_t tr_id_month = 0;
    int16_t tr_year = 0;
    int16_t tr_sender = 0;                  // AGV IP
    int16_t tr_type = 0;                    // 任务类型，TCSTaskTypeEnum
    int16_t tr_step_size = 0;               // 任务总节点数，1~N
    int16_t tr_start_delay = 0;             // 任务启动延迟，单位：s
    int16_t tr_check_state = 0;             // 任务检查状态，TCSTaskCheckStatusEnum
    int16_t tr_check_error = 0;             // 任务检查错误，TCSTaskCheckErrorEnum
    int16_t tr_total_odom = 0;              // 任务总里程，单位：cm
    int16_t tr_execute_time = 0;            // 任务已执行时间，单位：s
    int16_t tr_complete_percent = 0;        // 任务完成百分比，0~100，按量程计
    int16_t tr_node_index = 0;              // 当前节点下标，1~N
    int16_t tr_pause_info = 0;              // 任务暂停原因
    int16_t node_compelete_percent = 0;     // 当前节点完成百分比，0~100，按里程计
    int16_t node_type = 0;                  // 当前节点类型，TCSNodeTypeEnum
    int16_t node_next_type = 0;             // 下一节点类型，TCSNodeTypeEnum
    int16_t node_stop_state = 0;            // 当前节点是否暂停，0=无，1=暂停
    int16_t node_is_end_stop = 0;           // 0=非最后节点停车，1=最后节点停车

    int16_t node_info_1 = 0;                // 节点信息
    int16_t node_info_2 = 0;
    int16_t node_info_3 = 0;
    int16_t node_info_4 = 0;
    int16_t node_info_5 = 0;
    int16_t node_info_6 = 0;
    int16_t node_info_7 = 0;
    int16_t node_info_8 = 0;
    int16_t node_info_9 = 0;
    int16_t node_info_10 = 0;
    int16_t node_info_11 = 0;
    int16_t node_info_12 = 0;
    int16_t node_info_13 = 0;
    int16_t node_info_14 = 0;
    int16_t node_info_15 = 0;
    int16_t node_info_16 = 0;
    int16_t node_info_17 = 0;
    int16_t node_info_18 = 0;
    int16_t node_info_19 = 0;
    int16_t node_info_20 = 0;

    uint32_t poweron_time = 0;                   // 系统开机时间，单位：s
    int32_t agv_pose_x = 0;                      // AGV当前x坐标，单位：mm
    int32_t agv_pose_y = 0;                      // AGV当前y坐标，单位：mm
    int16_t agv_pose_theta = 0;                  // AGV当前角度，0~360，单位：0.1度
    int16_t lift_height = 0;                     // 举升高度，单位：mm
    int16_t lift_speed = 0;                      // 举升速度，单位：mm/s
    int16_t agv_vel = 0;                         // AGV当前速度，单位：mm/s
    int16_t is_parked = 0;                       // 是否驻车，0=否，1=是
    int16_t steering_angle = 0;                  // 当前舵角，单位：度
    int16_t battery_soc = 0;                     // 电量
    int16_t com_quantity = 0;                    // 通讯质量
    int16_t nav_quantity = 0;                    // 定位质量
    int16_t task_status = 0;                     // 任务状态，反馈IPC当前状态
    int16_t mode_status = 4;                     // 模式状态, 需要给4，否则RCS不下发任务
    int16_t auto_status = 0;                     // 自动状态
    uint16_t version_hardware = 0;               // 硬件版本
    uint16_t version_software = 0;               // 软件版本
    uint16_t error_1 = 0;                        // 错误码1
    uint16_t error_2 = 0;                        // 错误码2
    uint16_t error_3 = 0;                        // 错误码3
    uint16_t error_4 = 0;                        // 错误码4
    uint16_t error_5 = 0;                        // 错误码5
    uint16_t warning = 0;                        // 警告
    uint16_t cargo_loaded = 0;                   // 是否载货，1=有；0=没有
} Ipc2RcsDataStru;

class ErrorManager {
    public:
        static ErrorManager& GetInstance() {
            static ErrorManager instance;
            return instance;
        }

        ErrorManager(const ErrorManager&) = delete;
        ErrorManager& operator=(const ErrorManager&) = delete;

        // 获取错误列表
        std::vector<unsigned char> GetErrorList() {
            std::lock_guard<std::mutex> lock(mutex_);
            return error_list_;
        }

        // 添加一个错误到列表
        void AddError(unsigned char error) {
            std::lock_guard<std::mutex> lock(mutex_);
            error_list_.push_back(error);
        }

        // 清空错误列表
        void ClearErrorList() {
            std::lock_guard<std::mutex> lock(mutex_);
            error_list_.clear();
        }

    private:
        ErrorManager() = default;

        std::vector<unsigned char> error_list_;
        std::mutex mutex_;
};
}

#endif