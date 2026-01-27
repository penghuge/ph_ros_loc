#include "common_msgs/action_data.h"
#include "common_msgs/nav_data.h"
#include "common_msgs/one_action_task.h"
#include "common_msgs/plc_res_action_data.h"
#include "common_msgs/plc_res_nav_data.h"
#include "plc_udp_client.h"
namespace mio
{
#define __MIO_MSG_HEADER1__ 0xFF
#define __MIO_MSG_HEADER2__ 0xFA

#define __MIO_PROTO_ACTUATOR_NUM__ 5  // 协议中执行机构数量

#define DELETE_THREAD(ptr)  \
    if ((ptr) != nullptr) { \
        (ptr)->join();      \
        delete (ptr);       \
        (ptr) = nullptr;    \
    }

struct HeaderData {
    uint8_t head1 = 0;
    uint8_t head2 = 0;
    uint8_t second_version = 0;
    uint8_t major_version = 0;
    uint32_t cnt = 0;
    uint8_t sender_ip_4 = 0;
    uint8_t sender_ip_3 = 0;
    uint8_t sender_ip_2 = 0;
    uint8_t sender_ip_1 = 0;
    uint16_t mid = 0;
    uint16_t msg_len = 0;
    uint64_t reserve = 0;
};

typedef struct {
    int32_t v_x = 0;
    int32_t v_y = 0;
    int32_t w = 0;
    int32_t linear_acceleration_x = 0;
    int32_t linear_acceleration_y = 0;
    int32_t linear_acceleration_z = 0;
    int32_t angular_velocity_x = 0;
    int32_t angular_velocity_y = 0;
    int32_t angular_velocity_z = 0;
    int32_t roll = 0;
    int32_t pitch = 0;
    int32_t yaw = 0;
    int16_t Quaterniond_w = 0;
    int16_t Quaterniond_x = 0;
    int16_t Quaterniond_y = 0;
    int16_t Quaterniond_z = 0;

    int32_t error_code_hardware = 0;     // 底盘硬件故障代码（原值传送）
    int32_t error_code_emergency = 0;    // 底盘紧急故障代码（原值传送），eg安全plc
    int32_t error_code_connect = 0;      // 底盘硬件通讯类故障代码（原值传送）
    int32_t error_code_application = 0;  // 底盘运行类型故障代码（原值传送）
    int32_t capacity_remain = 0;         // PLC读取到的电池电量
    int32_t voltage = 0;                 // PLC读取到的电池电压
    int32_t reserve_3 = 0;               //
    int32_t reserve_4 = 0;               //
    int32_t reserve_5 = 0;               //
    int32_t reserve_6 = 0;               //
    int32_t reserve_7 = 0;               //

    int32_t reserve_8 = 0;
    char reserve_9 = 0;
    char reserve_10 = 0;
    char status_run = 0;            // agv 运行状态
    char status_sensor_signal = 0;  // 载货状态，0-未载货，1-载货
    char status_stop = 0;           // 停车到位状态, 是否到达停车点，0-未知，1-未到位，2-到位
    char status_charge = 0;         // 充电状态	char	0-未知；1-充电中；2-充电结束
    char reserve_11 = 0;            //
    char reserve_12 = 0;            //
    int32_t id_stop_node = 0;       // 到达停车点的节点编号(包含导航和动作节点)
    int32_t reserve_20 = 0;         //
    char reserve_13 = 0;            //
    char reserve_14 = 0;            //
    char reserve_15 = 0;            //
    char reserve_16 = 0;            //

    int32_t reserve_17 = 0;  // 预留
    int32_t reserve_18 = 0;  // 预留
    int32_t reserve_19 = 0;  // 预留

    char index_feedback_num_encoder = 0;  // 室外叉车索引
    char length_encoder = 0;              // 包括索引号在内的所有内容占用长度，单位 byte
    char reserve_encoder_1 = 0;           // 预留
    char reserve_encoder_2 = 0;           // 预留
    int16_t angle_left_front = 0;         // 0.01 degree
    int16_t angle_right_front = 0;        // 0.01 degree
    int32_t encoder_left = 0;             // 左轮编码值
    int32_t encoder_right = 0;            // 右轮编码值
    int16_t speed_left = 0;               // 0.01 m/s
    int16_t speed_right = 0;              // 0.01 m/s
    int16_t linear_center = 0;            // 0.01 m/s 车体中心线速度
    int16_t angular_center = 0;           // 0.001 rad/s 车体中心角速度

    char index_feedback_num_gnss = 0;  // gnss 索引
    char length_gnss = 0;              // 包括索引号在内的所有内容占用长度，单位 byte
    char status_gnss = 0;              // gnss 状态
    char reserve_gnss_1 = 0;           // 预留
    int32_t longitude = 0;             // 经度
    int32_t latitude = 0;              // 纬度
    int32_t altitude = 0;              // 高度
    int32_t dir_gnss = 0;              // 方向

} PLC_NAV_MSG;

typedef struct {
    int32_t id_current_node = 0;          // 当前任务的编号
    int32_t type_current_node = 0;        // 当前任务类型，枚举详见附录5
    int32_t speed_current_node = 0;       // 当前任务速度（管控下发的速度）
    int32_t x_des_point = 0;              // 终点坐标 x
    int32_t y_des_point = 0;              // 终点坐标 y
    int32_t delta_pose_theta = 0;         //
    int32_t type_next_node = 0;           // 下一个任务类型，枚举详见附录5
    int32_t speed_next_node = 0;          // 下一个任务速度
    int32_t data_next_node = 0;           // 下一个任务数据（比如：转弯半径）
    int32_t v_x = 0;                      // 手动下发 x 轴方向线速度
    int32_t v_y = 0;                      // 手动下发 y 轴方向线速度
    int32_t w = 0;                        // 手动下发旋转角速度
    int32_t loc_x = 0;                    // map坐标系下pose x
    int32_t loc_y = 0;                    // map坐标系下pose y
    int32_t loc_theta = 0;                // map坐标系下pose theta
    int32_t heading_angle = 0;            // 用于车体特殊角度（45°等）运行
    int32_t delta = 0;                    // 车体中心到路线的投影距离
    int32_t delta_theta = 0;              // 车头方向与路径朝向的夹角
    int32_t p_delta = 0;                  // 运动方向头部偏差
    int32_t L_delta = 0;                  // 运动方尾部偏差
    int32_t remain_turn_theta = 0;        // 自旋纠偏的剩余角度
    int32_t radius_turn = 0;              // 转弯任务下发转弯半径
    int32_t length_turn = 0;              // 转弯任务的圆弧长度
    int32_t dis_to_stop_node = 0;         // 到下一个停车点的距离
    int32_t dis_to_type_switch_node = 0;  // 到下一个运动类型切换点的距离

    int16_t ctrl_mode = 0;       // 控制模式, 0x00-无效模式 0x01-手动控制 0x02-自动控制
    int16_t obs_area = 0;        // 避障区域
    int16_t battery_remain = 0;  // 剩余电量
    int16_t voltage = 0;         // 电压

    char buzzer = 0;       // 蜂鸣器状态枚举
    char light = 0;        // 灯状态枚举
    char ctrl_charge = 0;  // 充电控制: 0-未知 1-去充电 2-停止充电
    char type_park = 0;    // 停车方式
    char type_loc = 0;     // 定位类型
    char reserve_1 = 0;    // 预留
    char status_loc = 0;   // 定位状态
    char status_nav = 0;  // 导航激光避障状态 0-正常运行，1-缓速运行，2-障碍物停车，3-紧急停车

    int32_t list_error = 0;            // 故障列表
    int32_t dis_remain_curr_node = 0;  // 当前节点剩余距离，负值表示过了当前目标点，PLC需要停车
    int32_t reserve_5 = 0;

    char flag_action_ganged = 0;
    char reserve_2 = 0;  // 预留
    char reserve_3 = 0;  // 预留
    char reserve_4 = 0;  // 预留

    /****** 以下是扩展数据 ******/
    char index_feedback_num_camera = 1;     // 托盘检测相机 索引
    char length_detect_pallet_camera = 20;  // 包括索引号在内的所有内容占用长度，单位 byte
    char id_camera = 1;                     // 相机ID，预留给多个相同作用的相机
    char flag_detect_pallet = 0;            // 相机是否检测到托盘: 1 成功 2 失败
    char flag_realtime_detect = 1;          // 相机实时检测是否开启：0 关闭 1 开启
    char flag_pick_task_valid = 0;          // 取货任务是否有效， 0 表示无效， 1 表示有效
    char resever_camera_2 = 0;              // 预留
    char resever_camera_3 = 0;              // 预留
    int32_t x_detect = 0;                   // 托盘在相机坐标系下的 x
    int32_t y_detect = 0;                   // 托盘在相机坐标系下的 y
    int32_t theta_detect = 0;               // 托盘在相机坐标系下的 theta

    char index_preview_point = 2;    // 提供给PLC的预瞄点 索引
    char length_preview_point = 20;  // 包括索引号在内的所有内容占用长度，单位 byte
    char flag_data_valid = 1;  // 后续的预瞄点数据是否有效，无效的话，PLC可以使用原始协议的偏差数据纠偏
    char reserve_preview_data = 0;     // 预留
    int32_t x_preview_point = 0;       // 预瞄点 x, 单位 mm
    int32_t y_preview_point = 0;       // 预瞄点 y, 单位 mm
    int32_t theta_start_point = 0;     // 路线起点的 theta, 单位是 0.001度(36100代表不关心)
    int32_t theta_end_point = 0;       // 路线终点的 theta, 单位是 0.001度(36100代表不关心)
    int32_t x_base_project_point = 0;  // 车体中心在路径上的投影点x
    int32_t y_base_project_point = 0;  // 车体中心在路径上的投影点y
    int32_t x_head_project_point = 0;  // 车头在路径上的投影点x
    int32_t y_head_project_point = 0;  // 车头在路径上的投影点y
    int32_t x_tail_project_point = 0;  // 车尾在路径上的投影点x
    int32_t y_tail_project_point = 0;  // 车尾在路径上的投影点y

} IPC_NAV_MSG;

typedef struct {
    int32_t action_cmd = 0;
    int32_t action_value = 0;
    int32_t action_id = 0;
} ACTION_ATTRIBUTE;

typedef struct {
    int32_t type_execute_action = 0;  // 0 代表执行机构从上往下依次执行, 1 代表全部同时执行
    ACTION_ATTRIBUTE attributes[__MIO_PROTO_ACTUATOR_NUM__];
} IPC_ACTION_MSG;

typedef struct {
    int32_t status_action = 0;      // 执行机构状态
    int32_t error_code_action = 0;  // 执行机构故障码
    int32_t value_action = 0;       // 执行机构动作值
    int32_t id_action = 0;          // 执行机构任务序号
} ACTUATOR_STATUS_WORK;

typedef struct {
    ACTUATOR_STATUS_WORK status_work[__MIO_PROTO_ACTUATOR_NUM__];
} PLC_ACTION_MSG;

class PlcInteraction
{
   public:
    PlcInteraction();
    ~PlcInteraction();

    void RecvNavThread();
    void RecvActionThread();
    bool ParsePacket(const std::vector<unsigned char>& queue, HeaderData& header_data, unsigned char* buffer,
                     size_t& packet_length);
    bool HandleRecvNavMsg(const unsigned char* buffer, const int len);
    bool HandleRecvActionMsg(const unsigned char* buffer, const int len);
    bool SearchHeader(std::vector<unsigned char>& data);
    void NavDataProcess(const common_msgs::nav_data& data_nav);
    void ActionDataProcess(const common_msgs::action_data& data_action);
    void ShowRecvLog(const std::string& name_call);
    void PublishImuData(const PLC_NAV_MSG& nav_data_plc);
    void PublishEncoderData(const PLC_NAV_MSG& nav_data_plc);
    void PublishOdomDataAndTf(const PLC_NAV_MSG& nav_data_plc);

   private:
    ros::Publisher pub_encoder_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_res_data_;
    ros::Publisher pub_res_action_data_;
    ros::Subscriber sub_nav_;
    ros::Subscriber sub_action_;

    bool flag_plc_send_encoder_fix_ = false;
    std::string ip_address_plc_;
    std::string topic_encoder_;
    std::string topic_vel_;
    std::string topic_imu_;
    std::string topic_odom_;

    int size_encoder_data_ = 24;
    int size_fix_data_ = 20;
    int port_socket_server_nav_;
    int port_socket_client_nav_;
    int port_socket_server_action_;
    int port_socket_client_action_;

    std::shared_ptr<UdpClient> udp_client_nav_ptr_;
    std::shared_ptr<UdpClient> udp_client_action_ptr_;

    std::thread* thread_recv_nav_ = nullptr;
    std::atomic<bool> m_stop_recv_nav_{true};
    std::thread* thread_recv_action_ = nullptr;
    std::atomic<bool> m_stop_recv_action_{true};

    ConnectStateEnum nav_connect_status_ = CONNSTATE_WAIT_FOR_CONNECT;
    ConnectStateEnum action_connect_status_ = CONNSTATE_WAIT_FOR_CONNECT;
    HeaderData header_data_;               // 数据帧头部数据
    PLC_NAV_MSG nav_data_plc_;             // plc 发出来的 nav 数据
    IPC_NAV_MSG nav_data_ipc_;             // ipc 需要下发的 nav 数据
    PLC_ACTION_MSG action_data_from_plc_;  // plc 发出来的 action 数据
    IPC_ACTION_MSG action_data_ipc_;       // ipc 需要下发的 action 数据
};
}  // namespace mio