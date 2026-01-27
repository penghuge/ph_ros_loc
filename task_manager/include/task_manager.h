#ifndef _TASK_MANAGER_H_
#define _TASK_MANAGER_H_

#include <cmath>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include <angles/angles.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include "common_msgs/action_data.h"
#include "common_msgs/common_haeder.h"
#include "common_msgs/ipc_res_data.h"
#include "common_msgs/nav_data.h"
#include "common_msgs/one_action_task.h"
#include "common_msgs/plc_res_action_data.h"
#include "common_msgs/plc_res_nav_data.h"
#include "common_msgs/rcs_cmd_data.h"
#include "nav_core/base_local_planner.h"
#include "pallet_identify_interface.h"
#include "pallet_identify_m4.h"
#include "task_manager/task_manager_paramsConfig.h"

namespace task
{
#define GET_INT32_FROM_INT16(tab_int16, index) ((tab_int16[(index) + 1] << 16) + tab_int16[(index)])

enum IpcStatusEnum
{
    STATUS_NONE,               // AGV处于手动或者其他状态(正在初始化)，RCS不能发数据
    STATUS_NO_TASK,            // AGV中没有存储任务，刚开机进入自动状态时显示, RCS可以发新任务
    STATUS_RECVING_TASK,       // 正在接收新任务， task_cmd = 4, 任务传输中, RCS可以发新任务
    STATUS_EXECUTING_TASK,     // IPC正在执行当前任务， RCS可以暂停任务或者终止任务
    STATUS_COMPLETED_TASK,     // 上一个任务已完成， RCS可以发新任务
    STATUS_PAUSE_TASK_BY_IPC,  // IPC主动暂停执行当前任务【可以是手动暂停、故障或者安全暂停】，RCS可以暂停或者终止任务
    STATUS_PAUSE_TASK_BY_RCS,  // IPC被RCS暂停任务，RCS可以恢复执行或者终止任务
    STATUS_ABORT_TASK          // 上一个任务已经终止，RCS可以发新任务
};

// 控制模式
enum CTRL_MODE_ENUM
{
    CTRL_MODE_UNKNOWN = 0x00,  // 无效模式
    CTRL_MODE_MANUAL = 0x01,   // 手动控制
    CTRL_MODE_AUTO = 0x02      // 自动控制
};

// 充电指令
enum CHARGE_CMD_ENUM
{
    CHARGE_CMD_UNKNOWN = 0x00,  // 未知
    CHARGE_CMD_GOING = 0x01,    // 去充电
    CHARGE_CMD_STOP = 0x02      // 停止充电
};

// 充电状态
enum STATUS_CHARGE_ENUM
{
    STATUS_CHARGE_UNKNOWN = 0x00,  // 未知
    STATUS_CHARGING = 0x01,        // 充电中
    STATUS_CHARGE_FINISHED = 0x02  // 充电结束
};

// 执行机构动作指令
enum ACTUATOR_CMD_ENUM
{
    ACTUATOR_CMD_STOP = 0x0,  //静止

    ACTUATOR_CMD_LEFT_GET = 0x11,   //辊筒左滚取货
    ACTUATOR_CMD_LEFT_PUT = 0x12,   //辊筒左滚放货
    ACTUATOR_CMD_RIGHT_GET = 0x13,  //辊筒右滚取货
    ACTUATOR_CMD_RIGHT_PUT = 0x14,  //辊筒右滚放货
    ACTUATOR_CMD_FRONT_GET = 0x15,  //辊筒前滚取货
    ACTUATOR_CMD_FRONT_PUT = 0x16,  //辊筒前滚放货
    ACTUATOR_CMD_BACK_GET = 0x17,   //辊筒后滚取货
    ACTUATOR_CMD_BACK_PUT = 0x18,   //辊筒后滚放货

    ACTUATOR_CMD_FORK_UP = 0x21,     //货叉上升
    ACTUATOR_CMD_FORK_DOWN = 0x22,   //货叉下降
    ACTUATOR_CMD_FORK_CLOSE = 0x23,  //货叉抱夹
    ACTUATOR_CMD_FORK_OPEN = 0x24,   //货叉松夹

    ACTUATOR_CMD_TRAY_UP = 0x31,    //顶升托盘上升
    ACTUATOR_CMD_TRAY_DOWN = 0x32,  //顶升托盘下降

    ACTUATOR_CMD_TRACTION_UP = 0x41,    //牵引棒上升
    ACTUATOR_CMD_TRACTION_DOWN = 0x42,  //牵引棒下降
};

// 执行机构状态
enum ACUATOR_STATUS_ENUM
{
    ACUATOR_IDLE = 0x00,
    ACUATOR_LIFT_DOWN_DONE = 0x02,  //下降到位
    ACUATOR_LIFT_UP_DONE = 0x01,    //举升到位
    ACUATOR_LIFTING = 0x04,         //举升/下降中
    ACUATOR_ROLLING = 0x10,         //辊筒左滚/右滚/前滚/后滚中
    ACUATOR_ROLL_GET_DONE = 0x11,   //辊筒有货静止
    ACUATOR_ROLL_PUT_DONE = 0x12,   //辊筒无货静止
    ACUATOR_CLAMPING = 0x20,
    ACUATOR_CLAMP_DONE = 0x21,
};

// 节点类型
enum RCSNodeTypeEnum
{
    RCS_NODE_NONE,
    RCS_NODE_LINE,
    RCS_NODE_CURVE,
    RCS_NODE_WORK,
    RCS_NODE_CHARGE,
    RCS_NODE_TRANSFER,
    RCS_NODE_AUTORUN_LINE,
    RCS_NODE_AUTORUN_CURVE,
    RCS_NODE_STANDBY_OPE,
    RCS_NODE_AUTORUN_BY_CAMERA,
    RCS_NODE_TASK_FINISH = 99,  // RCS增加用来判断特殊子任务[自调节、盲走]是否完成，因为有点吸附问题
    RCS_NODE_NUM
};

// 当前执行任务类型
enum CurrExecuteTaskTypeEnum
{
    IPC_IDLE,
    IPC_FOLLOW_PATH,
    IPC_AROUND_SPIN,
    IPC_PICK_CARGO,
    IPC_WORK,
    IPC_CHARGE,
    IPC_TASK_FINISH
};

// 偏差数据结构体
struct PathDeviation {
    double delta_x;           // 横向偏差（过B点为正）
    double delta_y;           // 纵向偏差（左侧为正）
    double delta_theta;       // 角度偏差（逆时针为正）
    double front_projection;  // 车头投影纵向偏差
    double rear_projection;   // 车尾投影纵向偏差
};

struct RCSNodeStru;

#pragma pack(push, 1)
struct RCSLineNodeStru {
    RCSLineNodeStru()
        : type(1),
          target_vel(0),
          heading(0),
          length(0),
          cargo_size(0),
          start_pt_x(0),
          start_pt_y(0),
          end_pt_x(0),
          end_pt_y(0),
          map_id(0),
          id_a(0),
          id_b(0)
    {
    }
    void Reset(const RCSNodeStru &node);
    int16_t type;
    int16_t target_vel;
    int16_t heading;
    int16_t length;
    int16_t cargo_size;
    int32_t start_pt_x;
    int32_t start_pt_y;
    int32_t end_pt_x;
    int32_t end_pt_y;
    int16_t depth;
    int16_t x_map_box_frame;  // 世界坐标系到车厢坐标系之间的转换
    int16_t y_map_box_frame;
    int16_t remain17;
    int16_t map_id;
    int16_t id_a;
    int16_t id_b;
};

struct RCSCurveNodeStru {
    RCSCurveNodeStru()
        : type(2),
          target_vel(0),
          heading(0),
          length(0),
          cargo_size(0),
          start_pt_x(0),
          start_pt_y(0),
          end_pt_x(0),
          end_pt_y(0),
          center_pt_x(0),
          center_pt_y(0),
          map_id(0),
          id_a(0),
          id_b(0)
    {
    }
    void Reset(const RCSNodeStru &node);
    int16_t type;
    int16_t target_vel;
    int16_t heading;
    int16_t length;
    int16_t cargo_size;
    int32_t start_pt_x;
    int32_t start_pt_y;
    int32_t end_pt_x;
    int32_t end_pt_y;
    int32_t center_pt_x;
    int32_t center_pt_y;
    int16_t map_id;
    int16_t id_a;
    int16_t id_b;
};

struct RCSWorkNodeStru {
    RCSWorkNodeStru()
        : type(3),
          static_work(0),
          target_vel(0),
          stop_before_done(0),
          done_wait_time(0),
          cargo_size(0),
          is_loading_cargo(0),
          heading(0),
          start_pt_x(0),
          start_pt_y(0),
          position2(0),
          position3(0),
          map_id(0),
          id_a(0),
          id_b(0)
    {
    }
    void Reset(const RCSNodeStru &node);
    int16_t type;
    int16_t static_work;
    int16_t target_vel;
    int16_t stop_before_done;
    int16_t done_wait_time;
    int16_t cargo_size;
    int16_t is_loading_cargo;  // 0: 空，1: 取货，2: 放货
    int16_t heading;
    int32_t start_pt_x;
    int32_t start_pt_y;
    int16_t position2;
    int16_t position3;
    int16_t remain15;
    int16_t remain16;
    int16_t remain17;
    int16_t map_id;
    int16_t id_a;
    int16_t id_b;
    int16_t remain20;
};

struct RCSChargeNodeStru {
    RCSChargeNodeStru()
        : type(4),
          charge_mode(0),
          charge_limit(0),
          cargo_size(0),
          heading(0),
          start_pt_x(0),
          start_pt_y(0),
          map_id(0),
          id_a(0)
    {
    }
    void Reset(const RCSNodeStru &node);
    int16_t type;
    int16_t charge_mode;
    int16_t charge_limit;
    int16_t cargo_size;
    int16_t heading;
    int32_t start_pt_x;
    int32_t start_pt_y;
    int16_t remain10;
    int16_t remain11;
    int16_t remain12;
    int16_t remain13;
    int16_t remain14;
    int16_t remain15;
    int16_t remain16;
    int16_t remain17;
    int16_t map_id;
    int16_t id_a;
    int16_t remain20;
};

struct RCSTransferNodeStru {
    RCSTransferNodeStru()
        : type(5),
          heading(0),
          start_pt_x(0),
          start_pt_y(0),
          is_where(0),
          operator_id(0),
          position(0),
          obj_speed(0),
          cmd(0),
          wait_time(0),
          cargo_size(0),
          map_id(0),
          id_a(0)
    {
    }
    void Reset(const RCSNodeStru &node);
    int16_t type;
    int16_t heading;
    int32_t start_pt_x;
    int32_t start_pt_y;
    int16_t is_where;
    int16_t operator_id;
    uint16_t position;
    uint16_t obj_speed;
    uint16_t cmd;
    int16_t wait_time;
    int16_t cargo_size;
    int16_t remain14;
    int16_t remain15;
    int16_t remain16;
    int16_t remain17;
    int16_t map_id;
    int16_t id_a;
    int16_t remain20;
};

struct RCSStandbyNodeStru {
    RCSStandbyNodeStru()
        : type(8),
          Ope_type(0),
          WaitTime(0),
          Sen_type(0),
          CodeID(0),
          cargo_size(0),
          tar_point_x(0),
          tar_point_y(0),
          tar_point_theta(0),
          adj_type(0),
          motion_mehtod(0),
          limit_x(0),
          limit_y(0),
          map_id(0),
          id_a(0),
          id_b(0)
    {
    }
    void Reset(const RCSNodeStru &node);
    int16_t type;
    int16_t Ope_type;  // target_vel;
    int16_t WaitTime;
    int16_t Sen_type;  // heading;
    int16_t CodeID;    // length;
    int16_t cargo_size;
    int32_t tar_point_x;
    int32_t tar_point_y;
    int32_t tar_point_theta;
    int16_t adj_type;
    int16_t motion_mehtod;
    int16_t limit_x;
    int16_t limit_y;
    int16_t map_id;
    int16_t id_a;
    int16_t id_b;
};

struct RCSFinishNodeStru {
    RCSFinishNodeStru()
        : type(99),
          remain_2(0),
          remain_3(0),
          remain_4(0),
          remain_5(0),
          remain_6(0),
          remain_7(0),
          remain_8(0),
          remain_9(0),
          remain_10(0),
          remain_11(0),
          remain_12(0),
          task_id_index(0),
          map_id(0),
          id_a(0),
          id_b(0)
    {
    }
    void Reset(const RCSNodeStru &node);
    int16_t type;
    int16_t remain_2;
    int16_t remain_3;
    int16_t remain_4;
    int16_t remain_5;
    int32_t remain_6;
    int32_t remain_7;
    int32_t remain_8;
    int32_t remain_9;
    int16_t remain_10;
    int16_t remain_11;
    int16_t remain_12;
    int16_t task_id_index;
    int16_t map_id;
    int16_t id_a;
    int16_t id_b;
};

struct RCSNodeStru {
    RCSNodeStru()
        : type(0),
          target_vel(0),
          heading(0),
          length(0),
          cargo_size(0),
          static_work(0),
          stop_before_done(0),
          done_wait_time(0),
          charge_mode(0),
          charge_limit(0),
          operator_id(0),
          position1(0),
          obj_speed(0),
          cmd(0),
          map_id(0),
          start_pt_x(0),
          start_pt_y(0),
          end_pt_x(0),
          end_pt_y(0),
          center_pt_x(0),
          center_pt_y(0),
          id_a(0),
          id_b(0),
          Ope_type(0),
          WaitTime(0),
          Sen_type(0),
          CodeID(0),
          tar_point_x(0),
          tar_point_y(0),
          tar_point_theta(0),
          adj_type(0),
          motion_mehtod(0),
          limit_x(0),
          limit_y(0)
    {
    }
    void Reset(const RCSLineNodeStru &line_node)
    {
        type = line_node.type;
        target_vel = line_node.target_vel;
        heading = line_node.heading;
        length = line_node.length;
        cargo_size = line_node.cargo_size;
        start_pt_x = line_node.start_pt_x;
        start_pt_y = line_node.start_pt_y;
        end_pt_x = line_node.end_pt_x;
        end_pt_y = line_node.end_pt_y;
        depth = line_node.depth;
        map_id = line_node.map_id;
        id_a = line_node.id_a;
        id_b = line_node.id_b;
        x_map_box_frame = line_node.x_map_box_frame;  // 世界坐标系到车厢坐标系之间的转换
        y_map_box_frame = line_node.y_map_box_frame;
        remain17 = line_node.remain17;
    }
    void Reset(const RCSCurveNodeStru &curve_node)
    {
        type = curve_node.type;
        target_vel = curve_node.target_vel;
        heading = curve_node.heading;
        length = curve_node.length;
        cargo_size = curve_node.cargo_size;
        start_pt_x = curve_node.start_pt_x;
        start_pt_y = curve_node.start_pt_y;
        end_pt_x = curve_node.end_pt_x;
        end_pt_y = curve_node.end_pt_y;
        center_pt_x = curve_node.center_pt_x;
        center_pt_y = curve_node.center_pt_y;
        map_id = curve_node.map_id;
        id_a = curve_node.id_a;
        id_b = curve_node.id_b;
    }
    void Reset(const RCSWorkNodeStru &work_node)
    {
        type = work_node.type;
        static_work = work_node.static_work;
        target_vel = work_node.target_vel;
        stop_before_done = work_node.stop_before_done;
        done_wait_time = work_node.done_wait_time;
        cargo_size = work_node.cargo_size;
        is_loading_cargo = work_node.is_loading_cargo;
        heading = work_node.heading;
        start_pt_x = work_node.start_pt_x;
        start_pt_y = work_node.start_pt_y;
        position2 = work_node.position2;
        position3 = work_node.position3;
        map_id = work_node.map_id;
        id_a = work_node.id_a;
        id_b = work_node.id_b;
    }
    void Reset(const RCSChargeNodeStru &charge_node)
    {
        type = charge_node.type;
        charge_mode = charge_node.charge_mode;
        charge_limit = charge_node.charge_limit;
        cargo_size = charge_node.cargo_size;
        heading = charge_node.heading;
        start_pt_x = charge_node.start_pt_x;
        start_pt_y = charge_node.start_pt_y;
        map_id = charge_node.map_id;
        id_a = charge_node.id_a;
    }
    void Reset(const RCSTransferNodeStru &transfer_node)
    {
        type = transfer_node.type;
        heading = transfer_node.heading;
        start_pt_x = transfer_node.start_pt_x;
        start_pt_y = transfer_node.start_pt_y;
        is_loading_cargo = transfer_node.is_where;
        operator_id = transfer_node.operator_id;
        position1 = transfer_node.position;
        obj_speed = transfer_node.obj_speed;
        cmd = transfer_node.cmd;
        done_wait_time = transfer_node.wait_time;
        cargo_size = transfer_node.cargo_size;
        map_id = transfer_node.map_id;
        id_a = transfer_node.id_a;
    }

    void Reset(const RCSStandbyNodeStru &standby_ope_node)
    {
        type = standby_ope_node.type;
        Ope_type = standby_ope_node.Ope_type;
        WaitTime = standby_ope_node.WaitTime;
        Sen_type = standby_ope_node.Sen_type;
        CodeID = standby_ope_node.CodeID;
        cargo_size = standby_ope_node.cargo_size;
        tar_point_x = standby_ope_node.tar_point_x;
        tar_point_y = standby_ope_node.tar_point_y;
        tar_point_theta = standby_ope_node.tar_point_theta;
        adj_type = standby_ope_node.adj_type;
        motion_mehtod = standby_ope_node.motion_mehtod;
        limit_x = standby_ope_node.limit_x;
        limit_y = standby_ope_node.limit_y;
        map_id = standby_ope_node.map_id;
        id_a = standby_ope_node.id_a;
        id_b = standby_ope_node.id_b;
    }
    void Reset(const RCSFinishNodeStru &finish_node)
    {
        type = finish_node.type;
        task_id_index = finish_node.task_id_index;
        map_id = finish_node.map_id;
        id_a = finish_node.id_a;
        id_b = finish_node.id_b;
    }

    int16_t type;
    int16_t target_vel;
    int16_t heading;
    int16_t length;
    int16_t cargo_size;
    int16_t static_work;
    int16_t stop_before_done;
    int16_t done_wait_time;
    int16_t is_loading_cargo;
    int16_t charge_mode;
    int16_t charge_limit;
    int16_t operator_id;
    int16_t x_map_box_frame;  // 世界坐标系到车厢坐标系之间的转换
    int16_t y_map_box_frame;
    int16_t remain17;
    int16_t depth;
    uint16_t position1;
    uint16_t position2;
    uint16_t position3;
    uint16_t obj_speed;
    uint16_t cmd;
    uint16_t map_id;  // map_change
    int32_t start_pt_x;
    int32_t start_pt_y;
    int32_t end_pt_x;
    int32_t end_pt_y;
    int32_t center_pt_x;
    int32_t center_pt_y;
    int16_t task_id_index;
    int16_t id_a;
    int16_t id_b;
    //  StandbyOpe
    int16_t Ope_type;
    int16_t WaitTime;
    int16_t Sen_type;
    int16_t CodeID;
    int32_t tar_point_x;
    int32_t tar_point_y;
    int32_t tar_point_theta;
    int16_t adj_type;
    int16_t motion_mehtod;
    int16_t limit_x;
    int16_t limit_y;
};
#pragma pack(pop)  // 恢复默认对齐

void RCSLineNodeStru::Reset(const RCSNodeStru &node)
{
    type = node.type;
    target_vel = node.target_vel;
    heading = node.heading;
    length = node.length;
    cargo_size = node.cargo_size;
    start_pt_x = node.start_pt_x;
    start_pt_y = node.start_pt_y;
    end_pt_x = node.end_pt_x;
    end_pt_y = node.end_pt_y;
    id_a = node.id_a;
    id_b = node.id_b;
}

void RCSCurveNodeStru::Reset(const RCSNodeStru &node)
{
    type = node.type;
    target_vel = node.target_vel;
    heading = node.heading;
    length = node.length;
    cargo_size = node.cargo_size;
    start_pt_x = node.start_pt_x;
    start_pt_y = node.start_pt_y;
    end_pt_x = node.end_pt_x;
    end_pt_y = node.end_pt_y;
    center_pt_x = node.center_pt_x;
    center_pt_y = node.center_pt_y;
    id_a = node.id_a;
    id_b = node.id_b;
}

void RCSWorkNodeStru::Reset(const RCSNodeStru &node)
{
    type = node.type;
    static_work = node.static_work;
    target_vel = node.target_vel;
    stop_before_done = node.stop_before_done;
    done_wait_time = node.done_wait_time;
    cargo_size = node.cargo_size;
    is_loading_cargo = node.is_loading_cargo;
    heading = node.heading;
    start_pt_x = node.start_pt_x;
    start_pt_y = node.start_pt_y;
    position2 = node.position2;
    position3 = node.position3;
    id_a = node.id_a;
}

void RCSChargeNodeStru::Reset(const RCSNodeStru &node)
{
    type = node.type;
    charge_mode = node.charge_mode;
    charge_limit = node.charge_limit;
    cargo_size = node.cargo_size;
    heading = node.heading;
    start_pt_x = node.start_pt_x;
    start_pt_y = node.start_pt_y;
    id_a = node.id_a;
}

void RCSTransferNodeStru::Reset(const RCSNodeStru &node)
{
    type = node.type;
    heading = node.heading;
    start_pt_x = node.start_pt_x;
    start_pt_y = node.start_pt_y;
    is_where = node.is_loading_cargo;
    operator_id = node.operator_id;
    position = node.position1;
    obj_speed = node.obj_speed;
    cmd = node.cmd;
    wait_time = node.done_wait_time;
    cargo_size = node.cargo_size;
    id_a = node.id_a;
}

void RCSStandbyNodeStru::Reset(const RCSNodeStru &node)
{
    type = node.type;
    Ope_type = node.Ope_type;
    WaitTime = node.WaitTime;
    Sen_type = node.Sen_type;
    CodeID = node.CodeID;
    cargo_size = node.cargo_size;
    tar_point_x = node.tar_point_x;
    tar_point_y = node.tar_point_y;
    tar_point_theta = node.tar_point_theta;
    adj_type = node.adj_type;
    motion_mehtod = node.motion_mehtod;
    limit_x = node.limit_x;
    limit_y = node.limit_y;
    map_id = node.map_id;
    id_a = node.id_a;
    id_b = node.id_b;
}

void RCSFinishNodeStru::Reset(const RCSNodeStru &node)
{
    type = node.type;
    task_id_index = node.task_id_index;
    map_id = node.map_id;
    id_a = node.id_a;
    id_b = node.id_b;
}

class TaskManager
{
   public:
    TaskManager();
    ~TaskManager();

    void RunCycle();
    void CalculateLinePathDeviation(const geometry_msgs::Point &A, const geometry_msgs::Point &B,
                                    const geometry_msgs::Pose &pose_current, PathDeviation &path_deviation);
    // 计算机器人实时位姿与1/4圆弧的偏差
    void CalculateCurvePathDeviation(const geometry_msgs::Point &center, const geometry_msgs::Point &point_start,
                                     const geometry_msgs::Point &point_end, const geometry_msgs::Pose &pose_current,
                                     const float radius, PathDeviation &path_deviation);

   private:
    template <typename T>
    T NormalizeAngle(T z)
    {
        const T kPi = T(M_PI);
        while (z > kPi)
            z -= 2. * kPi;
        while (z < -kPi)
            z += 2. * kPi;
        return z;
    }
    std::string IpcStatusToString(const IpcStatusEnum status);
    void ManualControlProcess(const geometry_msgs::Twist &vel_manual_control);
    void OdomProcess(const nav_msgs::Odometry &odom);
    void Current2dPoseProcess(const geometry_msgs::PoseStamped &pose_current);
    void Current3dPoseProcess(const nav_msgs::Odometry &pose_current);
    void TaskProcess(const common_msgs::rcs_cmd_data &data_task);
    void ProcessPlcResData(const common_msgs::plc_res_nav_data &data_res);
    void ProcessPlcResActionData(const common_msgs::plc_res_action_data &data_res_action);
    void CalCurveSignRadius(const int16_t heading, const int32_t start_x, const int32_t start_y, const int32_t end_x,
                            const int32_t end_y, const int32_t center_x, const int32_t center_y, double &radius_sign);
    bool ReadNodeFromRegister(const std::vector<uint16_t> &one_step_task, const int addr, RCSNodeStru &rcs_node);
    void PubIpcResData();
    bool CheckRcsPathTaskValidity(const std::vector<RCSNodeStru> &vec_step_task_stru);
    void GenerateLinePoints(const geometry_msgs::Point32 &point_start, const geometry_msgs::Point32 &point_end,
                            std::vector<geometry_msgs::Point32> &points_generate);
    void GenerateArcPoints(const geometry_msgs::Point32 &point_start, const geometry_msgs::Point32 &point_end,
                           const geometry_msgs::Point32 &center, double desired_arc_length,
                           std::vector<geometry_msgs::Point32> &points_generate);
    void GenerateGlobalDensePath(const std::vector<RCSNodeStru> &vec_step_task_stru,
                                 std::vector<std::vector<geometry_msgs::Point32>> &points_generate);
    geometry_msgs::Point32 FindClosestPoint(const geometry_msgs::Point32 &target_point,
                                            const std::vector<geometry_msgs::Point32> &points_step_task);
    double CalculateDistance(const geometry_msgs::Point32 &point1, const geometry_msgs::Point32 &point2);
    void UpdataLookAheadDis(const nav_msgs::Odometry &pose_current);
    void FillCommonNavData(common_msgs::nav_data &data_nav);
    void SetErrorCode(const common_header::IPC_ERROR_CODE error_code);
    void ClearErrorCode(const common_header::IPC_ERROR_CODE error_code);
    void PubActionData(const ACTUATOR_CMD_ENUM cmd_action, const int32_t value, const int32_t id_action);
    void GenerateAndPubNoTaskData();
    void GenerateAndPubFollowPathData();
    void GenerateAndPubAroundSpinData(const double angle_target);
    void GenerateAndPubWorkData(const int16_t static_work);
    void GenerateAndPubPickCargoData();
    void GenerateChargeData();
    void ExecuteTask();
    void GetLookaheadPoint(geometry_msgs::Point32 &point_lookahead,
                           const std::vector<geometry_msgs::Point32> &points_step_task);
    bool CheckNextTaskIsStopType();
    bool CheckTaskIsStopType(const int16_t type_task, const int16_t map_id);
    void GetCurrAndNextTaskInfo(const size_t &index_step_task, int16_t &id_curr_task, int16_t &type_curr_task,
                                float &speed_curr_task, float &radius_sign_curr_task, float &radius_length_curr_task,
                                int16_t &type_next_task, float &speed_next_task, float &radius_sign_next_task);
    void GetDisData(float &dis_to_stop_node, float &dis_to_type_switch_node, double delta_x = 0);
    double QuaternionToYaw(const geometry_msgs::Quaternion &q);
    void GetHeadTailPose(const geometry_msgs::Pose &pose_current, const double yaw, geometry_msgs::Point32 &head_pose,
                         geometry_msgs::Point32 &tail_pose);
    void InitTaskData();
    void FillStepTaskInfo(const RCSNodeStru &cur_node, uint16_t *data_step_task);
    void UpdataCurrentStepTaskIndex();
    void GetCurrentStepTaskRemainDis(double &dis_remain);
    bool FinishedCurrentStepTask();

    void ShowRcsTaskPoints(const std::vector<RCSNodeStru> &tasks_vec);
    void ShowGlobalDensePath(const std::vector<std::vector<geometry_msgs::Point32>> &points_vec);
    void ShowPoint(const geometry_msgs::Point32 &position, const float scale_marker, ros::Publisher &publisher);

    void DynamicReconfigureCallback(task_manager::task_manager_paramsConfig &config, uint32_t level);
    bool UpdateParamsSrv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

   private:
    ros::NodeHandle nh_;
    ros::Publisher pub_dense_path_;
    ros::Publisher pub_task_points_;
    ros::Publisher pub_ahead_point_;
    ros::Publisher pub_base_project_point_;
    ros::Publisher pub_head_project_point_;
    ros::Publisher pub_tail_project_point_;
    ros::Publisher pub_nav_data_;
    ros::Publisher pub_action_data_;
    ros::Publisher pub_ipc_res_data_;
    ros::Publisher pub_vel_;

    ros::Subscriber sub_manual_control_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_current_pose_;
    ros::Subscriber sub_rcs_task_;
    ros::Subscriber sub_plc_res_;
    ros::Subscriber sub_plc_action_res_;

    ros::ServiceServer update_params_srv_;

    size_t index_current_ahead_point_ = 0;
    bool flag_use_2d_loc_ = true;
    bool flag_show_project_point_ = false;
    bool flag_enable_mpc_ = false;
    bool flag_enable_pallet_identify_ = false;
    double min_dis_ahead_point_ = 0.6;
    double step_generate_path_ = 0.1;
    double distance_lookahead_ = 1.0;
    double scale_ahead_point_ = 0.7;
    float scale_project_point_ = 0.6;
    double distance_head_ = 3.0;
    double distance_tail_ = 0.45;
    double dis_derail_ = 0.2;
    float mm_to_m_ = 0.001;
    float m_to_mm_ = 1000.0;
    float dis_fix_ = 10.0;
    double threshold_arrive_des_ = 1.0;
    u_int8_t id_marker_ = 0;

    task_manager::task_manager_paramsConfig params_cfg_;
    double &landmark_thred_ = params_cfg_.landmark_thred;  //过点阈值
    double &derail_limit_x_ = params_cfg_.derail_limit_x, &derail_limit_y_ = params_cfg_.derail_limit_y,
           &derail_limit_theta_ = params_cfg_.derail_limit_theta;  // x轴脱轨阈值，y轴脱轨阈值，角度脱轨阈值
    double &static_work_thred_ = params_cfg_.static_work_thred;  //举升阈值

    geometry_msgs::Pose pose_current_;
    geometry_msgs::Twist twist_current_;
    double yaw_current_ = 0;
    geometry_msgs::Point start_point_curr_task_;
    geometry_msgs::Point center_point_curr_task_;
    geometry_msgs::Point end_point_curr_task_;

    common_header::TaskCmdEnum cmd_rcs_ = common_header::TASK_CMD_NONE;
    std::string topic_task_rcs_;
    IpcStatusEnum status_ipc_ = STATUS_NONE;
    common_header::Ipc2RcsDataStru data_res_ipc_;
    CurrExecuteTaskTypeEnum task_type_curr_ = IPC_IDLE;

    std::vector<RCSNodeStru> vec_step_task_stru_;
    std::vector<RCSNodeStru> vec_step_task_stru_global_;
    std::vector<std::vector<geometry_msgs::Point32>> vec_path_point_global_;

    bool flag_init_camera_ = false;  // 相机初始化是否成功
    std::string ip_camera_dtof_;
    std::unique_ptr<pallet_idenfity::PalletIdentifyBase> p_pallet_identify_;

    boost::shared_ptr<nav_core::BaseLocalPlanner> mpc_planner_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> mpc_loader_;

    dynamic_reconfigure::Server<task_manager::task_manager_paramsConfig> dyn_server_;

    // luoma or vmt data
    float x_manual_control_ = 0;  // 手动线速度
    float w_manual_control_ = 0;  // 手动角速度
    CTRL_MODE_ENUM ctrl_mode_ = CTRL_MODE_UNKNOWN;

    // ipc res task info
    size_t index_current_step_task_ = 0;
    int16_t compelete_percent_ = 0;

    // plc res nav info
    float g_vel_linear_ = 0;
    int16_t g_capacity_remain_ = 0;
    int8_t g_status_sensor_signal_ = 0;  // 传感器到位信号-物料，0-未到位，1-到位
    int8_t g_status_stop_ = 0;    // 停车到位状态: 是否到达停车点，0-未知，1-未到位，2-到位
    int8_t g_status_charge_ = 0;  // 充电状态:	0-未知；1-充电中；2-充电结束
    int32_t g_id_stop_node_ = 0;  // 到达停车点的节点编号(包含导航和动作节点)

    // plc res action info
    int32_t g_status_action_1 = 0;      // 执行机构1执行状态
    int32_t g_error_code_action_1 = 0;  // 执行机构1故障码
    int32_t g_value_action_1 = 0;       // 执行机构1实际动作值，比如叉车叉齿当前高度
    int32_t g_id_finish_action_1 = 0;   // 执行结构1完成动作对应的动作编号即动作序列号

    // nav data to plc
    int16_t id_curr_task_ = 0;
    int16_t type_curr_task_ = 0;
    float speed_curr_task_ = 0;
    float radius_sign_curr_task_ = 0;
    float radius_length_curr_task = 0;
    int16_t type_next_task_ = 0;
    float speed_next_task_ = 0;
    float radius_sign_next_task_ = 0;
    int32_t error_code_ = 0;
    PathDeviation path_deviation_;

    // action data to plc
    int32_t id_action_current_ = 1;  // 下发动作序列号，从1开始，如果从0开始，PLC默认回复0会导致刚好对上
};
}  // namespace task
#endif