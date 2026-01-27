/******************************************************************
 *
 * we define all the error codes as enum-type in this file
 *      error code and relative definition
 *
 * ***************************************************************/

#ifndef AGV_ERROR_CODE_H
#define AGV_ERROR_CODE_H

namespace MainControl {

#define __ERROR_CODE_MAX_SIZE__                         300

#define __ERROR_CODE_ERROR_BEGIN__                      1
#define __ERROR_CODE_ERROR_END__                        99
#define __ERROR_CODE_WARN_BEGIN__                       100
#define __ERROR_CODE_WARN_END__                         199
#define __ERROR_CODE_INFO_BEGIN__                       200
#define __ERROR_CODE_INFO_END__                         255

enum AGV_ERROR_CODE
{
    /*-- serious error, from __ERROR_CODE_ERROR_BEGIN__ to __ERROR_CODE_ERROR_END__ --*/
    ERROR_MIO_RECV_FAILED = __ERROR_CODE_ERROR_BEGIN__  ,//主控: MIO通讯接收异常
    ERROR_MIO_SEND_FAILED                               ,//主控: MIO通讯发送异常
    ERROR_SERVER_CONNECT_FAILED                         ,//主控: 远程通讯连接失败
    ERROR_SERVER_RECV_FAILED                            ,//主控: 远程通讯接收异常
    ERROR_SERVER_SEND_FAILED                            ,//主控: 远程通讯发送异常
    ERROR_MANUALCTRL_INIT_FAILED                        ,//主控: 手持器通讯初始化失败

    ERROR_NAVI_DERAILMENT                               ,//算法: 脱轨
    ERROR_NAVI_CFG_PARAM_INVALID                        ,//算法: 导航参数错误
    ERROR_NAVI_INIT_FAILED                              ,//算法: 导航初始化失败
    ERROR_NAVI_SCAN_MATCH_FAILED                        ,//算法: 导航初始定位失败
    ERROR_NAVI_MAIN_LOCALIZER_TIMEOUT                   ,//算法: 导航定位失败
    ERROR_NAVI_LOCAL_PLANNER_FAILED                     ,//算法: 导航局部规划失败
    ERROR_NAVI_LASER_INIT_FAILED                        ,//算法(激光): 激光初始化失败
    ERROR_NAVI_LASER_INIT_COM_FAILED                    ,//算法(激光): 激光通讯端口打开失败
    ERROR_NAVI_LASER_COM_DISCONNECT                     ,//算法(激光): 激光通讯中断
    ERROR_NAVI_LASER_READ_COM_FAILED                    ,//算法(激光): 激光数据读取失败
    ERROR_NAVI_LASER_DATA_INVALID                       ,//算法(激光): 激光数据无效
    ERROR_NAVI_LASER_OBS_IN_STOP_AREA                   ,//算法(激光): 导航激光避障触发
    ERROR_NAVI_LASER_DEVICE_ERROR                       ,//算法(激光): 激光设备内部故障
    ERROR_NAVI_IMU_INIT_FAILED                          ,//算法(惯导): 惯导初始化失败
    ERROR_NAVI_IMU_INIT_COM_FAILED                      ,//算法(惯导): 惯导通讯端口打开失败
    ERROR_NAVI_IMU_READ_COM_FAILED                      ,//算法(惯导): 惯导数据读取失败
    ERROR_NAVI_IMU_DATA_INVALID                         ,//算法(惯导): 惯导数据无效

    ERROR_PATH_PLAN_GENERATE_FAILED                     ,//路径规划：未搜索到可通行路径
    ERROR_PATH_PLAN_TRANSFORM_FORMAT_FAILED             ,//路径规划：路径本地格式化失败

    ERROR_MULTI_AGV_DISORDER                            ,//协同: 多AGV协同失调
    ERROR_MULTI_AGV_COM_ERROR                           ,//协同: 多AGV协同通讯故障
    ERROR_MULTI_AGV_DISCONNECT                          ,//协同: 多AGV协同通讯中断
    ERROR_MULTI_AGV_ERROR_STOP                          ,//协同: 多AGV协同故障停车
    ERROR_MULTI_AGV_FOLLOW_FAILED                       ,//协同: 多AGV协同跟随失败

    ERROR_MIO_REGISTER                                  ,//MIO: 注册失败
    ERROR_MIO_REGISTER_REPLY_ERR                        ,//MIO: 注册回复失败
    ERROR_MIO_RUN_CTRL_REPLY_ERR                        ,//MIO: 运行控制回复失败
    ERROR_MIO_RUN_INFO_REPLY_ERR                        ,//MIO: 运行信息回复失败
    ERROR_MIO_MOTOR_READ_REPLY_ERR                      ,//MIO: 驱动器参数读取回复失败
    ERROR_MIO_ACTUATOR_READ_REPLY_ERR                   ,//MIO: 执行机构读失败
    ERROR_MIO_ACTUATOR_WRITE_REPLY_ERR                  ,//MIO: 执行机构写失败
    ERROR_MIO_WARNING_WIRTE_REPLY_ERR                   ,//MIO: 警示信息写回复失败

    ERROR_MIO_COM_ERR_DRIVER_1                          ,//MIO: 驱动器1连接中断
    ERROR_MIO_COM_ERR_DRIVER_2                          ,//MIO: 驱动器2连接中断
    ERROR_MIO_COM_ERR_DRIVER_3                          ,//MIO: 驱动器3连接中断
    ERROR_MIO_COM_ERR_DRIVER_4                          ,//MIO: 驱动器4连接中断
    ERROR_MIO_COM_ERR_DRIVER_5                          ,//MIO: 驱动器5连接中断
    ERROR_MIO_COM_ERR_DRIVER_6                          ,//MIO: 驱动器6连接中断
    ERROR_MIO_COM_ERR_DRIVER_7                          ,//MIO: 驱动器7连接中断
    ERROR_MIO_COM_ERR_DRIVER_8                          ,//MIO: 驱动器8连接中断

    ERROR_MIO_INTER_SYS_ERR_DRIVER_1                    ,//MIO: 驱动器1内部故障
    ERROR_MIO_INTER_SYS_ERR_DRIVER_2                    ,//MIO: 驱动器2内部故障
    ERROR_MIO_INTER_SYS_ERR_DRIVER_3                    ,//MIO: 驱动器3内部故障
    ERROR_MIO_INTER_SYS_ERR_DRIVER_4                    ,//MIO: 驱动器4内部故障
    ERROR_MIO_INTER_SYS_ERR_DRIVER_5                    ,//MIO: 驱动器5内部故障
    ERROR_MIO_INTER_SYS_ERR_DRIVER_6                    ,//MIO: 驱动器6内部故障
    ERROR_MIO_INTER_SYS_ERR_DRIVER_7                    ,//MIO: 驱动器7内部故障
    ERROR_MIO_INTER_SYS_ERR_DRIVER_8                    ,//MIO: 驱动器8内部故障

    ERROR_MIO_AGV_DGN_SAFE_LASER                        ,//MIO: 安全激光触发
    ERROR_MIO_AGV_DGN_EMERG_BTN                         ,//MIO: 急停按钮触发
    ERROR_MIO_AGV_DGN_IMPACT                            ,//MIO: 安全触边触发
    ERROR_MIO_AGV_DGN_IMPACT_FAULT                      ,//MIO: 安全触边故障

    ERROR_MIO_SPE_COMPLETE_MSG_REPLY_ERR                ,//MIO: 完整报文写回复失败

    /*-- warning, from __ERROR_CODE_WARN_BEGIN__ to __ERROR_CODE_WARN_END__ --*/
    WARNING_NAVI_LASER_SCAN_UPDATE_TIMEOUT = __ERROR_CODE_WARN_BEGIN__,//算法: 激光扫描数据更新超时
    WARNING_NAVI_LASER_POSE_UPDATE_TIMEOUT              ,//算法: 激光位置数据更新超时
    WARNING_NAVI_LASER_POLLUTION                        ,//算法: 激光表面污染
    WARNING_NAVI_IMU_UPDATE_TIMEOUT                     ,//算法: 惯导数据更新超时
    WARNING_NAVI_ODOM_UPDATE_TIMEOUT                    ,//算法: 编码器数据更新超时
    WARNING_NAVI_MAIN_LOCALIZER_TIMEOUT                 ,//算法: 导航定位超时

    WARNING_PATH_PLAN_AGV_TRAJ_DIFF_ANGLE_TOO_LARGE     ,//路径规划：AGV与路线角度偏差超差
    WARNING_PATH_PLAN_TARGET_IS_CURRENT                 ,//路径规划：AGV已经位于目标点

    WARNING_MULTI_AGV_UPDATE_TIMEOUT                    ,//协同： 多AGV协同数据更新超时
    WARNING_MULTI_AGV_DATA_INVALID                      ,//协同： 多AGV协同数据无效

    WARNING_MIO_SAFE_LASER_WARNING                      ,//MIO: 安全激光警告

    WARNING_MIO_MANUAL_DEV_DISCONN                      ,//MIO: 手持器断连

    WARNING_MIO_PID_READ_NO_REPLY                       ,//MIO: PID读取回复失败
    WARNING_MIO_PID_WRITE_NO_REPLY                      ,//MIO: PID写入回复失败
    WARNING_MIO_LOG_READ_NO_REPLY                       ,//MIO: 日志读取回复失败

    WARNING_MIO_PID_READ_SEND_TIMEOUT                   ,//MIO: PID读取发送超时
    WARNING_MIO_PID_WRITE_SEND_TIMEOUT                  ,//MIO: PID写入发送超时
    WARNING_MIO_LOG_READ_SEND_TIMEOUT                   ,//MIO: 日志读取发送超时

    WARNING_MIO_RUN_CTRL_SEND_TIMEOUT                   ,//MIO: 运行控制发送超时
    WARNING_MIO_RUN_INFO_SEND_TIMEOUT                   ,//MIO: 运行信息发送超时
    WARNING_MIO_MOTOR_READ_SEND_TIMEOUT                 ,//MIO: 驱动器参数读取发送超时
    WARNING_MIO_ACTUATOR_READ_SEND_TIMEOUT              ,//MIO: 执行机构读发送超时
    WARNING_MIO_ACTUATOR_WRITE_SEND_TIMEOUT             ,//MIO: 执行机构写发送超时
    WARNING_MIO_WARNING_WIRTE_SEND_TIMEOUT              ,//MIO: 警示信息写发送超时
    WARNING_MIO_SPE_COMPLETE_MSG_SEND_TIMEOUT           ,//MIO: 完整报文发送超时

    WARNING_UNKNOWN                                     ,//未知故障
    /*-- information, from __ERROR_CODE_INFO_BEGIN__ to __ERROR_CODE_INFO_END__ --*/
};

//故障属性
struct ERROR_ATTRIBUTE_STRU
{
    bool m_ResetEnable = false; //是否可以自复位
};

//故障级别
enum AGV_ERROR_RATE_ENUM
{
    ERROR_RATE_NONE = 0,
    ERROR_RATE_SERIOUS,
    ERROR_RATE_WARNING,
    ERROR_RATE_INFORMATION,
    ERROR_RATE_NUM
};

} // namespace MainControl

#endif  //!AGV_ERROR_CODE_H

