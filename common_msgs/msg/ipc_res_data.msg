int16 tr_cmd                     # 任务指令，RCSTaskCmdEnum
int16 tr_id_index
int16 tr_id_hour
int16 tr_id_day
int16 tr_id_month
int16 tr_year
int16 tr_sender                  # AGV IP
int16 tr_type                    # 任务类型，TCSTaskTypeEnum
int16 tr_step_size               # 任务总节点数，1~N
int16 tr_start_delay             # 任务启动延迟，单位：s
int16 tr_check_state             # 任务检查状态，TCSTaskCheckStatusEnum
int16 tr_check_error             # 任务检查错误，TCSTaskCheckErrorEnum
int16 tr_total_odom              # 任务总里程，单位：cm
int16 tr_execute_time            # 任务已执行时间，单位：s
int16 tr_complete_percent        # 任务完成百分比，0~100，按量程计
int16 tr_node_index              # 当前节点下标，1~N
int16 tr_pause_info              # 任务暂停原因
int16 node_compelete_percent     # 当前节点完成百分比，0~100，按里程计
int16 node_type                  # 当前节点类型，TCSNodeTypeEnum
int16 node_next_type             # 下一节点类型，TCSNodeTypeEnum
int16 node_stop_state            # 当前节点是否暂停，0=无，1=暂停
int16 node_is_end_stop           # 0=非最后节点停车，1=最后节点停车

int16 node_info_1                # 节点信息
int16 node_info_2
int16 node_info_3
int16 node_info_4
int16 node_info_5
int16 node_info_6
int16 node_info_7
int16 node_info_8
int16 node_info_9
int16 node_info_10
int16 node_info_11
int16 node_info_12
int16 node_info_13
int16 node_info_14
int16 node_info_15
int16 node_info_16
int16 node_info_17
int16 node_info_18
int16 node_info_19
int16 node_info_20

uint32 poweron_time                      # 系统开机时间，单位：s
int32 agv_pose_x                         # AGV当前x坐标，单位：mm
int32 agv_pose_y                         # AGV当前y坐标，单位：mm
int16 agv_pose_theta                     # AGV当前角度，0~360，单位：0.1度
int16 lift_height                        # 举升高度，单位：mm
int16 lift_speed                         # 举升速度，单位：mm/s
int16 agv_vel                            # AGV当前速度，单位：mm/s
int16 is_parked                          # 是否驻车，0=否，1=是
int16 steering_angle                     # 当前舵角，单位：度
int16 battery_soc                        # 电量
int16 com_quantity                       # 通讯质量
int16 nav_quantity                       # 定位质量
int16 task_status                        # 任务状态，反馈IPC当前状态
int16 mode_status                        # 模式状态
int16 auto_status                        # 自动状态
uint16 version_hardware                  # 硬件版本
uint16 version_software                  # 软件版本
uint16 error_1                           # 错误码1
uint16 error_2                           # 错误码2
uint16 error_3                           # 错误码3
uint16 error_4                           # 错误码4
uint16 error_5                           # 错误码5
uint16 warning                           # 警告
uint16 cargo_loaded                      # 是否载货，1=有；0=没有