int16 cmd                     # 任务指令，RCSTaskCmdEnum
int16 id_index
int16 id_hour
int16 id_day
int16 id_month
int16 year
int16 sender                  # AGV IP
int16 type                    # 任务类型，RCSTaskTypeEnum
int16 step_size               # 任务总节点数，1~N
int16 start_delay             # 任务启动延迟，单位：s
uint16[] data_step_tasks      # 分步任务数据