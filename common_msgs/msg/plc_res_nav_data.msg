int32 capacity_remain    # PLC读取到的电池电量
float32 voltage          # PLC读取到的电池电压
float32 vel_linear       # PLC上传的车体中心线速度
int8 status_sensor_signal # 传感器到位信号-物料，0-未到位，1-到位
int8 status_stop         # 停车到位状态: 是否到达停车点，0-未知，1-未到位，2-到位
int8 status_charge       # 充电状态:	0-未知；1-充电中；2-充电结束
int32 id_stop_node       # 到达停车点的节点编号(包含导航和动作节点)