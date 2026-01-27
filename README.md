# 2d_amr_indoor

# ros版本车控代码


## 一. 代码说明

agv_files: 目前配置文件，后续会删掉，配置文件放在各自对应的ros节点中。

base: 静态tf发布节点，在 urdf 文件中配置 激光、imu等传感器外参 即 传感器与 base_link 之间的位置关系。

carto_loc: 无反定位 即 cartographer 定位 ros 节点。

common_msgs 是公共消息包，需要增加的自定义消息，都需要放置在此包中，方便各个ros包去引用。

hipnuc_imu: imu 驱动 ros 包。

nav_core: 后续作为路径规划的包。

odom: 里程计数据发布以及  base_link 到 odom tf 转换发布的 ros 包。

pf_lidar_ros_driver-main: 激光雷达驱动的 ros 包。

plc_interaction: 跟PLC交互的 ros 包，负责接收以及给PLC发送指令，其他包不允许直接与PLC通信。

rcs_interaction: 跟RCS交互的 ros 包，负责接收以及给RCS发送应答数据，其他包不允许直接与RCS通信。

reflector_location: 有反定位 ros 包。

task_manager: 负责任务管理的 ros 包， 对接各ros 节点，生成plc需要的路线偏差、预瞄点、当前任务信息、下一个任务信息、剩余距离、切换距离等数据， 负责任务的管理。 



## 二. 编译指令

在 src 上一层目录，使用如下编译指令编译: 

catkin_make_isolated --install --use-ninja -j1

如果算力允许，可以使用 -j3 加速编译。