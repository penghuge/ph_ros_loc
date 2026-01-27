#!/bin/bash

# 记录日志到文件
exec > /tmp/ros_tmux.log 2>&1
echo "===== Script started at $(date) ====="

# 显式加载 ROS 环境
source /opt/ros/noetic/setup.bash
source ~/ws_2d_amr/install_isolated/setup.bash

SESSION_NAME="ros_session"

# 1️ 检查 `tmux` 是否已存在，防止重复启动
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "TMUX session $SESSION_NAME already exists. Exiting..."
    exit 0
fi

# 2️ 启动 tmux 会话
tmux new-session -d -s $SESSION_NAME

# 3️ 窗口 0：sensor（roscore + 传感器）
tmux rename-window -t $SESSION_NAME:0 "sensor"
tmux send-keys -t $SESSION_NAME:0 "roscore" C-m
sleep 3

tmux split-window -v -t $SESSION_NAME:0
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/ws_2d_amr/ && source install_isolated/setup.bash && roslaunch pf_driver r2000_no_rviz.launch" C-m

tmux split-window -h -t $SESSION_NAME:0
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/ws_2d_amr/ && source install_isolated/setup.bash && roslaunch plc_interaction plc_interaction_start.launch" C-m

tmux split-window -v -t $SESSION_NAME:0.2
tmux send-keys -t $SESSION_NAME:0.3 "cd ~/ws_2d_amr/ && source install_isolated/setup.bash && roslaunch yunshen_robot_navigation description_robot.launch" C-m

tmux select-layout -t $SESSION_NAME:0 tiled

# 4️ 窗口 1：loc（PLC 交互 + 定位）
tmux new-window -t $SESSION_NAME -n "loc"
tmux send-keys -t $SESSION_NAME:1 "cd ~/ws_2d_amr/ && source install_isolated/setup.bash && roslaunch cartographer_ros localization_2d.launch" C-m

tmux split-window -h -t $SESSION_NAME:1
tmux send-keys -t $SESSION_NAME:1.1 "cd ~/ws_2d_amr/ && source install_isolated/setup.bash && roslaunch show_interaction show_interaction_start.launch" C-m

# 5️ 窗口 2：task_manager（RCS 交互 + 任务管理）
tmux new-window -t $SESSION_NAME -n "task_manager"
tmux send-keys -t $SESSION_NAME:2 "cd ~/ws_2d_amr/ && source install_isolated/setup.bash && roslaunch rcs_interaction rcs_interaction_start.launch" C-m

tmux split-window -h -t $SESSION_NAME:2
tmux send-keys -t $SESSION_NAME:2.1 "cd ~/ws_2d_amr/ && source install_isolated/setup.bash && roslaunch task_manager task_manager_start.launch" C-m

# 6️ 选择 sensor 窗口
tmux select-window -t $SESSION_NAME:0

# 7️ 让 `tmux` 进程保持运行，防止 `systemd` 误判退出
tmux attach-session -t $SESSION_NAME
