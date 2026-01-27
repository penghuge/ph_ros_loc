#include "rcs_interaction.h"

namespace rcs {

RcsInteraction::RcsInteraction()
{
    nh_.param<std::string>("/ip_rcs", params_rcs_.rcs_ip, std::string("192.168.2.210"));
    nh_.param<std::string>("/topic_task_rcs", topic_task_rcs_, "task_rcs");
    nh_.param<int>("/port",           params_rcs_.rcs_port, 502);
    nh_.param<int>("/start_bits",     params_rcs_.rcs_start_bits, 0);
    nh_.param<int>("/nb_bits",        params_rcs_.rcs_nb_bits, 100);
    nh_.param<int>("/start_input_bits", params_rcs_.rcs_start_input_bits, 0);
    nh_.param<int>("/nb_input_bits",    params_rcs_.rcs_nb_input_bits, 100);
    nh_.param<int>("/start_registers",  params_rcs_.rcs_start_registers, 0);
    nh_.param<int>("/nb_registers",     params_rcs_.rcs_nb_registers, 2500);
    nh_.param<int>("/start_input_registers", params_rcs_.rcs_start_input_registers, 0);
    nh_.param<int>("/nb_input_registers",    params_rcs_.rcs_nb_input_registers, 100);
    nh_.param<int>("/recv_timeout",          params_rcs_.rcs_recv_timeout, 3);
    nh_.param<int>("/disconnect_count_max",  params_rcs_.rcs_disconnect_count_max, 6);

    pub_rcs_task_ = nh_.advertise<common_msgs::rcs_cmd_data>(topic_task_rcs_, 10);
    sub_ipc_res_data_ = nh_.subscribe("ipc_res_data", 10, &RcsInteraction::IpcResDataProcess, this);

    StartListening();
}

bool RcsInteraction::InitSocket()
{
    ROS_INFO(" ip: %s, prot: %d", params_rcs_.rcs_ip.c_str(), params_rcs_.rcs_port);
    m_ctx_ = modbus_new_tcp(params_rcs_.rcs_ip.c_str(), params_rcs_.rcs_port);
    if (nullptr == m_ctx_) {
        ROS_ERROR("Failed to create Modbus context: %s", modbus_strerror(errno));
        return false;
    }
    m_header_length_ = modbus_get_header_length(m_ctx_);
    m_mb_mapping_ = modbus_mapping_new_start_address(
                    params_rcs_.rcs_start_bits,       params_rcs_.rcs_nb_bits,
                    params_rcs_.rcs_start_input_bits, params_rcs_.rcs_nb_input_bits,
                    params_rcs_.rcs_start_registers,  params_rcs_.rcs_nb_registers,
                    params_rcs_.rcs_start_input_registers, params_rcs_.rcs_nb_input_registers);
    if (NULL == m_mb_mapping_) {
        ROS_ERROR(" m_mb_mapping_ is null ");
        return false;
    }
    m_fd_ = modbus_tcp_listen(m_ctx_, 1);
    if(m_fd_ < 0) {
        ROS_ERROR(" modbus_tcp_listen less than 0 ");
        return false;
    }
    int ret = modbus_tcp_accept(m_ctx_, &m_fd_);
    if(ret < 0) {
        ROS_ERROR(" modbus_tcp_accept less than 0 ");
        return false;
    }
    // 设置超时
    modbus_set_indication_timeout(m_ctx_, params_rcs_.rcs_recv_timeout, params_rcs_.rcs_recv_timeout);
    ROS_INFO("Init modbus tcp server success! fd: %d, ret: %d", m_fd_, ret);
    return true;
}

int RcsInteraction::UninitSocket()
{
    flag_stop_thread_ = true;
    DELETE_THREAD(thread_rcs_server_);
    if (m_fd_ != -1) {
        close(m_fd_);
    }
    modbus_mapping_free(m_mb_mapping_);
    modbus_close(m_ctx_);
    modbus_free(m_ctx_);
    return 0;
}

void RcsInteraction::StartListening()
{
    flag_stop_thread_ = false;
    if(nullptr == thread_rcs_server_) {
        thread_rcs_server_ = new std::thread(&RcsInteraction::ServerThread, this);
    }
}

void RcsInteraction::ServerThread()
{
    int pid = syscall(SYS_gettid);
    ROS_INFO("rcs thread created: %d", pid);
    ROS_INFO("Attempt to init modbus tcp server: [ip: %s, port: %d]", params_rcs_.rcs_ip.c_str(), params_rcs_.rcs_port);
    status_connect_ = CONNSTATE_WAIT_FOR_CONNECT;
    int rc = 0;
    while (!flag_stop_thread_) {  // init socket
        if(InitSocket()) {
            break;
        }
        modbus_free(m_ctx_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    status_connect_ = CONNSTATE_CONNECTED;
    ROS_INFO("Start RCS modbus tcp server thread");
    time_last_ = std::chrono::system_clock::now();
    time_now_ = std::chrono::system_clock::now();
    int disconn_cnt = 0;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH] = { 0 };
    ROS_INFO(" Modbus mapping info -> nb_bits: %d, nb_input_bits: %d, nb_input_registers: %d, nb_registers: %d,"
             " start_bits: %d, start_input_bits: %d, start_input_registers: %d, start_registers: %d",
             m_mb_mapping_->nb_bits, m_mb_mapping_->nb_input_bits, m_mb_mapping_->nb_input_registers, m_mb_mapping_->nb_registers,
             m_mb_mapping_->start_bits, m_mb_mapping_->start_input_bits, m_mb_mapping_->start_input_registers, m_mb_mapping_->start_registers);

    while(!flag_stop_thread_)
    {
        if(m_fd_ < 0) { // reconnect
            ROS_INFO("Waiting for TCS modbus tcp client to reconnect...");
            status_connect_ = CONNSTATE_DISCONNECT;
            while (!flag_stop_thread_) {
                m_fd_ = modbus_tcp_listen(m_ctx_, 1);
                if(m_fd_ >= 0) {
                    break;
                }
                ROS_ERROR("Reconnect mcgs modbus tcp server failed: [ error: %s ]", modbus_strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            modbus_tcp_accept(m_ctx_, &m_fd_);
            status_connect_ = CONNSTATE_CONNECTED;
            disconn_cnt = 0;
            ROS_INFO("RCS Modbus tcp client reconnect successfully, fd: %d", m_fd_);
        }

        rc = modbus_receive(m_ctx_, query);
        // 打印接收到的指令
        // ROS_INFO("RCS Modbus header len: %d, recv query len: %d", m_header_length_, rc);
        // for(int i = 0; i < rc; ++i) {
        //     ROS_INFO("%x #n", query[i]);
        // }
        // ROS_INFO("#h");

        time_now_ =  std::chrono::system_clock::now();
        elapsed_seconds_ = time_now_ - time_last_;
        double cost_time = elapsed_seconds_.count();
        if(cost_time > params_rcs_.rcs_recv_timeout) {
            ROS_ERROR("RCS modbus recv data cost_time: [%f s]", cost_time);
        }
        time_last_ = time_now_;
        if (rc == -1) { // 判断连接故障
            if(errno != EMBBADCRC && ++disconn_cnt >= params_rcs_.rcs_disconnect_count_max) {
                /* Connection closed by the client or error */
                ROS_ERROR("RCS Modbus tcp server connection error: %s(%d)", modbus_strerror(errno), errno);
                close(m_fd_);
                m_fd_ = -1;
            }
            continue;
        }
        disconn_cnt = 0;

        /*ModbusTCP数据帧：MBAP+PDU
          报文头MBAP
            事务处理标识	协议标识	长度	单元标识符
             2字节	        2字节	  2字节	    1字节
          帧结构PDU
            功能码+数据组成。功能码为1字节，数据长度不定
              功能码：
                0x01-读线圈；0x05-写单个线圈；0x0F-写多个线圈；0x02-读离散量输入；0x04-读输入寄存器；
                0x03-读保持寄存器；0x06-写单个保持寄存器；0x10-写多个保持寄存器
        */
        switch (query[7]) {
            case MODBUS_FC_READ_INPUT_REGISTERS: {
                // ROS_INFO(" enter send data to rcs ...... ");
                // 刷新向管控输出的输入寄存器，即input_registers
                UpdateInputRegisters();
                // 刷新modbus本地缓存，并回复poll
                modbus_reply(m_ctx_, query, rc, m_mb_mapping_);
                break;
            }
            case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
                // ROS_INFO(" enter recv data from rcs ...... ");
                // 刷新modbus本地缓存，并回复poll
                modbus_reply(m_ctx_, query, rc, m_mb_mapping_);
                // 解析管控更新的保持寄存器, 数据存储
                ReadHoldingRegisters();
                break;
            }
            default: {
                break;
            }
        }
    }
    ROS_INFO("Shutdown modbus tcp server thread");
}

void RcsInteraction::FillIpcErrorCode(common_header::Ipc2RcsDataStru& data_ipc_to_rcs)
{
    std::vector<unsigned char> error_list = common_header::ErrorManager::GetInstance().GetErrorList();
    size_t size_one_err_bit = 8 * sizeof(data_ipc_to_rcs.error_1); // 故障码, 只上传严重故障
    const size_t err_size = 5;
    uint16_t err_buf[err_size] = {0};  // 故障码每一位各对应一个故障；无故障时，各位均为0
    int err = 0;
    for (size_t i = 0; i < error_list.size(); ++i) {
        err = static_cast<int>(error_list[i]) - 1;
        SET_BIT(err_buf[static_cast<int>(err / size_one_err_bit)], err % size_one_err_bit);
    }
    memcpy((char *)(&data_ipc_to_rcs.error_1), (char *)err_buf, err_size * sizeof(data_ipc_to_rcs.error_1));
}

void RcsInteraction::IpcResDataProcess(const common_msgs::ipc_res_data& data_res)
{
    lock_.lock();
    data_ipc_to_rcs_.tr_cmd      = data_res.tr_cmd;
    data_ipc_to_rcs_.tr_id_index = data_res.tr_id_index;
    data_ipc_to_rcs_.tr_id_hour  = data_res.tr_id_hour;
    data_ipc_to_rcs_.tr_id_day   = data_res.tr_id_day;
    data_ipc_to_rcs_.tr_id_month = data_res.tr_id_month;
    data_ipc_to_rcs_.tr_year     = data_res.tr_year;
    data_ipc_to_rcs_.tr_sender   = data_res.tr_sender;
    data_ipc_to_rcs_.tr_type     = data_res.tr_type;
    data_ipc_to_rcs_.tr_step_size    = data_res.tr_step_size;
    data_ipc_to_rcs_.tr_start_delay  = data_res.tr_start_delay;
    data_ipc_to_rcs_.tr_check_state  = data_res.tr_check_state;
    data_ipc_to_rcs_.tr_check_error  = data_res.tr_check_error;
    data_ipc_to_rcs_.tr_total_odom   = data_res.tr_total_odom;
    data_ipc_to_rcs_.tr_execute_time = data_res.tr_execute_time;
    data_ipc_to_rcs_.tr_complete_percent = data_res.tr_complete_percent;
    data_ipc_to_rcs_.tr_node_index       = data_res.tr_node_index;
    data_ipc_to_rcs_.tr_pause_info       = data_res.tr_pause_info;
    data_ipc_to_rcs_.node_compelete_percent = data_res.node_compelete_percent;
    data_ipc_to_rcs_.node_type        = data_res.node_type;
    data_ipc_to_rcs_.node_next_type   = data_res.node_next_type;
    data_ipc_to_rcs_.node_stop_state  = data_res.node_stop_state;
    data_ipc_to_rcs_.node_is_end_stop = data_res.node_is_end_stop;

    data_ipc_to_rcs_.node_info_1 = data_res.node_info_1;
    data_ipc_to_rcs_.node_info_2 = data_res.node_info_2;
    data_ipc_to_rcs_.node_info_3 = data_res.node_info_3;
    data_ipc_to_rcs_.node_info_4 = data_res.node_info_4;
    data_ipc_to_rcs_.node_info_5 = data_res.node_info_5;
    data_ipc_to_rcs_.node_info_6 = data_res.node_info_6;
    data_ipc_to_rcs_.node_info_7 = data_res.node_info_7;
    data_ipc_to_rcs_.node_info_8 = data_res.node_info_8;
    data_ipc_to_rcs_.node_info_9 = data_res.node_info_9;
    data_ipc_to_rcs_.node_info_10 = data_res.node_info_10;
    data_ipc_to_rcs_.node_info_11 = data_res.node_info_11;
    data_ipc_to_rcs_.node_info_12 = data_res.node_info_12;
    data_ipc_to_rcs_.node_info_13 = data_res.node_info_13;
    data_ipc_to_rcs_.node_info_14 = data_res.node_info_14;
    data_ipc_to_rcs_.node_info_15 = data_res.node_info_15;
    data_ipc_to_rcs_.node_info_16 = data_res.node_info_16;
    data_ipc_to_rcs_.node_info_17 = data_res.node_info_17;
    data_ipc_to_rcs_.node_info_18 = data_res.node_info_18;
    data_ipc_to_rcs_.node_info_19 = data_res.node_info_19;
    data_ipc_to_rcs_.node_info_20 = data_res.node_info_20;

    data_ipc_to_rcs_.poweron_time = data_res.poweron_time;
    data_ipc_to_rcs_.agv_pose_x   = data_res.agv_pose_x;
    data_ipc_to_rcs_.agv_pose_y   = data_res.agv_pose_y;
    data_ipc_to_rcs_.agv_pose_theta = data_res.agv_pose_theta;
    data_ipc_to_rcs_.lift_height  = data_res.lift_height;
    data_ipc_to_rcs_.lift_speed   = data_res.lift_speed;
    data_ipc_to_rcs_.agv_vel      = data_res.agv_vel;
    data_ipc_to_rcs_.is_parked    = data_res.is_parked;
    data_ipc_to_rcs_.steering_angle = data_res.steering_angle;
    data_ipc_to_rcs_.battery_soc    = data_res.battery_soc;
    data_ipc_to_rcs_.com_quantity   = data_res.com_quantity;
    data_ipc_to_rcs_.nav_quantity   = data_res.nav_quantity;
    data_ipc_to_rcs_.task_status = data_res.task_status;
    data_ipc_to_rcs_.mode_status = data_res.mode_status;
    data_ipc_to_rcs_.auto_status = data_res.auto_status;
    data_ipc_to_rcs_.version_hardware = data_res.version_hardware;
    data_ipc_to_rcs_.version_software = data_res.version_software;
    // data_ipc_to_rcs_.error_1 = data_res.error_1;
    // data_ipc_to_rcs_.error_2 = data_res.error_2;
    // data_ipc_to_rcs_.error_3 = data_res.error_3;
    // data_ipc_to_rcs_.error_4 = data_res.error_4;
    // data_ipc_to_rcs_.error_5 = data_res.error_5;

    FillIpcErrorCode(data_ipc_to_rcs_);

    data_ipc_to_rcs_.warning = data_res.warning;
    data_ipc_to_rcs_.cargo_loaded = data_res.cargo_loaded;
    lock_.unlock();
}

void RcsInteraction::UpdateInputRegisters()
{
    lock_.lock();
    memcpy(m_mb_mapping_->tab_input_registers, (uint16_t *)(&data_ipc_to_rcs_), sizeof(data_ipc_to_rcs_));  // 将数据更新给输入寄存器
    lock_.unlock();
}

void RcsInteraction::FillAndPubRcsTask(const uint16_t *const reg, const common_header::TaskCmdEnum& task_cmd)
{
    common_msgs::rcs_cmd_data data_rcs_cmd;
    size_t length_info = static_cast<size_t>(sizeof(common_header::Rcs2IpcTaskInfoStru));
    memcpy((uint16_t *)(&rcs_task_info_), reg, length_info);

    data_rcs_cmd.cmd         = task_cmd;
    data_rcs_cmd.id_index    = rcs_task_info_.tr_id_index;
    data_rcs_cmd.id_hour     = rcs_task_info_.tr_id_hour;
    data_rcs_cmd.id_day      = rcs_task_info_.tr_id_day;
    data_rcs_cmd.id_month    = rcs_task_info_.tr_id_month;
    data_rcs_cmd.year        = rcs_task_info_.tr_year;
    data_rcs_cmd.sender      = rcs_task_info_.tr_sender;
    data_rcs_cmd.type        = rcs_task_info_.tr_type;
    data_rcs_cmd.step_size   = rcs_task_info_.tr_step_size;
    data_rcs_cmd.start_delay = rcs_task_info_.tr_start_delay;

    int16_t size_step_task = rcs_task_info_.tr_step_size;
    size_t index_step_task = static_cast<size_t>(length_info * 0.5);
    if(common_header::TASK_CMD_RECV == task_cmd) {
        for (size_t i = 0; i < count_one_step_task_ * size_step_task; ++i) {
            data_rcs_cmd.data_step_tasks.push_back(reg[index_step_task + i]);
        }
    } else {
        data_rcs_cmd.step_size = 0; // RCS目前下发暂停，停止等都只更新前面两个字节，后面数据都不更新，会保持之前的，因此这里代替RCS清0，否则后续逻辑不匹配
    }
    pub_rcs_task_.publish(data_rcs_cmd);
}

// 将枚举值转换为字符串
std::string RcsInteraction::RcsTaskCmdToString(const RcsTaskCmdEnum cmd)
{
    switch (cmd) {
        case TASKCMD_NONE:             return "TASKCMD_NONE";
        case TASKCMD_PAUSE:            return "TASKCMD_PAUSE";
        case TASKCMD_EXECUTE:          return "TASKCMD_EXECUTE";
        case TASKCMD_OBORT:            return "TASKCMD_OBORT";
        case TASKCMD_RECV:             return "TASKCMD_RECV";
        case TASKCMD_REMOVE_STEP_PAUSE:return "TASKCMD_REMOVE_STEP_PAUSE";
        default:                       return "UNKNOWN_CMD";
    }
}

void RcsInteraction::ReadHoldingRegisters()
{
    const uint16_t *const reg = m_mb_mapping_->tab_registers;
    cmd_rcs_last_ = cmd_rcs_current_;
    cmd_rcs_current_ = (RcsTaskCmdEnum)reg[m_mb_mapping_->start_registers];
    ROS_INFO("tcs_step_0: cmd_from_tcs: cmd changed: [%s >> %s]", RcsTaskCmdToString(cmd_rcs_last_).c_str(), RcsTaskCmdToString(cmd_rcs_current_).c_str());

    common_header::TaskCmdEnum task_cmd = common_header::TASK_CMD_NONE; // 转换 RECV，第一次接收到 TASKCMD_EXECUTE 表示传输完成
    switch (cmd_rcs_current_) {
        case TASKCMD_PAUSE: {
            ROS_INFO("tcs_step_1: recv pause task cmd ");
            task_cmd = common_header::TASK_CMD_PAUSE;
            FillAndPubRcsTask(reg, task_cmd);
            break;
        }
        case TASKCMD_EXECUTE: {
            ROS_INFO("tcs_step_2_0: recv execute task cmd ");
            //上次是接收任务，本次是执行任务，则解析寄存器中的路线
            if(TASKCMD_RECV == cmd_rcs_last_) {
                ROS_INFO("tcs_step_2_1: rcs cmd recv finished ...... ");
                task_cmd = common_header::TASK_CMD_RECV;
            } else { // 仅当作启动指令
                ROS_INFO("tcs_step_2_2: rcs need start task ");
                task_cmd = common_header::TASK_CMD_EXECUTE;
            }
            FillAndPubRcsTask(reg, task_cmd);
            break;
        }
        case TASKCMD_OBORT: {
            ROS_INFO("tcs_step_3: recv obort task cmd ");
            task_cmd = common_header::TASK_CMD_OBORT;
            FillAndPubRcsTask(reg, task_cmd);
            break;
        }
        case TASKCMD_RECV: {
            ROS_INFO("tcs_step_4: recv send task data cmd ");
            break;
        }
        case TASKCMD_REMOVE_STEP_PAUSE: {    // 暂时不处理分步启动逻辑，作用是用来启动分步任务暂停的
            ROS_INFO("tcs_step_5: recv remove step pause task cmd ");
            // task_cmd = common_header::TASK_CMD_REMOVE_STEP_PAUSE;
            // FillAndPubRcsTask(reg, task_cmd);
            break;
        }
        default: {
            ROS_ERROR("tcs_err_1: recv not define task cmd ");
            break;
        }
    }
}

RcsInteraction::~RcsInteraction()
{
    UninitSocket();
}

}