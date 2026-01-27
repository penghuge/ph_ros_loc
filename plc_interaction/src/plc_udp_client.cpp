#include "plc_udp_client.h"

namespace mio {

UdpClient::UdpClient(std::string& ip_address_plc, const int socket_server_port_nav, const int socket_client_port_nav)
{
    plc_ip_address_ = ip_address_plc;
    plc_socket_server_port_ = socket_server_port_nav;
    plc_socket_client_port_ = socket_client_port_nav;
}

UdpClient::~UdpClient()
{
    UninitSocket(true);
}

void UdpClient::UninitSocket(const bool display_log)
{
    if (display_log) {
        ROS_INFO("Attempting to stop listening...");
    }
    StopListening();
    if (display_log) {
        ROS_INFO("Listening has been stopped!");
    }
    if (!m_is_initialized_) {
        ROS_WARN("Socket client hasn't been initialized yet!");
        return;
    }
    if (display_log) {
        ROS_INFO("Attempt to uninitialize socket client!");
    }

    try {
        /* Attempt to close the TCP connection */
        if (display_log) {
            ROS_INFO("Closing connection to socket server...");
        }
        TeardownConnection();
        if (display_log) {
            ROS_INFO("Connection closed!");
        }
    } catch (const std::exception& e) {
        ROS_ERROR(" udp_err_8: Uninitialize socket client failed(error: %s)!", e.what());
    }
    m_is_initialized_ = false;
}

int UdpClient::StartConnection(const bool display_log)
{
    StartListening(display_log);
    return 0;
}

bool UdpClient::SendMsgToSocketClient(const unsigned char *send_msg, const int len)
{
    int send_num = sendto(m_socket_client_fd_, send_msg, len, 0,
                              (struct sockaddr*)&m_server_addr_, m_server_addr_len_);
    if (send_num == len) {
    //        ROS_INFO("Send msg to controller success, len(%d >> %d), client fd = %d, server addr = %d",
    //                 len, send_num, m_socket_client_fd_, m_server_addr_);
        return true;
    }
    ROS_ERROR(" udp_err_12: SendMsgToSocketClient failed: error: %s, len(%d >> %d), client fd = %d, server addr = %d)",
                 strerror(errno), len, send_num, m_socket_client_fd_, m_server_addr_);
    return false;
}

bool UdpClient::ConstructAndSendMsg(const unsigned int data_len, const unsigned char *data)
{
    header_send_data_.head1 = 0xFF;
    header_send_data_.head2 = 0xFA;
    header_send_data_.major_version = 0x01;
    header_send_data_.second_version = 0x0;
    header_send_data_.cnt = ++cnt_send_;

    header_send_data_.sender_ip_1 = 0xA;
    header_send_data_.sender_ip_2 = 0x1E;
    header_send_data_.sender_ip_3 = 0x64;
    header_send_data_.sender_ip_4 = 0xC;
    header_send_data_.mid = 0xF000;
    header_send_data_.msg_len = data_len;

    memset(msg_send_, 0, __MIO_MSG_MAX_LEN__);
    memcpy(msg_send_, (unsigned char *)&header_send_data_, sizeof(HeaderSendData));
    memcpy(msg_send_ + sizeof(HeaderSendData), data, data_len);

    int msg_len = sizeof(HeaderSendData) + data_len;
    msg_send_[msg_len] = CalculateCheckSum(msg_send_, msg_len); // check sum locate at the last position
    ++msg_len;   // length increase owing to add check sum

    // LOG_INFO("nav_step_9: send_to_plc: mid = %x, len = %d", mid, msg_len);
//    for (int i = 0; i < msg_len; i++) {
//        LOG_INFO("[%d]%x #n", i + 1, msg_send_[i]);
//    }
//    LOG_INFO("#h");

    if (!SendMsgToSocketClient(msg_send_, msg_len)) {
         ROS_ERROR(" udp_err_9: send msg to socketClient failed ...... ");
        return false;
    }
    return true;
}

int UdpClient::SetupConnection(const bool display_log)
{
    if(display_log) {
        ROS_INFO("Do WSAStartup()...");
    }
    if(display_log) {
        ROS_INFO("Do socket()...");
    }
    /* Create the TCP/IP socket */
    m_socket_client_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if(display_log) {
        ROS_INFO("Remote socket do connect()...");
    }
//    ROS_INFO("sche ip = %s", m_ip_address.c_str());
    memset(&m_server_addr_, 0, sizeof(struct sockaddr_in));
    m_server_addr_.sin_family = AF_INET;
    m_server_addr_.sin_addr.s_addr = inet_addr(plc_ip_address_.c_str());	//server IP
    m_server_addr_.sin_port = htons(plc_socket_server_port_);	//set socket port

    memset(&m_client_addr_, 0, sizeof(struct sockaddr_in));
    m_client_addr_.sin_family = AF_INET;
    m_client_addr_.sin_addr.s_addr = INADDR_ANY;	   //client IP
    m_client_addr_.sin_port = htons(plc_socket_client_port_);	//set socket port

    m_server_addr_len_ = sizeof (m_server_addr_);
    m_client_addr_len_ = sizeof (m_client_addr_);

    if(::bind(m_socket_client_fd_, (struct sockaddr *)&m_client_addr_, m_client_addr_len_) < 0)
    {
        ROS_ERROR(" udp_err_0: udp bind error:%s", strerror(errno));
        return 3;
    }
    return 0;
}

void UdpClient::SocketClientThread()
{
    m_connect_status_ = CONNSTATE_WAIT_FOR_CONNECT;
    flag_stop_socket_client_ = false;
    int ret = 0;
    ROS_INFO(" attempt to connect plc udp: %s: %d", plc_ip_address_.c_str(), (int)plc_socket_server_port_);
    while(!flag_stop_socket_client_) {
        ret = SetupConnection(false);
        if(0 == ret) {
            break;
        }
//        ROS_WARN("Setup connection failed(error: %s, code: %d)", strerror(errno), ret);
        std::this_thread::sleep_for(std::chrono::milliseconds(plc_reconnect_interval_));
    }
    if(flag_stop_socket_client_) {
        return;
    }
    ROS_INFO("Remote client to AGV socket thread has started! fd = %d", m_socket_client_fd_);

    int len = 0;
    unsigned char msg_buffer[__MIO_MSG_MAX_LEN__];
    m_connect_status_ = CONNSTATE_CONNECTED;
    m_is_initialized_ = true;
    m_recv_data_deque_.clear();
    TimeCommon::Time last_t = TimeCommon::TimeNow();
    TimeCommon::Time now = last_t;

    while (!flag_stop_socket_client_)
    {
        //reset data buffer
        memset(msg_buffer, 0, __MIO_MSG_MAX_LEN__);

        len = ReadSocketBuffer(msg_buffer);
        now = TimeCommon::TimeNow();
        double cost_time = TimeCommon::ToSeconds(now - last_t);
        if (cost_time >= 2.0) {
            ROS_ERROR(" udp_err_1: receive AGV data len = %d, cycle period = %f", len, cost_time);
        }
        last_t = now;
        if (len < 0) {
            if (m_lost_recv_count_ >= plc_disconnect_count_thred_ && CONNSTATE_DISCONNECT != m_connect_status_) {
                m_connect_status_ = CONNSTATE_DISCONNECT;
                ROS_ERROR(" udp_err_10: recv failed, lost_recv_cnt: %d ", m_lost_recv_count_);
            }
            continue;
        }
        if(CONNSTATE_CONNECTED != m_connect_status_) {
            m_connect_status_ = CONNSTATE_CONNECTED;
        }

        std::unique_lock<std::mutex> lock(m_recv_data_mutex_);
        // ROS_INFO("R_step_0: Socket msg len = %d, conn = %d", len, m_connect_status_);
        for(int i = 0; i < len; ++i) {
            m_recv_data_deque_.push_back(msg_buffer[i]);
//            ROS_INFO("%x #n", msg_buffer[i]);
        }
//        ROS_INFO("#h");
        lock.unlock();
        m_data_cv_.notify_one();
    }
    m_connect_status_ = CONNSTATE_DISCONNECT;
    ROS_INFO("Remote client to AGV socket thread has shutdown");
}

int UdpClient::ReadSocketBuffer(unsigned char * const dest_buffer)
{
    //set block and timeout if necessary
    struct timeval timeout_val;
    memset(&timeout_val, 0, sizeof(timeout_val));
//    ROS_INFO("%d",plc_recv_timeout_);
    timeout_val.tv_sec = (int)(plc_recv_timeout_ / 1000000); // select timeout
    timeout_val.tv_usec = (int)(plc_recv_timeout_ % 1000000);

    fd_set read_fdset;
    FD_ZERO(&read_fdset);
    FD_SET(m_socket_client_fd_, &read_fdset);
    int ret = select(m_socket_client_fd_ + 1, &read_fdset, NULL, NULL,
                     (plc_recv_timeout_ > 0) ? &timeout_val : 0);
    if (ret == 0) {
//        ROS_WARN("Select timeout");
        ++m_lost_recv_count_;
        return -1;
    } else if(ret < 0) {
        ROS_ERROR(" udp_err_2: select error(%s)!", strerror(errno));
        ++m_lost_recv_count_;
        return -1;
    }

    unsigned char recv_buf[2 * __MIO_MSG_MAX_LEN__] = { 0 };
    int len = recvfrom(m_socket_client_fd_, (char *)recv_buf, sizeof(recv_buf), 0,
                       (struct sockaddr*)&m_client_addr_, (socklen_t *)&m_client_addr_len_);

    // 数据阻塞接收
    if (len == -1 && errno == EAGAIN) {
        ROS_ERROR(" udp_err_3: recv timeout ...... ");
        ++m_lost_recv_count_;
        return -1;
    }

    if (len > 0) {
        memcpy(dest_buffer, recv_buf, len);
        m_lost_recv_count_ = 0;
        return len;
    }
    // receive 0 byte
    if(0 == len && errno == EINTR) {
        m_lost_recv_count_ = 0;
        return len;
    }
    ROS_ERROR(" udp_err_5: receive schedule data len = %d", len);
    ++m_lost_recv_count_;
    return -1;
}

void UdpClient::StartListening(const bool display_log)
{
    if (display_log) {
        ROS_INFO("Attempt to start socket client thread ...");
    }
    if (nullptr == m_socket_client_thread_) {
        m_socket_client_thread_ = new std::thread(&UdpClient::SocketClientThread, this);
    }
}

void UdpClient::StopListening()
{
    try {
        flag_stop_socket_client_ = true;
        if((m_socket_client_thread_) != nullptr) {
            m_socket_client_thread_->join(); 
            delete m_socket_client_thread_; 
            m_socket_client_thread_ = nullptr;
        }
    } catch (const std::exception& e) {
        ROS_ERROR(" udp_err_6: Stop socket client listening failed(error: %s)!", e.what());
    }
}

void UdpClient::TeardownConnection() // Close the socket
{
    if (close(m_socket_client_fd_) < 0) {
        ROS_ERROR(" udp_err_7: Close socket failed! fd = %d, Error: %s", m_socket_client_fd_, strerror(errno));
    }
    m_socket_client_fd_ = -1;
}
}
