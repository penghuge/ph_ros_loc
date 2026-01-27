#include "m4_socket_client.h"

#include <signal.h>

namespace pallet_idenfity
{
M4SocketClient::M4SocketClient(const std::string &ip, const int port, const int conne_t, const int check_conne_timeout)
    : m_ip_address_(ip), m_socket_port_(port), m_connect_time_(conne_t), m_last_update_data_time_(TimeCommon::TimeNow())
{
    valid_data_ = false;
    m_check_connection_timeout_ = TimeCommon::FromMilliseconds(check_conne_timeout);
}

M4SocketClient::~M4SocketClient()
{
    UninitSocketClient();
}

/**********************************************************************************
function: initialize TCP/IP client
**********************************************************************************/
bool M4SocketClient::InitSocketClient()
{
    if (m_is_initialized_) {
        ROS_INFO("m4_err_0: m4 has already been initialized.");
        return true;
    }
    ROS_INFO("m4_step_1: attempt to setup m4 connection...");

    // build TCP/IP connection
    if (!SetupConnection()) {
        return false;
    }
    ROS_INFO("m4_step_11: m4 connection has been setup!");
    if (!StartPalletIdentify()) {  // send start pallet identify cmd
        ROS_INFO("m4_err_24: StartPalletIdentify failed .....");
        return false;
    }
    // start the thread for receiving data
    StartListening();

    std::unique_lock<std::mutex> lock(init_mutex_);
    m_is_initialized_ = true;
    lock.unlock();
    // notify all lockers who are waiting for initialization
    m_init_condition_.notify_all();
    return true;
}

/**********************************************************************************
function: Establish a TCP/IP connection to the unit
**********************************************************************************/
bool M4SocketClient::SetupConnection()
{
    m_connect_status = common_header::CONNSTATE_WAIT_FOR_CONNECT;
    ROS_INFO("m4_step_2: attempting to connect m4 : %s: %d", m_ip_address_.c_str(), (int)m_socket_port_);

    struct sockaddr_in client_addr;
    /* Initialize the buffer */
    memset(&client_addr, 0, sizeof(struct sockaddr_in));
    /* Setup the socket client inet address structure */
    // Internet protocol address family
    client_addr.sin_family = AF_INET;
    // TCP/IP port number
    client_addr.sin_port = htons(m_socket_port_);
    // Convert IP string to numerical address
    client_addr.sin_addr.s_addr = inet_addr(m_ip_address_.c_str());

    try {
        int conn_return;
        int totle_num = m_connect_time_ + 1;
        while (m_connect_time_) {
            usleep(50000);
            ROS_INFO("m4_step_3: Connect m4 for the %d time!", totle_num - m_connect_time_);
            if (m_socket_client_fd_ >= 0) {
                if (close(m_socket_client_fd_) < 0) {
                    ROS_ERROR("m4_err_1: Close failed! fd = %d, Error: %s", m_socket_client_fd_, strerror(errno));
                }
                m_socket_client_fd_ = -1;
            }
            /* Create the TCP/IP socket */
            if ((m_socket_client_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
                ROS_ERROR("m4_err_2: Socket client setup connection: socket() failed! Error: %s", strerror(errno));
                continue;
            }
            signal(SIGPIPE, SIG_IGN);
            if ((conn_return =
                     connect(m_socket_client_fd_, (struct sockaddr *)&client_addr, sizeof(struct sockaddr_in))) < 0) {
                m_connect_time_--;
                /* Check whether it is b/c it would block */
                if (errno != EINPROGRESS) {
                    ROS_ERROR("m4_err_3: m4 setup connection: connect() failed! Error: %s", strerror(errno));
                    continue;
                }

                /* Use select to wait on the socket */
                int valid_opt = 0;
                int num_active_files = 0;
                // File descriptor set for monitoring I/O
                fd_set file_desc_set;

                /* Initialize and set the file descriptor set for select */
                FD_ZERO(&file_desc_set);
                FD_SET(m_socket_client_fd_, &file_desc_set);

                /* Setup the timeout structure */
                // This structure will be used for setting our timeout values
                struct timeval timeout_val;
                // Initialize the buffer
                memset(&timeout_val, 0, sizeof(timeout_val));
                // Wait for specified time before throwing a timeout
                timeout_val.tv_sec = 2.0;  // select timeout
                timeout_val.tv_usec = 0.0;

                /* Wait for the OS to tell us that the connection is established! */
                num_active_files = select(m_socket_client_fd_ + 1, 0, &file_desc_set, 0, &timeout_val);

                /* Figure out what to do based on the output of select */
                if (num_active_files > 0) {
                    /* This is just a sanity check */
                    if (!FD_ISSET(m_socket_client_fd_, &file_desc_set)) {
                        ROS_WARN("m4_err_4: m4 setup connection: Unexpected file descriptor!");
                    }

                    /* Check for any errors on the socket - just to be sure */
                    socklen_t len = sizeof(int);
                    if (getsockopt(m_socket_client_fd_, SOL_SOCKET, SO_ERROR, (void *)(&valid_opt), &len) < 0) {
                        ROS_WARN("m4_err_5: m4 setup connection: getsockopt() failed!");
                    }

                    /* Check whether the opt value indicates error */
                    if (valid_opt) {
                        ROS_WARN("m4_err_6: m4 setup connection: socket error on connect()!");
                    }
                } else if (num_active_files == 0) {
                    /* A timeout has occurred! */
                    ROS_WARN("m4_err_7: m4 setup connection: select() timeout!");
                    continue;
                } else {
                    /* An error has occurred! */
                    ROS_WARN("m4_err_8: m4 setup connection: select() failed!");
                    continue;
                }
            } else {
                m_connect_status = common_header::CONNSTATE_CONNECTED;
                return true;
            }
        }
    } catch (const std::exception &e) {
        ROS_ERROR("m4_err_9: m4 setup connection: Unknown exception occurred! %s", e.what());
    }
    return false;
}

/**********************************************************************************
function: Activates the buffer monitor for the driver
**********************************************************************************/
void M4SocketClient::StartListening()
{
    /* Try to start the monitor */
    try {
        ROS_INFO("m4_step_4 : Attempt to start listening m4...");
        if (nullptr == m_socket_client_recv_thread_) {
            m_socket_client_recv_thread_ = new std::thread(&M4SocketClient::SocketClientRecvThread, this);
        }
    } catch (const std::exception &e) { /* Handle a thread exception */
        ROS_ERROR("m4_err_10: m4 start listening: Unknown exception!!!%s", e.what());
    }
}

bool M4SocketClient::StartPalletIdentify()
{
    char cmd_start_identify[4] = {0x60, 0x04, 0x00, 0x00};
    if (m_socket_client_fd_ >= 0) {
        if (send(m_socket_client_fd_, cmd_start_identify, 4, 0) < 0) {
            ROS_ERROR(" m4_err_22: send m4 start pallet identify cmd failed ");
            return false;
        }
    } else {
        ROS_ERROR(" m4_err_23: m_socket_client_fd_ < 0 ");
        return false;
    }
    return true;
}

/**********************************************************************************
function: TCP/IP client receive data thread
**********************************************************************************/
void M4SocketClient::SocketClientRecvThread()
{
    try {
        {
            // suspend thread, waiting for initialization is completed.
            std::unique_lock<std::mutex> init_lock(init_mutex_);
            while (!m_is_initialized_) {
                ROS_INFO("m_init_condition_ waiting...");
                m_init_condition_.wait(init_lock);
                ROS_INFO("m_init_condition_ notified...");
            }
            init_lock.unlock();
        }
        ROS_INFO("m4_step_5 : receive m4 data thread is started ......");

        int len = 0;
        char data_buffer[size_one_frame_];
        std::vector<char> socket_data_queue;
        socket_data_queue.clear();

        m_recv_data_ = true;
        while (m_recv_data_) {
            TimeCommon::Time start_time = TimeCommon::TimeNow();
            // reset data buffer
            len = 0;
            memset(data_buffer, 0, size_one_frame_);
            len = ReadSocketBuffer(data_buffer, size_one_frame_, 1000000);
            if (len < 0) {
                ROS_ERROR("m4_err_21: read socket buffer len < 0 ");
                continue;
            }
            size_t que_size = socket_data_queue.size();
            socket_data_queue.resize(que_size + (size_t)len);
            for (int i = 0; i < len; ++i) {
                socket_data_queue[que_size + i] = data_buffer[i];
            }

            double rec_cost = TimeCommon::ToSeconds(TimeCommon::TimeNow() - start_time) * 1000.0;

            if (!ProcessPacketData(socket_data_queue)) {
                // ROS_INFO("m4_err_20: process data not perfect......");
            }
            m_last_update_data_time_ = TimeCommon::TimeNow();
            double total_cost = TimeCommon::ToSeconds(m_last_update_data_time_ - start_time) * 1000.0;
            if (total_cost > 200) {  // pallet identify result will less than 5HZ
                ROS_INFO("m4_step_8 : rec_total_cost_time: %f, %f", rec_cost, total_cost);
            }
        }
    } catch (const std::exception &e) {
        ROS_ERROR("m4_err_12: SocketClientRecvThread Unexpected: %s", e.what());
    }
    ROS_INFO("m4_err_11: receive m4 data thread has been shut down !!!!!!");
}

/**********************************************************************************
function: Attempt to read a certain number of bytes from the stream
* \param *dest_buffer A pointer to the destination buffer
* \param num_bytes_to_read The number of bytes to read into the buffer
* \param timeout_value The number of microseconds allowed between subsequent bytes in a message
* \return True if the number of requested bytes were successfully read
**********************************************************************************/
int M4SocketClient::ReadSocketBuffer(char *const dest_buffer, const int num_bytes_to_read,
                                     const unsigned int timeout_value)
{
    /* Some helpful variables */
    int num_bytes_read = 0;
    int num_active_files = 0;

    // File descriptor set for monitoring I/O
    fd_set file_desc_set;

    /* Initialize and set the file descriptor set for select */
    FD_ZERO(&file_desc_set);
    FD_SET(m_socket_client_fd_, &file_desc_set);

    /* Setup the timeout structure */
    // This structure will be used for setting our timeout values
    struct timeval timeout_val;
    // Initialize the buffer
    memset(&timeout_val, 0, sizeof(timeout_val));
    // Wait for specified time before throwing a timeout
    timeout_val.tv_sec = (int)(timeout_value / 1000000);  // select timeout
    timeout_val.tv_usec = (int)(timeout_value % 1000000);

    /* Wait for the OS to tell us that data is waiting! */
    num_active_files = select(m_socket_client_fd_ + 1, &file_desc_set, 0, 0, (timeout_value > 0) ? &timeout_val : 0);

    /* Figure out what to do based on the output of select */
    if (num_active_files > 0) {
        if (FD_ISSET(m_socket_client_fd_, &file_desc_set)) {
            num_bytes_read = recv(m_socket_client_fd_, dest_buffer, num_bytes_to_read, 0);
            return num_bytes_read;
        }
    } else if (num_active_files == 0) {
        /* A timeout has occurred! */
        //        ROS_WARN("M4SocketClient::ReadSocketBuffer: select() timeout!");
    } else {
        /* An error has occurred! */
        ROS_WARN("m4_err_13: M4SocketClient readSocketBuffer select failed !!!!!!");
    }
    return -1;
}

void M4SocketClient::SetPalletIdentifyData(const double x, const double y, const double theta)
{
    std::lock_guard<std::mutex> mylock(data_mutex_);
    x_pallet_identify_ = x;
    y_pallet_identify_ = y;
    theta_pallet_identify_ = theta;
}

void M4SocketClient::GetPalletData(double &x, double &y, double &theta)
{
    std::lock_guard<std::mutex> mylock(data_mutex_);
    x = x_pallet_identify_;
    y = y_pallet_identify_;
    theta = theta_pallet_identify_;
}

void M4SocketClient::DataIsValid(bool &valid_data)
{
    valid_data = valid_data_;
}

bool M4SocketClient::ProcessPacketData(std::vector<char> &data_queue)
{
    if (!FindPacketStart(data_queue)) {
        ROS_INFO("m4_err_18: Can't find header, clear vector size = %d!", data_queue.size());
        return false;
    }

    if (data_queue.size() < size_one_frame_) {
        ROS_INFO("m4_err_19: data_queue size less than size_one_frame ......");
        return false;
    }
    if (data_queue[0] || data_queue[1]) {
        data_queue.erase(data_queue.begin(), data_queue.begin() + size_one_frame_);
        valid_data_ = false;
        // ROS_INFO("m4_err_33: rec m4 one frame data, ret: [%d, %d]", data_queue[0], data_queue[1]);
        return false;
    }

    // ROS_INFO("m4_step_6_0: origin:  %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x",
    //             data_queue[0], data_queue[1], data_queue[2], data_queue[3], data_queue[4], data_queue[5],
    //             data_queue[6], data_queue[7], data_queue[8], data_queue[9], data_queue[10], data_queue[11],
    //             data_queue[12], data_queue[13], data_queue[14], data_queue[15], data_queue[16], data_queue[17],
    //             data_queue[18], data_queue[19]);

    char pre_x = data_queue[4];
    char pre_y = data_queue[5];
    char pre_theta = data_queue[6];
    for (size_t i = 0; i < size_union_; ++i) {
        chat_to_uint_x_.c[size_union_ - i - 1] = data_queue[index_x_ + i];
        chat_to_uint_y_.c[size_union_ - i - 1] = data_queue[index_y_ + i];
        chat_to_uint_theta_.c[size_union_ - i - 1] = data_queue[index_theta_ + i];
    }

    double x = chat_to_uint_x_.ui * um_to_mm;
    double y = chat_to_uint_y_.ui * um_to_mm;
    double theta = chat_to_uint_theta_.ui * 0.001;  // degree
    if (1 == pre_x) {
        x *= -1;
    }
    if (1 == pre_y) {
        y *= -1;
    }
    if (1 == pre_theta) {
        theta *= -1;
    }

    // ROS_INFO("m4_step_6_2: x:%d, y:%d, theta:%d", chat_to_uint_x_.ui,chat_to_uint_y_.ui,chat_to_uint_theta_.ui);
    ROS_INFO("m4_step_6_3: x:%f, y:%f, theta:%f", x, y, theta);

    SetPalletIdentifyData(x, y, theta);

    // ROS_INFO("m4_step_6_6: pi_data,ret:%d,length:%d,pre_x:%d,pre_y:%d,pre_theta:%d,x:%d,y:%d,theta:%d",
    //             one_frame_m4_.ret,one_frame_m4_.l_length,one_frame_m4_.pre_x,one_frame_m4_.pre_y,one_frame_m4_.pre_theta,
    // 			chat_to_uint_x_.ui,chat_to_uint_y_.ui,chat_to_uint_theta_.ui);
    valid_data_ = true;
    data_queue.erase(data_queue.begin(), data_queue.begin() + size_one_frame_);
    return true;
}

bool M4SocketClient::FindPacketStart(std::vector<char> &data_que)
{
    size_t size = data_que.size();
    if (size < size_one_frame_) {
        return false;
    }
    for (std::vector<char>::iterator data = data_que.begin(); data != data_que.end() - 8; ++data) {
        bool flag_3 = (((unsigned char)*(data + 2)) == 0x00) && (((unsigned char)*(data + 3)) == 0x20);
        bool flag_4 = (((unsigned char)*(data + 4)) == 0x00) || (((unsigned char)*(data + 4)) == 0x01);
        bool flag_5 = (((unsigned char)*(data + 5)) == 0x00) || (((unsigned char)*(data + 5)) == 0x01);
        bool flag_6 = (((unsigned char)*(data + 6)) == 0x00) || (((unsigned char)*(data + 6)) == 0x01);
        bool flag_7 = (((unsigned char)*(data + 7)) == 0x00) || (((unsigned char)*(data + 7)) == 0x01);

        if (flag_3 && flag_4 && flag_5 && flag_6 && flag_7) {
            data_que.erase(data_que.begin(), data);
            return true;
        }
    }

    return false;
}

/**********************************************************************************
function: close TCP/IP listening and connection
**********************************************************************************/
void M4SocketClient::UninitSocketClient()
{
    if (!m_is_initialized_) {
        ROS_WARN("m4_err_16: UninitSocketClient device isn't initialized!!!");
        return;
    }
    ROS_INFO("Attempting to uninitialize socket client ...");

    try {
        /* Attempt to cancel the buffer monitor */
        ROS_INFO("Attempting to stop socket client listening...");
        StopListening();
        ROS_INFO("Listening has been stopped!");
        /* Attempt to close the tcp connection */
        ROS_INFO("Closing connection ...");
        TeardownConnection();
        ROS_INFO("Connection closed!");
    } catch (const std::exception &e) {
        ROS_ERROR("m4_err_17: UninitSocketClient Unknown exception!!! %s", e.what());
    }

    /* Mark the device as uninitialized */
    m_is_initialized_ = false;
    m_connect_status = common_header::CONNSTATE_DISCONNECT;
    ROS_INFO("Uninit socket client successfully!");
    /* Success! */
}

/**********************************************************************************
function: stop TCP/IP listening, shut down the data receiving thread
**********************************************************************************/
void M4SocketClient::StopListening()
{
    try {
        m_recv_data_ = false;
        if (nullptr != m_socket_client_recv_thread_) {
            m_socket_client_recv_thread_->join();
            delete m_socket_client_recv_thread_;
            m_socket_client_recv_thread_ = nullptr;
        }
    } catch (const std::exception &e) {
        ROS_ERROR("m4_err_15: StopListening: Unknown exception! %s", e.what());
    }
}

/**********************************************************************************
function: Teardown TCP connection
**********************************************************************************/
void M4SocketClient::TeardownConnection()
{
    /* Close the socket! */
    if (close(m_socket_client_fd_) < 0) {
        ROS_ERROR("m4_err_14: TeardownConnection: close() failed!");
    }
    m_socket_client_fd_ = -1;
    m_connect_status = common_header::CONNSTATE_DISCONNECT;
}

bool M4SocketClient::CheckConnection()
{
    if (!ConnectStatus()) {
        return false;
    }
    if ((TimeCommon::TimeNow() - m_last_update_data_time_) > m_check_connection_timeout_) {
        UninitSocketClient();
        return false;
    }
    return true;
}

}  // namespace pallet_idenfity
