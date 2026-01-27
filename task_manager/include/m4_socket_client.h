#pragma once

#include <arpa/inet.h>  // for sockaddr_in, inet_addr, and htons
#include <fcntl.h>
#include <sys/ioctl.h>   // for using ioctl functionality for the socket input buffer
#include <sys/socket.h>  // for socket function definitions
#include <sys/time.h>    // for select timeout parameter
#include <sys/types.h>   // for fd data types
#include <unistd.h>      // for select functionality (e.g. FD_SET, etc...)

#include <atomic>
#include <condition_variable>
#include <iomanip>
#include <mutex>
#include <new>
#include <thread>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ros/ros.h>

#include "common_msgs/common_haeder.h"
#include "common_msgs/common_time.h"

namespace pallet_idenfity
{
union CharToUint {
    char c[4];
    unsigned int ui;
};

class M4SocketClient
{
   public:
    M4SocketClient(const std::string &ip, const int port, const int conne_t, const int check_conne_timeout = 2000);
    ~M4SocketClient();

    bool InitSocketClient();
    void UninitSocketClient();
    bool StartPalletIdentify();
    void GetPalletData(double &x, double &y, double &theta);
    void DataIsValid(bool &valid_data);
    common_header::ConnectStateEnum ConnectStatus()
    {
        if (!m_is_initialized_) {
            return common_header::CONNSTATE_DISCONNECT;
        }
        return m_connect_status;
    }

   private:
    bool SetupConnection();
    void StartListening();
    void SocketClientRecvThread();
    int ReadSocketBuffer(char *const dest_buffer, const int num_bytes_to_read, const unsigned int timeout_value = 0);
    bool ProcessPacketData(std::vector<char> &data_queue);
    bool FindPacketStart(std::vector<char> &data_queue);
    void StopListening();
    void TeardownConnection();
    bool CheckConnection();
    void SetPalletIdentifyData(const double x, const double y, const double theta);

   private:
    int size_one_frame_{32};
    std::thread *m_socket_client_recv_thread_ = nullptr;
    std::mutex data_mutex_, init_mutex_;
    std::condition_variable m_init_condition_;
    bool m_recv_data_ = false;
    std::atomic<bool> valid_data_;

    int size_union_ = 4;
    int index_x_ = 8;
    int index_y_ = 12;
    int index_theta_ = 16;
    CharToUint chat_to_uint_x_;
    CharToUint chat_to_uint_y_;
    CharToUint chat_to_uint_theta_;
    double x_pallet_identify_;
    double y_pallet_identify_;
    double theta_pallet_identify_;
    double um_to_mm = 0.001;

    int m_socket_client_fd_ = -1;
    bool m_is_initialized_ = false;
    common_header::ConnectStateEnum m_connect_status = common_header::CONNSTATE_DISCONNECT;

    std::string m_ip_address_;
    int m_socket_port_;
    int m_connect_time_;
    TimeCommon::Time m_last_update_data_time_;
    TimeCommon::Duration m_check_connection_timeout_;
};

}  // namespace pallet_idenfity
