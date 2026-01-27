#ifndef PLC_UDP_CLIENT_H
#define PLC_UDP_CLIENT_H
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "plc_com_base.h"
#include "common_time.h"

namespace mio {

class UdpClient : public UdpComBase {
public:
    UdpClient(std::string& ip_address_plc, const int socket_server_port_nav, const int socket_client_port_nav);
    ~UdpClient();

    void UninitSocket(const bool display_log);
    int  StartConnection(const bool display_log);
    bool SendMsgToSocketClient(const unsigned char *send_msg, const int len);
    bool ConstructAndSendMsg(const unsigned int data_len, const unsigned char *data);

private:
    int  SetupConnection(const bool display_log);
    void SocketClientThread();
    int  ReadSocketBuffer(unsigned char * const dest_buffer);
    void StartListening(const bool display_log = false);
    void StopListening();
    void TeardownConnection();

private:
    std::thread *m_socket_client_thread_ = nullptr;

    std::atomic<bool> flag_stop_socket_client_{true};
    bool m_is_initialized_ = false;
    int  m_socket_client_fd_ = -1;

    struct sockaddr_in m_server_addr_;
    struct sockaddr_in m_client_addr_;

    // struct sockaddr_in m_server_addr1;
    // struct sockaddr_in m_client_addr1;

    int m_server_addr_len_ = 0;
    int m_client_addr_len_ = 0;
    // int m_server_addr1_len = 0;
    // int m_client_addr1_len = 0;

    std::string plc_ip_address_ ="";
    int plc_socket_server_port_ = 0;
    int plc_socket_client_port_ = 0;
    int plc_reconnect_count_ = 20;
    int plc_recv_timeout_ = 800000;
    int plc_send_timeout_ = 200000; // unit: us
    int plc_conn_timeout_ = 30000;
    int plc_reconnect_interval_ = 500; // unit: ms
    int plc_disconnect_count_thred_ = 100;

    int m_lost_recv_count_ = 0; /* m_disconnect_count_thred*/
    // int m_count = 0;
    uint32_t cnt_send_ = 0;
    unsigned char msg_send_[__MIO_MSG_MAX_LEN__];

};
}
#endif
