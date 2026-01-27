#ifndef PLC_COM_BASE_H
#define PLC_COM_BASE_H
#include <iostream>
#include <vector>
#include <deque>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <arpa/inet.h>		  // for sockaddr_in, inet_addr, and htons
#include <sys/ioctl.h>        // for using ioctl functionality for the socket input buffer
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "signal.h"
#include "unistd.h"

namespace mio {

#define __MIO_MSG_MAX_LEN__            1024
#define __MIO_MSG_MID_ID__             2

enum ConnectStateEnum
{
    CONNSTATE_DISCONNECT,
    CONNSTATE_WAIT_FOR_CONNECT,
    CONNSTATE_CONNECTED,
};

struct HeaderSendData {
    uint8_t head1 = 0;
    uint8_t head2 = 0;
    uint8_t major_version = 0;
    uint8_t second_version = 0;
    uint32_t cnt = 0;
    uint8_t sender_ip_4 = 0;
    uint8_t sender_ip_3 = 0;
    uint8_t sender_ip_2 = 0;
    uint8_t sender_ip_1 = 0;
    uint16_t mid = 0;
    uint16_t msg_len = 0;
    uint64_t reserve = 0;
};

class UdpComBase {
    public:
        UdpComBase() {}
        virtual ~UdpComBase() {}
        ConnectStateEnum ConnectStatus() {
            return m_connect_status_;
        }

        bool CheckCheckSum(const std::deque<unsigned char> &msg_que, int len)
        {
            unsigned char check_sum = 0;

            //add from cmd bit to check sum bit
            for (int i = __MIO_MSG_MID_ID__; i < len; i++) {
                check_sum += ~msg_que[i];
            }
            check_sum = ~check_sum;
        // 	LOG_INFO("Controller client msg data check sum = %x", check_sum);

            if (0 == check_sum) {
                return true;
            }
            ROS_WARN("Check sum of controller client msg indicates an invalid msg!");
            return false;
        }

        bool CheckCheckSum(const unsigned char * const msg_que, int len)
        {
            unsigned char check_sum = 0;
            //add from cmd bit to check sum bit
            for (int i = __MIO_MSG_MID_ID__; i < len; i++) {
                check_sum += ~msg_que[i];
            }
            check_sum = ~check_sum;
            // 	LOG_INFO("Controller client msg data check sum = %x", check_sum);

            if (0 == check_sum) {
                return true;
            }
            ROS_WARN("Check sum of controller client msg indicates an invalid msg!");
            return false;
        }

        /**********************************************************************************
        function: calculate check sum for sending msg
        **********************************************************************************/
        unsigned char CalculateCheckSum(const unsigned char *send_msg, const int msg_len)
        {
            unsigned char check_sum = 0;
            //check sum includes cmd, len, data
            for (int i = __MIO_MSG_MID_ID__; i < msg_len; ++i) {
                check_sum += ~send_msg[i];
            }
            return check_sum;
        }

        int GetSocketData(std::vector<unsigned char> &data_que)
        {
            std::unique_lock<std::mutex> lock(m_recv_data_mutex_);
            m_data_cv_.wait(lock, [&] { return !m_recv_data_deque_.empty(); });
            data_que.insert(data_que.end(), std::make_move_iterator(m_recv_data_deque_.begin()), 
                                            std::make_move_iterator(m_recv_data_deque_.end()));
            int len = m_recv_data_deque_.size();
            m_recv_data_deque_.clear();
            return len;
        }

    public:
        std::vector<unsigned char> m_recv_data_deque_;
        std::condition_variable m_data_cv_;
        std::mutex m_recv_data_mutex_;
        ConnectStateEnum m_connect_status_ = CONNSTATE_WAIT_FOR_CONNECT;
        HeaderSendData header_send_data_;
};

} // namespace mio
#endif
