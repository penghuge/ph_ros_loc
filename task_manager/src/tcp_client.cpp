#include "tcp_client.h"

#include <signal.h>

namespace pallet_idenfity
{
TCPClient::TCPClient()
{
    sock_ = -1;
    m_connect_time_ = 2000;
}

bool TCPClient::SetupConnection(const std::string &m_ip_address, const int m_socket_port)
{
    ROS_INFO("mangzou_tcp_step_0: attempting to connect m4: %s: %d", m_ip_address.c_str(), m_socket_port);

    struct sockaddr_in client_addr;
    /* Initialize the buffer */
    memset(&client_addr, 0, sizeof(struct sockaddr_in));
    /* Setup the socket client inet address structure */
    // Internet protocol address family
    client_addr.sin_family = AF_INET;
    // TCP/IP port number
    client_addr.sin_port = htons(m_socket_port);
    // Convert IP string to numerical address
    client_addr.sin_addr.s_addr = inet_addr(m_ip_address.c_str());
    int conn_return = 0;

    try {
        while (m_connect_time_) {
            usleep(50000);
            //			std::cout << "mangzou_tcp_step_1: connect m4 cost time: " << totle_num - m_connect_time_
            //<< std::endl; 			ROS_INFO("mangzou_tcp_step_1: Connect m4 for the %d time", totle_num -
            // m_connect_time_);
            if (m_socket_client_fd >= 0) {
                if (close(m_socket_client_fd) < 0) {
                    ROS_ERROR(" mangzou_tcp_err_8: Close failed! fd = %d, Error: %s", m_socket_client_fd,
                              strerror(errno));
                }
                m_socket_client_fd = -1;
            }
            /* Create the TCP/IP socket */
            if ((m_socket_client_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
                ROS_ERROR("mangzou_tcp_err_7: Socket client setup connection: socket() failed! Error: %s",
                          strerror(errno));
                continue;
            }
            signal(SIGPIPE, SIG_IGN);
            if ((conn_return =
                     connect(m_socket_client_fd, (struct sockaddr *)&client_addr, sizeof(struct sockaddr_in))) < 0) {
                m_connect_time_--;
                /* Check whether it is b/c it would block */
                if (errno != EINPROGRESS) {
                    ROS_ERROR("mangzou_tcp_err_6: setup connection: connect() failed! Error: %s", strerror(errno));
                    continue;
                }

                /* Use select to wait on the socket */
                int valid_opt = 0;
                int num_active_files = 0;
                // File descriptor set for monitoring I/O
                fd_set file_desc_set;

                /* Initialize and set the file descriptor set for select */
                FD_ZERO(&file_desc_set);
                FD_SET(m_socket_client_fd, &file_desc_set);

                /* Setup the timeout structure */
                // This structure will be used for setting our timeout values
                struct timeval timeout_val;
                // Initialize the buffer
                memset(&timeout_val, 0, sizeof(timeout_val));
                // Wait for specified time before throwing a timeout
                timeout_val.tv_sec = (int)(2000000 / 1000000);  // select timeout
                timeout_val.tv_usec = (int)(2000000 % 1000000);

                /* Wait for the OS to tell us that the connection is established! */
                num_active_files = select(m_socket_client_fd + 1, 0, &file_desc_set, 0, &timeout_val);

                /* Figure out what to do based on the output of select */
                if (num_active_files > 0) {
                    /* This is just a sanity check */
                    if (!FD_ISSET(m_socket_client_fd, &file_desc_set)) {
                        ROS_WARN("mangzou_tcp_err_5: setup connection: Unexpected file descriptor!");
                    }

                    /* Check for any errors on the socket - just to be sure */
                    socklen_t len = sizeof(int);
                    if (getsockopt(m_socket_client_fd, SOL_SOCKET, SO_ERROR, (void *)(&valid_opt), &len) < 0) {
                        ROS_WARN("mangzou_tcp_err_4: setup connection: getsockopt() failed!");
                    }

                    /* Check whether the opt value indicates error */
                    if (valid_opt) {
                        ROS_WARN("mangzou_tcp_err_3: setup connection: socket error on connect()!");
                    }
                } else if (num_active_files == 0) {
                    /* A timeout has occurred! */
                    ROS_WARN("mangzou_tcp_err_2: setup connection: select() timeout!");
                    continue;
                } else {
                    /* An error has occurred! */
                    ROS_WARN("mangzou_tcp_err_1: setup connection: select() failed !!!!!!");
                    continue;
                }
            } else {
                return true;
            }
        }
    } catch (const std::exception &e) {
        ROS_ERROR("mangzou_tcp_err_0: setup connection, Unknown exception occurred! %s", e.what());
        return false;
    }
    return false;
}

bool TCPClient::SetUp(std::string address, int port)
{
    if (sock_ == -1) {
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == -1) {
            ROS_ERROR(" mangzou_tcp_err_0: Could not create socket ");
        }
    }
    if ((signed)inet_addr(address.c_str()) == -1) {
        struct hostent *he;
        struct in_addr **addr_list;
        if ((he = gethostbyname(address.c_str())) == NULL) {
            herror("gethostbyname");
            ROS_ERROR(" mangzou_tcp_err_1: Failed to resolve hostname ");
            return false;
        }
        addr_list = (struct in_addr **)he->h_addr_list;
        for (int i = 0; addr_list[i] != NULL; i++) {
            server_.sin_addr = *addr_list[i];
            break;
        }
    } else {
        server_.sin_addr.s_addr = inet_addr(address.c_str());
    }
    server_.sin_family = AF_INET;
    server_.sin_port = htons(port);
    if (connect(sock_, (struct sockaddr *)&server_, sizeof(server_)) < 0) {
        ROS_ERROR(" mangzou_tcp_err_2: connect failed. ");
        return false;
    }
    return true;
}

bool TCPClient::Send(std::string data)
{
    if (sock_ != -1) {
        if (send(sock_, data.c_str(), strlen(data.c_str()), 0) < 0) {
            ROS_ERROR(" mangzou_tcp_err_3: Send failed ");
            return false;
        }
    } else {
        return false;
    }
    return true;
}

int TCPClient::ReadSocketBuffer(char *const buffer_data, const int size)
{
    int count = recv(sock_, buffer_data, size, 0);
    if (count < 0) {
        ROS_ERROR(" mangzou_tcp_err_5: receive failed ");
    }
    return count;
}

std::string TCPClient::read()
{
    char buffer[1] = {};
    std::string reply;
    while (buffer[0] != '\n') {
        if (recv(sock_, buffer, sizeof(buffer), 0) < 0) {
            ROS_ERROR(" mangzou_tcp_err_4: receive failed ");
            return "";
        }
        reply += buffer[0];
    }
    return reply;
}

void TCPClient::exit()
{
    close(sock_);
}

}  // namespace pallet_idenfity
