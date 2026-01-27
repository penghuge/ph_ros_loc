#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#include <ros/ros.h>

namespace pallet_idenfity
{
class TCPClient
{
   private:
    int m_socket_client_fd{-1};
    int sock_;
    int m_connect_time_{2000};
    struct sockaddr_in server_;

   public:
    TCPClient();
    bool SetupConnection(const std::string &m_ip_address, const int m_socket_port);
    bool SetUp(std::string address, int port);
    bool Send(std::string data);
    int ReadSocketBuffer(char *const buffer_data, const int size);
    std::string read();
    void exit();
};
}  // namespace pallet_idenfity
