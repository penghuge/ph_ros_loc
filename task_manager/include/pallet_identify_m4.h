#pragma once
#include <iostream>
#include <memory>

#include <ros/ros.h>

#include "m4_socket_client.h"
#include "pallet_identify_interface.h"
#include "tcp_client.h"

namespace pallet_idenfity
{
class PalletIdentifyM4 : public PalletIdentifyBase
{
   public:
    PalletIdentifyM4();
    ~PalletIdentifyM4();

    bool InitPalletIdentify(const std::string ip_camera_dtof);
    bool GetPalletIdentifyData(PalletPose &pallet_identify_result);
    bool CheckIdentifyDataNormal(const double x, const double y, const double theta);
    bool TestPalletIdentifyData();

   private:
    int port_{5501};
    int m_connect_time_ = 500;
    int check_connect_timeout_ = 2000;
    double y_max_identify_ = 500;  // 0.5m
    M4SocketClient *m_m4_socket_client = nullptr;
};

}  // namespace pallet_idenfity
