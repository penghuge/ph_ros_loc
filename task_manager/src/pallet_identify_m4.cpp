#include "pallet_identify_m4.h"

#include "common_msgs/common_time.h"
#include "pallet_identify_interface.h"

namespace pallet_idenfity
{
bool PalletIdentifyM4::InitPalletIdentify(const std::string ip_camera_dtof)
{
    m_m4_socket_client = new M4SocketClient(ip_camera_dtof, port_, m_connect_time_, check_connect_timeout_);
    if (!m_m4_socket_client->InitSocketClient()) {
        ROS_ERROR(" m4_err_30: InitSocketClient failed !!!!!!");
        return false;
    }
    if (!m_m4_socket_client->ConnectStatus()) {
        ROS_ERROR(" m4_err_31: ConnectStatus failed !!!!!!");
        return false;
    }
    ROS_ERROR(" m4_step_7: InitPalletIdentify success ......");
    return true;
}

// 业务上应该不会出现横向偏差大于 y_max_identify_[目前是0.3m] 距离, 如果出现，认为是误识别[识别到旁边托盘],
// 即使不是误识别，也是叉车大角度转向，此时可以不反馈
bool PalletIdentifyM4::CheckIdentifyDataNormal(const double x, const double y, const double theta)
{
    if (fabs(y) > y_max_identify_) {
        return false;
    }
    return true;
}

bool PalletIdentifyM4::GetPalletIdentifyData(PalletPose &pallet_identify_result)
{
    static ros::Time last_print_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed = current_time - last_print_time;
    if (!m_m4_socket_client->ConnectStatus()) {
        ROS_ERROR(" m4_err_32: ConnectStatus failed !!!!!!");
        return false;
    }
    bool valid_data = false;
    m_m4_socket_client->DataIsValid(valid_data);
    if (!valid_data) {
        return false;
    }
    double x = 0;
    double y = 0;
    double theta = 0;
    m_m4_socket_client->GetPalletData(x, y, theta);
    if (!CheckIdentifyDataNormal(x, y, theta)) {
        return false;
    }
    pallet_identify_result.x = x;
    pallet_identify_result.y = y;
    pallet_identify_result.theta = theta;

    if (elapsed.toSec() >= 0.1) {
        ROS_INFO("pi_step_42: pallet pose of camera frame: [%f, %f], yaw: [%f] degree\n", pallet_identify_result.x,
                 pallet_identify_result.y, pallet_identify_result.theta);
        last_print_time = current_time;
    }
    return true;
}

bool PalletIdentifyM4::TestPalletIdentifyData()
{
    PalletPose pose;
    double x = 0;
    double y = 0;
    double theta = 0;
    for (size_t i = 0; i < 10; ++i) {
        m_m4_socket_client->GetPalletData(x, y, theta);
        pose.x = x;
        pose.y = y;
        pose.theta = theta;
        ROS_INFO("pi_step_44: pose_pi_m4:[%f, %f], yaw: [%f] degree\n", pose.x, pose.y, pose.theta);
    }
    return true;
}

PalletIdentifyM4::PalletIdentifyM4()
{
}

PalletIdentifyM4::~PalletIdentifyM4()
{
    if (nullptr != m_m4_socket_client) {
        m_m4_socket_client->UninitSocketClient();
    }
}

}  // namespace pallet_idenfity
