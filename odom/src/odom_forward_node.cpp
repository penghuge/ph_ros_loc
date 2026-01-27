#include <stdio.h>

#include <iostream>

#include <ros/ros.h>
#include "odom_forward.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_forward_node");
    std::unique_ptr<odom::OdomForward> ptr_odom_forward_ = std::make_unique<odom::OdomForward>();

    ros::Rate hz_pub(100);
    while (ros::ok()) {
        ros::spinOnce();
        hz_pub.sleep();
    }
    return 0;
}
