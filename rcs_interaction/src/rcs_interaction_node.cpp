#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include "rcs_interaction.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rcs_interaction");
    std::unique_ptr<rcs::RcsInteraction> ptr_rcs_interaction_ = std::make_unique<rcs::RcsInteraction>();

	ros::Rate hz_pub(100);
    while (ros::ok()) {
        ros::spinOnce();
        hz_pub.sleep();
    }
	return 0;
}
