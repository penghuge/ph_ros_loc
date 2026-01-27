#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include "plc_interaction.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "plc_interaction");
    std::unique_ptr<mio::PlcInteraction> ptr_plc_interaction_ = std::make_unique<mio::PlcInteraction>();

	ros::Rate hz_pub(100);
    while (ros::ok()) {
        ros::spinOnce();
        hz_pub.sleep();
    }
	return 0;
}
