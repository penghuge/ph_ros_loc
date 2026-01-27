#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "task_manager.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "task_manager");
    std::unique_ptr<task::TaskManager> ptr_task_manager_ = std::make_unique<task::TaskManager>();

	ros::Rate hz_pub(50);
    while (ros::ok()) {
        ptr_task_manager_->RunCycle();
        ros::spinOnce();
        hz_pub.sleep();
    }
	return 0;
}
