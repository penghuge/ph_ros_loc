

#include "reflector_location_node.h"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <cmath>
#include <string>

#include "reflector_location/location.h"
#include "reflector_location/location_option.h"

namespace {
constexpr char NODENAME[] = "reflector_location_node";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NODENAME);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_INFO("reflector_location_node start...");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto reflocation = std::make_shared<location::Location>();
  location::RefLocNode node(reflocation);

  ros::Rate rate(100);
  ros::AsyncSpinner s(1);
  s.start();
  while (!node.Init(&nh) && ros::ok()) {
    ROS_ERROR("reflector_location_node init failed. retry");
    rate.sleep();
  }
  ROS_INFO("reflector_location_node init success.");
  ros::waitForShutdown();
}
