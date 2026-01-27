
#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "reflector_location_node");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  int res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}
