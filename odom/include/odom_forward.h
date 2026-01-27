#ifndef ODOM_FORWARD_H
#define ODOM_FORWARD_H
#include <ros/ros.h>

#include "nav_msgs/Odometry.h"

namespace odom
{
class OdomForward
{
   public:
    OdomForward();
    ~OdomForward();

    void OdomCallback(const nav_msgs::OdometryConstPtr &msg);

   private:
    ros::Subscriber sub_odom_;
    ros::Publisher pub_odom_forward_;
};

}  // namespace odom

#endif