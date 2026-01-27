#include "odom_forward.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

namespace odom
{
OdomForward::OdomForward()
{
    ros::NodeHandle nh;
    sub_odom_ = nh.subscribe("/odom", 1000, &OdomForward::OdomCallback, this);
    pub_odom_forward_ = nh.advertise<nav_msgs::Odometry>("odom_forward", 10);
}

OdomForward::~OdomForward()
{
}

void OdomForward::OdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    nav_msgs::Odometry odom_msgs = *msg;
    //航迹推算
    ros::Time current_time = odom_msgs.header.stamp;
    static ros::Time last_time = current_time;

    static double x = 0.0;
    static double agv_x = 0.0;
    static double y = 0.0;
    static double agv_y = 0.0;
    static double theta = 0.0;

    double dt = (current_time - last_time).toSec();
    double delta_x = odom_msgs.twist.twist.linear.x * dt;
    double delta_y = odom_msgs.twist.twist.linear.y * dt;
    double delta_theta = odom_msgs.twist.twist.angular.z * dt;

    theta += delta_theta;
    //角度归一化[-pi,pi)
    if (M_PI <= theta)
        theta = theta - 2 * M_PI;
    else if (-M_PI > theta)
        theta = theta + 2 * M_PI;

    x += delta_x * cos(theta) - delta_y * sin(theta);
    y += delta_x * sin(theta) + delta_y * cos(theta);

    odom_msgs.pose.pose.position.x = x;
    odom_msgs.pose.pose.position.y = y;
    odom_msgs.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    odom_msgs.pose.pose.orientation = odom_quat;
    pub_odom_forward_.publish(odom_msgs);

    //发布坐标变换
    static tf::TransformBroadcaster tf_broadcaster;

    tf::StampedTransform transform;

    transform.setOrigin(
        tf::Vector3(odom_msgs.pose.pose.position.x, odom_msgs.pose.pose.position.y, odom_msgs.pose.pose.position.z));

    // 设置旋转部分（四元数）
    tf::Quaternion rotation(odom_msgs.pose.pose.orientation.x, odom_msgs.pose.pose.orientation.y,
                            odom_msgs.pose.pose.orientation.z, odom_msgs.pose.pose.orientation.w);
    transform.setRotation(rotation);

    // 设置时间戳和坐标系
    transform.stamp_ = ros::Time::now();
    transform.frame_id_ = "odom";                  // 父坐标系
    transform.child_frame_id_ = "base_footprint";  // 子坐标系

    // 发送变换
    tf_broadcaster.sendTransform(transform);
    last_time = current_time;
}
}  // namespace odom