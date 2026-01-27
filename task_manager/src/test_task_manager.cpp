#include <angles/angles.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include "task_manager.h"

class TaskManagerTestSuite : public ::testing::Test
{
   protected:
    task::TaskManager test_manager_obj_;

    // 创建 Pose
    geometry_msgs::Pose CreatePose(double x, double y, double yaw)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0;  // 保持 z 坐标为 0
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);  // 直接设置偏航角
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

    // 创建 Point
    geometry_msgs::Point CreatePoint(double x, double y, double z)
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
};

// 基础测试：路径左侧偏移
TEST_F(TaskManagerTestSuite, LeftSideBaseline)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 0, 0);
    auto pose = CreatePose(1.5, 0.5, 0);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_NEAR(dev.delta_x, -0.5, 0.001);
    EXPECT_NEAR(dev.delta_y, 0.5, 0.001);
    EXPECT_NEAR(dev.delta_theta, 0.0, 0.001);
    EXPECT_NEAR(dev.front_projection, 0.5, 0.001);
    EXPECT_NEAR(dev.rear_projection, 0.5, 0.001);
}

TEST_F(TaskManagerTestSuite, LeftSideBaseline1)
{
    geometry_msgs::Point A = CreatePoint(7.041, -88.037, 0);
    geometry_msgs::Point B = CreatePoint(2.768, -40.195, 0);
    auto pose = CreatePose(4.763, -62.496, 0);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_NEAR(dev.delta_x, -22.39, 0.001);
    EXPECT_NEAR(dev.delta_y, -0.003, 0.001);
    EXPECT_NEAR(dev.delta_theta, -1.659, 0.001);
    EXPECT_NEAR(dev.front_projection, -0.999, 0.001);
    EXPECT_NEAR(dev.rear_projection, 0.992, 0.001);
}

//  路径左侧偏移，角度 45度
TEST_F(TaskManagerTestSuite, LeftSideBaseline45)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 0, 0);
    auto pose = CreatePose(1.0, 0.0, M_PI_4);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    const double sqrt2 = std::sqrt(2);
    EXPECT_NEAR(dev.delta_x, -1.0, 0.001);
    EXPECT_NEAR(dev.delta_y, 0.0, 0.001);
    EXPECT_NEAR(dev.delta_theta, M_PI_4, 0.001);
    EXPECT_NEAR(dev.front_projection, 1 / sqrt2, 0.001);
    EXPECT_NEAR(dev.rear_projection, -1 / sqrt2, 0.001);
}

//  路径右侧测试
TEST_F(TaskManagerTestSuite, RightSideDeviation)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 0, 0);
    auto pose = CreatePose(1.0, -0.3, 0);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_NEAR(dev.delta_x, -1.0, 0.001);
    EXPECT_NEAR(dev.delta_y, -0.3, 0.001);
    EXPECT_NEAR(dev.delta_theta, 0.0, 0.001);
    EXPECT_NEAR(dev.front_projection, -0.3, 0.001);
    EXPECT_NEAR(dev.rear_projection, -0.3, 0.001);
}

//  路径右侧测试 45
TEST_F(TaskManagerTestSuite, RightSideDeviation45)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 0, 0);
    auto pose = CreatePose(0.0, -2.0, M_PI_4);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_NEAR(dev.delta_x, -2.0, 0.001);
    EXPECT_NEAR(dev.delta_y, -2.0, 0.001);
    EXPECT_NEAR(dev.delta_theta, M_PI_4, 0.001);
    EXPECT_NEAR(dev.front_projection, -2.0 + 1 / std::sqrt(2), 0.001);
    EXPECT_NEAR(dev.rear_projection, -2.0 - 1 / std::sqrt(2), 0.001);
}

//  过终点后的测试
TEST_F(TaskManagerTestSuite, BeyondEndPoint)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 0, 0);
    auto pose = CreatePose(2.5, 0.2, 0);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_NEAR(dev.delta_x, 0.5, 0.001);
    EXPECT_NEAR(dev.delta_y, 0.2, 0.001);
    EXPECT_NEAR(dev.delta_theta, 0.0, 0.001);
    EXPECT_NEAR(dev.front_projection, 0.2, 0.001);
    EXPECT_NEAR(dev.rear_projection, 0.2, 0.001);
}

// 零长度路径测试
TEST_F(TaskManagerTestSuite, ZeroLengthPath)
{
    geometry_msgs::Point A = CreatePoint(1, 2, 0);
    geometry_msgs::Point B = CreatePoint(1, 2, 0);
    auto pose = CreatePose(1.5, 2.5, 0);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_DOUBLE_EQ(dev.delta_x, 0.0);
    EXPECT_DOUBLE_EQ(dev.delta_y, 0.0);
    EXPECT_DOUBLE_EQ(dev.delta_theta, 0.0);
    EXPECT_DOUBLE_EQ(dev.front_projection, 0.0);
    EXPECT_DOUBLE_EQ(dev.rear_projection, 0.0);
}

// 车体旋转测试
TEST_F(TaskManagerTestSuite, RotatedOrientation)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 0, 0);
    auto pose = CreatePose(1.0, 0.0, M_PI_2);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_DOUBLE_EQ(dev.delta_x, -1.0);
    EXPECT_DOUBLE_EQ(dev.delta_y, 0.0);
    EXPECT_NEAR(dev.delta_theta, M_PI_2, 0.001);
    EXPECT_NEAR(dev.front_projection, 1.0, 0.001);
    EXPECT_NEAR(dev.rear_projection, -1.0, 0.001);
}

// 斜向路径测试
TEST_F(TaskManagerTestSuite, DiagonalPath)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 2, 0);
    auto pose = CreatePose(1.0, 1.5, 0);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    const double sqrt2 = std::sqrt(2);
    EXPECT_NEAR(dev.delta_x, -1.060, 0.001);
    EXPECT_NEAR(dev.delta_y, 0.353553, 0.001);
    EXPECT_NEAR(dev.delta_theta, -M_PI_4, 0.001);
    EXPECT_NEAR(dev.front_projection, -0.353553, 0.001);
    EXPECT_NEAR(dev.rear_projection, 1.06066, 0.001);
}

// 小车也是45度
TEST_F(TaskManagerTestSuite, DiagonalPath45)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 2, 0);
    auto pose = CreatePose(1.0, 1.5, M_PI_4);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    const double sqrt2 = std::sqrt(2);
    const double expected_dy = 0.353553;
    EXPECT_NEAR(dev.delta_x, -1.060, 0.001);
    EXPECT_NEAR(dev.delta_y, expected_dy, 0.001);
    EXPECT_NEAR(dev.delta_theta, 0.0, 0.001);
    EXPECT_NEAR(dev.front_projection, expected_dy, 0.001);
    EXPECT_NEAR(dev.rear_projection, expected_dy, 0.001);
}

// 小车135度
TEST_F(TaskManagerTestSuite, DiagonalPath135)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 2, 0);
    auto pose = CreatePose(1.0, 2.0, M_PI_4 + M_PI_2);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    const double sqrt2 = std::sqrt(2);
    const double expected_dy = 0.353553;
    EXPECT_NEAR(dev.delta_x, -0.7071, 0.001);
    EXPECT_NEAR(dev.delta_y, 1 / sqrt2, 0.001);
    EXPECT_NEAR(dev.delta_theta, M_PI_2, 0.001);
    EXPECT_NEAR(dev.front_projection, 1 + 1 / sqrt2, 0.001);
    EXPECT_NEAR(dev.rear_projection, -(1 - 1 / sqrt2), 0.001);
}

// 完全偏离路径测试
TEST_F(TaskManagerTestSuite, FullDeviation)
{
    geometry_msgs::Point A = CreatePoint(0, 0, 0);
    geometry_msgs::Point B = CreatePoint(2, 0, 0);
    auto pose = CreatePose(0.5, 1.2, 0);
    task::PathDeviation dev;
    test_manager_obj_.CalculateLinePathDeviation(A, B, pose, dev);

    EXPECT_NEAR(dev.delta_x, -1.5, 0.001);
    EXPECT_NEAR(dev.delta_y, 1.2, 0.001);
    EXPECT_NEAR(dev.front_projection, 1.2, 0.001);
    EXPECT_NEAR(dev.rear_projection, 1.2, 0.001);
}

TEST_F(TaskManagerTestSuite, RobotOnArc)
{
    geometry_msgs::Point A = CreatePoint(5, 0, 0);
    geometry_msgs::Point B = CreatePoint(0, -5, 0);
    geometry_msgs::Point center = CreatePoint(0, 0, 0);
    auto pose = CreatePose(3.54, -3.54, -M_PI_4);
    float raidus = 5.0;
    task::PathDeviation dev;
    test_manager_obj_.CalculateCurvePathDeviation(center, A, B, pose, raidus, dev);

    EXPECT_NEAR(dev.delta_x, -3.9269, 0.01);
    EXPECT_NEAR(dev.delta_y, 0.0, 0.01);
    EXPECT_NEAR(dev.delta_theta, M_PI_2, 0.001);
}

TEST_F(TaskManagerTestSuite, RobotOutSideArc)
{
    geometry_msgs::Point A = CreatePoint(5, 0, 0);
    geometry_msgs::Point B = CreatePoint(0, -5, 0);
    geometry_msgs::Point center = CreatePoint(0, 0, 0);
    auto pose = CreatePose(5, -5, 0);
    float raidus = 5.0;
    task::PathDeviation dev;
    test_manager_obj_.CalculateCurvePathDeviation(center, A, B, pose, raidus, dev);

    EXPECT_NEAR(dev.delta_x, -3.9269, 0.01);
    EXPECT_NEAR(dev.delta_y, 2.071, 0.01);
    EXPECT_NEAR(dev.delta_theta, M_PI_4 + M_PI_2, 0.001);
}

TEST_F(TaskManagerTestSuite, RobotInSideArc)
{
    geometry_msgs::Point A = CreatePoint(5, 0, 0);
    geometry_msgs::Point B = CreatePoint(0, -5, 0);
    geometry_msgs::Point center = CreatePoint(0, 0, 0);
    auto pose = CreatePose(3, -3, -M_PI_4);
    float raidus = 5.0;
    task::PathDeviation dev;
    test_manager_obj_.CalculateCurvePathDeviation(center, A, B, pose, raidus, dev);

    EXPECT_NEAR(dev.delta_x, -3.9269, 0.01);
    EXPECT_NEAR(dev.delta_y, -0.757, 0.01);
    EXPECT_NEAR(dev.delta_theta, M_PI_2, 0.001);
}

TEST_F(TaskManagerTestSuite, RobotAlreadyEnterArcS1)
{
    geometry_msgs::Point A = CreatePoint(6.87, -26.68, 0);
    geometry_msgs::Point B = CreatePoint(5.39, -28.2, 0);
    geometry_msgs::Point center = CreatePoint(5.377, -26.7, 0);
    auto pose = CreatePose(6.87, -26.6, -M_PI_2);
    float raidus = 1.5;
    task::PathDeviation dev;
    test_manager_obj_.CalculateCurvePathDeviation(center, A, B, pose, raidus, dev);

    EXPECT_NEAR(dev.delta_x, -2.4435, 0.01);
    EXPECT_NEAR(dev.delta_y, 0.0, 0.01);
    EXPECT_NEAR(dev.delta_theta, -0.066, 0.001);
}

TEST_F(TaskManagerTestSuite, RobotAlreadyEnterArcS2)
{
    geometry_msgs::Point A = CreatePoint(-0.75, 0.895, 0);
    geometry_msgs::Point B = CreatePoint(-2.75, 2.895, 0);
    geometry_msgs::Point center = CreatePoint(-0.75, 2.895, 0);
    auto pose = CreatePose(-0.9, 1.0, M_PI - 0.17);
    float raidus = 2.0; // 顺时针
    task::PathDeviation dev;
    test_manager_obj_.CalculateCurvePathDeviation(center, A, B, pose, raidus, dev);

    EXPECT_NEAR(dev.delta_x, -2.983, 0.01);
    EXPECT_NEAR(dev.delta_y, -0.099, 0.01);
    EXPECT_NEAR(dev.delta_theta, -0.091, 0.001);
}

TEST_F(TaskManagerTestSuite, RobotAlreadyEnterArcN2)
{
    geometry_msgs::Point A = CreatePoint(-0.75, 0.895, 0);
    geometry_msgs::Point B = CreatePoint(-2.75, 2.895, 0);
    geometry_msgs::Point center = CreatePoint(-0.75, 2.895, 0);
    auto pose = CreatePose(-2.75, 2.95, M_PI_2);
    float raidus = 2.0; // 逆时针
    task::PathDeviation dev;
    test_manager_obj_.CalculateCurvePathDeviation(center, A, B, pose, raidus, dev);

    EXPECT_NEAR(dev.delta_x, 0.0549, 0.01);
    EXPECT_NEAR(dev.delta_y, 0.0007, 0.01);
    EXPECT_NEAR(dev.delta_theta, 0.027, 0.001);
}

TEST_F(TaskManagerTestSuite, RobotAlreadyEnterArcN1)
{
    geometry_msgs::Point A = CreatePoint(4.39, -28.2, 0);
    geometry_msgs::Point B = CreatePoint(2.89, -29.7, 0);
    geometry_msgs::Point center = CreatePoint(4.39, -29.7, 0);
    auto pose = CreatePose(4.4, -28.2, M_PI - 0.0001);
    float raidus = -1.5;
    task::PathDeviation dev;
    test_manager_obj_.CalculateCurvePathDeviation(center, A, B, pose, raidus, dev);

    EXPECT_NEAR(dev.delta_x, -2.366, 0.01);
    EXPECT_NEAR(dev.delta_y, 0.0, 0.01);
    EXPECT_NEAR(dev.delta_theta, 0.006, 0.001);
}

// 启动测试
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_task_manager");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
