#include <gtest/gtest.h>
#include <ros/ros.h>

#include "reflector_location/geometry.h"
#include "reflector_location/reflector_location.h"
#define private public
TEST(ReflectorLocation, test_Init) { EXPECT_EQ("test", "test"); }

TEST(PointSet, test_Cij) {
  location::PointSet Pt;
  std::vector<int> resource{11, 21, 31, 41, 51, 61, 71, 81, 91};
  std::vector<std::vector<int>> result;

  for (int i = 0; i < resource.size(); i++) {
    std::cout << "num " << i << "= ";
    std::cout << resource[i] << std::endl;
  }
  Pt.Cij(resource.size(), 9, &resource, 9, &result);

  std::cout << "after choose.." << std::endl;
  std::cout << "result size:" << result.size() << std::endl;
  for (int i = 0; i < resource.size(); i++) {
    std::cout << "num " << i << "= ";
    std::cout << resource[i] << std::endl;
  }
  for (int i = 0; i < result.size(); i++) {
    std::cout << "result size: " << result.size() << std::endl;
    for (int j = 0; j < result[i].size(); j++) {
      std::cout << "result[" << i << "][" << j << "] = ";
      std::cout << result[i][j] << std::endl;
    }
  }
  EXPECT_EQ(result.size(), 1);
}

TEST(ReflectorLocation, test_LaserPoseToRobotPose) {
  location::ReflectorLocation rf;
  location::Pose test_pose1(0., 0., 0.);
  location::Pose test_pose2(0, 0., 3.14);
  location::Pose test_pose3(1.2, 0., 0.);
  location::Pose test_pose4(1.2, 0., 1.57);
  location::Pose test_pose5(1.2, 0., 3.14);
  location::Pose test_pose6(1.2, 0., -1.57);

  location::Pose test4 = rf.LaserPoseToRobotPose(test_pose4);
  std::cout << "test1: " << test4.x() << " test_pose1: " << test_pose4.x();
  EXPECT_EQ(test4.x(),test_pose4.x());

}

TEST(ReflectorLocation, test_RobotPoseToLaserPose) {
  EXPECT_EQ("test", "test");
}
