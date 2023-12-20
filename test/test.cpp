
#include <gtest/gtest.h>

#include "../include/ariac_collection_bot/ccs.hpp" 


class CompetitionARIAC_test:public Parts
{ 
  public:
  geometry_msgs::msg::Pose BuildPose(
      double x, double y, double z, geometry_msgs::msg::Quaternion orientation) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

  return pose;

}
};

CompetitionARIAC_test ctest;

TEST(competitionARIAC_test, TestBuildPose) {
    // Define test inputs
    double x = 1.0, y = 2.0, z = 3.0;
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = 0.1;
    orientation.y = 0.2;
    orientation.z = 0.3;
    orientation.w = 0.4;

    // Call BuildPose
    auto result_pose = ctest.BuildPose(x, y, z, orientation);

    // Check if the result matches the input
    EXPECT_DOUBLE_EQ(result_pose.position.x, x);
    EXPECT_DOUBLE_EQ(result_pose.position.y, y);
    EXPECT_DOUBLE_EQ(result_pose.position.z, z);
    EXPECT_DOUBLE_EQ(result_pose.orientation.x, orientation.x);
    EXPECT_DOUBLE_EQ(result_pose.orientation.y, orientation.y);
    EXPECT_DOUBLE_EQ(result_pose.orientation.z, orientation.z);
    EXPECT_DOUBLE_EQ(result_pose.orientation.w, orientation.w);
}
