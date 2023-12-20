
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

    geometry_msgs::msg::Pose MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2) {
      KDL::Frame f1;
      KDL::Frame f2;

      tf2::fromMsg(p1, f1);
      tf2::fromMsg(p2, f2);

      KDL::Frame f3 = f1 * f2;

      return tf2::toMsg(f3);
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


TEST(competitionARIAC_test, TestMultiplyPose)
{
  geometry_msgs::msg::Pose p1, p2, expected_result, result;

  // Initialize p1 with suitable values
  p1.position.x = 1.0;
  p1.position.y = 2.0;
  p1.position.z = 3.0;
  p1.orientation.x = 0.0;
  p1.orientation.y = 0.0;
  p1.orientation.z = 0.0;
  p1.orientation.w = 1.0;

  // Initialize p2 with suitable values
  p2.position.x = 2.0;
  p2.position.y = 3.0;
  p2.position.z = 4.0;
  p2.orientation.x = 0.0;
  p2.orientation.y = 0.0;
  p2.orientation.z = 0.0;
  p2.orientation.w = 1.0;

  // Set the expected_result based on the expected behavior of MultiplyPose
  expected_result.position.x = 3.0;
  expected_result.position.y = 5.0;
  expected_result.position.z = 7.0;
  expected_result.orientation.x = 0.0;
  expected_result.orientation.y = 0.0;
  expected_result.orientation.z = 0.0;
  expected_result.orientation.w = 1.0;

  result = ctest.MultiplyPose(p1, p2);

  // Add assertions based on the expected result
  ASSERT_NEAR(result.position.x, expected_result.position.x, 1e-5);
  ASSERT_NEAR(result.position.y, expected_result.position.y, 1e-5);
  ASSERT_NEAR(result.position.z, expected_result.position.z, 1e-5);
  
  ASSERT_NEAR(result.orientation.x, expected_result.orientation.x, 1e-5);
  ASSERT_NEAR(result.orientation.y, expected_result.orientation.y, 1e-5);
  ASSERT_NEAR(result.orientation.z, expected_result.orientation.z, 1e-5);
  ASSERT_NEAR(result.orientation.w, expected_result.orientation.w, 1e-5);
}