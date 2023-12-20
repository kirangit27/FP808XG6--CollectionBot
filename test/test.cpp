// Copyright (c) 2023 808X_Group6
/**
 * @file test.cpp
 * @brief testing file implementation
 * @version 0.1
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <gtest/gtest.h>

#include "../include/ariac_collection_bot/ccs.hpp"

class CompetitionARIAC_test : public Parts {
 public:
  geometry_msgs::msg::Pose BuildPose(
      double x, double y, double z,
      geometry_msgs::msg::Quaternion orientation) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
  }

  geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1,
                                        geometry_msgs::msg::Pose p2) {
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
  }

  geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p,
                                                   double y) {
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
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

TEST(competitionARIAC_test, TestMultiplyPose) {
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

TEST(competitionARIAC_test, TestConstants) {
  // Test kit_tray_thickness_
  ASSERT_DOUBLE_EQ(ctest.kit_tray_thickness_, 0.01);
  // Test drop_height_
  ASSERT_DOUBLE_EQ(ctest.drop_height_, 0.002);
  // Test pick_offset_
  ASSERT_DOUBLE_EQ(ctest.pick_offset_, 0.003);
  // Test battery_grip_offset_
  ASSERT_DOUBLE_EQ(ctest.battery_grip_offset_, -0.05);
}

TEST(competitionARIAC_test, TestPartTypes) {
  // Test part_types_
  ASSERT_EQ(ctest.part_types_[ariac_msgs::msg::Part::BATTERY], "battery");
  ASSERT_EQ(ctest.part_types_[ariac_msgs::msg::Part::PUMP], "pump");
}

TEST(competitionARIAC_test, TestPartColors) {
  // Test part_colors_
  ASSERT_EQ(ctest.part_colors_[ariac_msgs::msg::Part::RED], "red");
  ASSERT_EQ(ctest.part_colors_[ariac_msgs::msg::Part::BLUE], "blue");
}

TEST(competitionARIAC_test, TestQuaternionFromRPY) {
  // Test with known values
  double roll = 0.1;
  double pitch = 0.2;
  double yaw = 0.3;

  geometry_msgs::msg::Quaternion result =
      ctest.QuaternionFromRPY(roll, pitch, yaw);

  // Convert the quaternion back to RPY for verification
  tf2::Quaternion expected_quaternion;
  expected_quaternion.setRPY(roll, pitch, yaw);

  // Verify that the returned quaternion matches the expected values
  ASSERT_NEAR(result.x, expected_quaternion.x(), 1e-5);
  ASSERT_NEAR(result.y, expected_quaternion.y(), 1e-5);
  ASSERT_NEAR(result.z, expected_quaternion.z(), 1e-5);
  ASSERT_NEAR(result.w, expected_quaternion.w(), 1e-5);
}

TEST(competitionARIAC_test, TestPartHeights) {
  // Test part_heights_
  ASSERT_DOUBLE_EQ(ctest.part_heights_[ariac_msgs::msg::Part::BATTERY], 0.04);
  ASSERT_DOUBLE_EQ(ctest.part_heights_[ariac_msgs::msg::Part::PUMP], 0.12);
}

TEST(competitionARIAC_test, TestQuadOffsets) {
  // Test quad_offsets_
  ASSERT_DOUBLE_EQ(
      ctest.quad_offsets_[ariac_msgs::msg::KittingPart::QUADRANT1].first,
      -0.08);
  ASSERT_DOUBLE_EQ(
      ctest.quad_offsets_[ariac_msgs::msg::KittingPart::QUADRANT1].second,
      0.12);
}

TEST(competitionARIAC_test, TestRailPositions) {
  // Test rail_positions_
  ASSERT_DOUBLE_EQ(ctest.rail_positions_["agv1"], -4.5);
  ASSERT_DOUBLE_EQ(ctest.rail_positions_["agv2"], -1.2);
}

TEST(competitionARIAC_test, TestFloorKts1Js) {
  // Test floor_kts1_js_
  ASSERT_DOUBLE_EQ(ctest.floor_kts1_js_["linear_actuator_joint"], 4.0);
  ASSERT_DOUBLE_EQ(ctest.floor_kts1_js_["floor_shoulder_pan_joint"], 1.57);
}

TEST(competitionARIAC_test, TestFloorKts2Js) {
  // Test floor_kts2_js_
  ASSERT_DOUBLE_EQ(ctest.floor_kts2_js_["linear_actuator_joint"], -4.0);
  ASSERT_DOUBLE_EQ(ctest.floor_kts2_js_["floor_shoulder_pan_joint"], -1.57);
}
