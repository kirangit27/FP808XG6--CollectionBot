/**
 * @file ccs.cpp
 * @author Kiran S Patil (kpatil27@umd.edu)
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @author Surya Chappidi (chappidi@umd.edu)
 * @brief Implementation file for managing interactions and functionalities in
 * the Automated Robot Industrial Automation Competition (ARIAC).
 * @version 0.1
 * @date 2023-12-6
 *
 * This file contains the implementation of the CompetitionARIAC class methods.
 * It includes functionalities for handling competition states, processing
 * orders, and controlling robots for various tasks in the ARIAC.
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/ariac_collection_bot/ccs.hpp"

/**
 * @brief Callback for processing new orders.
 *
 * This function is called whenever a new order is received.
 * It stores the order information in the orders_ member.
 *
 * @param msg The order message received.
 */
void CompetitionARIAC::OrderCallback(
    const ariac_msgs::msg::Order::ConstSharedPtr msg) {
  orders_.push_back(*msg);
}

/**
 * @brief Callback for updating competition state.
 *
 * This function updates the current state of the competition.
 *
 * @param msg The competition state message received.
 */
void CompetitionARIAC::CompetitionStateCallback(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) {
  competition_state_ = msg->competition_state;
}

/**
 * @brief Callback for Kit Tray Table 1 camera data.
 *
 * This function processes camera data from Kit Tray Table 1.
 * It updates the tray and camera poses.
 *
 * @param msg The camera image message received.
 */
void CompetitionARIAC::KitTrayTable1Callback(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) {
  if (!kts1_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
    kts1_camera_recieved_data = true;
  }

  kts1_trays_ = msg->tray_poses;
  kts1_camera_pose_ = msg->sensor_pose;
}

/**
 * @brief Callback for Kit Tray Table 2 camera data.
 *
 * This function processes camera data from Kit Tray Table 2.
 * It updates the tray and camera poses.
 *
 * @param msg The camera image message received.
 */
void CompetitionARIAC::KitTrayTable2Callback(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) {
  if (!kts2_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
    kts2_camera_recieved_data = true;
  }

  kts2_trays_ = msg->tray_poses;
  kts2_camera_pose_ = msg->sensor_pose;
}

/**
 * @brief Callback for left bins camera data.
 *
 * This function processes camera data from left bins.
 * It updates the parts and camera poses.
 *
 * @param msg The camera image message received.
 */
void CompetitionARIAC::LeftBinsCameraCallback(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) {
  if (!left_bins_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from left bins camera");
    left_bins_camera_recieved_data = true;
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

/**
 * @brief Callback for right bins camera data.
 *
 * This function processes camera data from right bins.
 * It updates the parts and camera poses.
 *
 * @param msg The camera image message received.
 */
void CompetitionARIAC::RightBinsCameraCallback(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) {
  if (!right_bins_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from right bins camera");
    right_bins_camera_recieved_data = true;
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}

/**
 * @brief Callback for the state of the floor gripper.
 *
 * This function updates the state of the floor gripper.
 *
 * @param msg The vacuum gripper state message received.
 */
void CompetitionARIAC::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) {
  floor_gripper_state_ = *msg;
}

/**
 * @brief Multiplies two geometry poses.
 *
 * This function multiplies two poses and returns the result.
 *
 * @param p1 The first pose.
 * @param p2 The second pose.
 * @return The resulting pose after multiplication.
 */
geometry_msgs::msg::Pose CompetitionARIAC::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2) {
  KDL::Frame f1;
  KDL::Frame f2;

  tf2::fromMsg(p1, f1);
  tf2::fromMsg(p2, f2);

  KDL::Frame f3 = f1 * f2;

  return tf2::toMsg(f3);
}

/**
 * @brief Logs a pose's position and orientation in degrees.
 *
 * This function logs the X, Y, Z coordinates and the Roll, Pitch, Yaw
 * orientation in degrees.
 *
 * @param p The pose to log.
 */
void CompetitionARIAC::LogPose(geometry_msgs::msg::Pose p) {
  tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z,
                    p.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  roll *= 180 / M_PI;
  pitch *= 180 / M_PI;
  yaw *= 180 / M_PI;

  RCLCPP_INFO(get_logger(),
              "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
              p.position.x, p.position.y, p.position.z, roll, pitch, yaw);
}

/**
 * @brief Builds a pose from position coordinates and orientation.
 *
 * This function creates a geometry_msgs::msg::Pose object from given x, y, z
 * coordinates and a Quaternion orientation.
 *
 * @param x X coordinate of the position.
 * @param y Y coordinate of the position.
 * @param z Z coordinate of the position.
 * @param orientation The Quaternion representing the orientation.
 * @return A Pose object with the specified position and orientation.
 */
geometry_msgs::msg::Pose CompetitionARIAC::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;

  return pose;
}

/**
 * @brief Gets the pose of a frame relative to the world frame.
 *
 * This function returns the pose of the specified frame_id in the world
 * coordinate frame. It handles exceptions if the transform cannot be obtained.
 *
 * @param frame_id The ID of the frame for which the pose is requested.
 * @return The Pose of the frame in world coordinates.
 */
geometry_msgs::msg::Pose CompetitionARIAC::FrameWorldPose(
    std::string frame_id) {
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "Could not get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

/**
 * @brief Retrieves the yaw from a pose.
 *
 * This function extracts the yaw (rotation about the Z-axis) from the
 * orientation of the given pose.
 *
 * @param pose The Pose from which to extract the yaw.
 * @return The yaw value in radians.
 */
double CompetitionARIAC::GetYaw(geometry_msgs::msg::Pose pose) {
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

/**
 * @brief Creates a Quaternion from roll, pitch, and yaw angles.
 *
 * This function generates a Quaternion representing an orientation from
 * given roll, pitch, and yaw values.
 *
 * @param r Roll angle in radians.
 * @param p Pitch angle in radians.
 * @param y Yaw angle in radians.
 * @return A Quaternion representing the specified orientation.
 */
geometry_msgs::msg::Quaternion CompetitionARIAC::QuaternionFromRPY(double r,
                                                                   double p,
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

/**
 * @brief An instance of the Parts class.
 *
 * This object 'p' is used to access part properties and configurations such as
 * part types, colors, heights, and specific robotic handling configurations for
 * the ARIAC competition.
 */
Parts p;

/**
 * @brief Adds a model to the planning scene.
 *
 * This function adds a specified model to the planning scene using a mesh file.
 * It sets the model's ID, frame ID, mesh file path, and pose before adding it
 * to the collision objects in the planning scene.
 *
 * @param name Name of the model to be added.
 * @param mesh_file Path to the mesh file representing the model.
 * @param model_pose Pose of the model in the planning scene.
 */
void CompetitionARIAC::AddModelToPlanningScene(
    std::string name, std::string mesh_file,
    geometry_msgs::msg::Pose model_pose) {
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("test_competitor");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_.addCollisionObjects(collision_objects);
}

/**
 * @brief Adds various models to the planning scene.
 *
 * This function adds predefined models like bins, assembly stations, and
 * conveyor to the planning scene. It defines their positions and orientations,
 * and uses AddModelToPlanningScene for adding each.
 */
void CompetitionARIAC::AddModelsToPlanningScene() {
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {
      {"bin1", std::pair<double, double>(-1.9, 3.375)},
      {"bin2", std::pair<double, double>(-1.9, 2.625)},
      {"bin3", std::pair<double, double>(-2.65, 2.625)},
      {"bin4", std::pair<double, double>(-2.65, 3.375)},
      {"bin5", std::pair<double, double>(-1.9, -3.375)},
      {"bin6", std::pair<double, double>(-1.9, -2.625)},
      {"bin7", std::pair<double, double>(-2.65, -2.625)},
      {"bin8", std::pair<double, double>(-2.65, -3.375)}};

  geometry_msgs::msg::Pose bin_pose;
  for (auto const &bin : bin_positions) {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions =
      {
          {"as1", std::pair<double, double>(-7.3, 3)},
          {"as2", std::pair<double, double>(-12.3, 3)},
          {"as3", std::pair<double, double>(-7.3, -3)},
          {"as4", std::pair<double, double>(-12.3, -3)},
      };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const &station : assembly_station_positions) {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene(station.first, "assembly_station.stl",
                            assembly_station_pose);
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
      {"as1_insert", std::pair<double, double>(-7.7, 3)},
      {"as2_insert", std::pair<double, double>(-12.7, 3)},
      {"as3_insert", std::pair<double, double>(-7.7, -3)},
      {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;
  for (auto const &insert : assembly_insert_positions) {
    assembly_insert_pose.position.x = insert.second.first;
    assembly_insert_pose.position.y = insert.second.second;
    assembly_insert_pose.position.z = 1.011;
    assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene(insert.first, "assembly_insert.stl",
                            assembly_insert_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose;
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

  AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

  AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

  AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

/**
 * @brief Sets the orientation of the robot.
 *
 * This function creates a Quaternion representing an orientation from a given
 * rotation value.
 *
 * @param rotation Rotation angle in radians.
 * @return A Quaternion representing the specified orientation.
 */
geometry_msgs::msg::Quaternion CompetitionARIAC::SetRobotOrientation(
    double rotation) {
  tf2::Quaternion tf_q;
  tf_q.setRPY(0, 3.14159, rotation);

  geometry_msgs::msg::Quaternion q;

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q;
}

/**
 * @brief Moves the floor robot to a target position.
 *
 * This function plans and executes a movement of the floor robot to a
 * predefined target. It checks for the success of the planning phase before
 * execution.
 *
 * @return True if the movement was successful, false otherwise.
 */
bool CompetitionARIAC::FloorRobotMovetoTarget() {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(floor_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

/**
 * @brief Executes a Cartesian movement of the floor robot.
 *
 * This function plans and executes a Cartesian path for the floor robot based
 * on specified waypoints. It also uses a trajectory re-timing algorithm to
 * control the velocity and acceleration scaling factors.
 *
 * @param waypoints A vector of poses defining the Cartesian path.
 * @param vsf Velocity scaling factor.
 * @param asf Acceleration scaling factor.
 * @return True if the trajectory was successfully executed, false otherwise.
 */
bool CompetitionARIAC::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf) {
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction =
      floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(
      floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_.execute(trajectory));
}

/**
 * @brief Waits for the floor robot's gripper to attach to an object.
 *
 * This function continuously checks if the gripper has successfully attached to
 * an object within a specified timeout.
 *
 * @param timeout Maximum time to wait for the gripper to attach.
 */
void CompetitionARIAC::FloorRobotWaitForAttach(double timeout) {
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)) {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

/**
 * @brief Sends the floor robot to its home position.
 *
 * This function moves the floor robot to a predefined home position.
 */
void CompetitionARIAC::FloorRobotSendHome() {
  // Move floor robot to home joint state
  floor_robot_.setNamedTarget("home");
  FloorRobotMovetoTarget();
}

/**
 * @brief Sets the state of the floor robot's gripper.
 *
 * This function enables or disables the floor robot's gripper based on the
 * given state.
 *
 * @param enable True to enable the gripper, false to disable it.
 * @return True if the operation was successful, false otherwise.
 */
bool CompetitionARIAC::FloorRobotSetGripperState(bool enable) {
  if (floor_gripper_state_.enabled == enable) {
    if (floor_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else
      RCLCPP_INFO(get_logger(), "Already disabled");

    return false;
  }

  // Call enable service
  auto request =
      std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = floor_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

/**
 * @brief Changes the gripper type of the floor robot.
 *
 * This function changes the gripper type of the floor robot by moving to a tool
 * changer and calling a service to perform the change.
 *
 * @param station Tool changer station ID.
 * @param gripper_type Type of the gripper to be changed to.
 * @return True if the gripper change was successful, false otherwise.
 */
bool CompetitionARIAC::FloorRobotChangeGripper(std::string station,
                                               std::string gripper_type) {
  // Move gripper into tool changer
  auto tc_pose =
      FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z + 0.4,
                                SetRobotOrientation(0.0)));

  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z, SetRobotOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

  if (gripper_type == "trays") {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  } else if (gripper_type == "parts") {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto result = floor_robot_tool_changer_->async_send_request(request);
  result.wait();
  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z + 0.4,
                                SetRobotOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) return false;

  return true;
}

/**
 * @brief Picks up a tray and places it on an AGV.
 *
 * This function locates a tray by its ID, picks it up, and places it onto a
 * specified AGV. It includes steps for finding the tray, moving to it,
 * adjusting the gripper, and handling the tray throughout the process.
 *
 * @param tray_id The ID of the tray to be picked up.
 * @param agv_num The number of the AGV where the tray is to be placed.
 * @return True if the pick and place operation was successful, false otherwise.
 */
bool CompetitionARIAC::FloorRobotPickandPlaceTray(int tray_id, int agv_num) {
  // Check if kit tray is on one of the two tables
  geometry_msgs::msg::Pose tray_pose;
  std::string station;
  bool found_tray = false;

  // Check table 1
  for (auto tray : kts1_trays_) {
    if (tray.id == tray_id) {
      station = "kts1";
      tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
      found_tray = true;
      break;
    }
  }
  // Check table 2
  if (!found_tray) {
    for (auto tray : kts2_trays_) {
      if (tray.id == tray_id) {
        station = "kts2";
        tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
        found_tray = true;
        break;
      }
    }
  }
  if (!found_tray) return false;

  double tray_rotation = GetYaw(tray_pose);

  // Move floor robot to the corresponding kit tray table
  if (station == "kts1") {
    floor_robot_.setJointValueTarget(p.floor_kts1_js_);
  } else {
    floor_robot_.setJointValueTarget(p.floor_kts2_js_);
  }
  FloorRobotMovetoTarget();

  // Change gripper to tray gripper
  if (floor_gripper_state_.type != "tray_gripper") {
    FloorRobotChangeGripper(station, "trays");
  }

  // Move to tray
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2,
                                SetRobotOrientation(tray_rotation)));
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + p.pick_offset_,
                                SetRobotOrientation(tray_rotation)));
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  FloorRobotSetGripperState(true);

  FloorRobotWaitForAttach(3.0);

  // Add kit tray to planning scene
  std::string tray_name = "kit_tray_" + std::to_string(tray_id);
  AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
  floor_robot_.attachObject(tray_name);

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2,
                                SetRobotOrientation(tray_rotation)));
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      p.rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

  FloorRobotMovetoTarget();

  auto agv_tray_pose =
      FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
  auto agv_rotation = GetYaw(agv_tray_pose);

  waypoints.clear();
  waypoints.push_back(BuildPose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

  waypoints.push_back(BuildPose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + p.kit_tray_thickness_ + p.drop_height_,
      SetRobotOrientation(agv_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  FloorRobotSetGripperState(false);

  floor_robot_.detachObject(tray_name);

  LockAGV(agv_num);

  waypoints.clear();
  waypoints.push_back(
      BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  return true;
}

/**
 * @brief Picks a part from a bin and places it in the robot's gripper.
 *
 * This function identifies and picks a specified part from one of the bins. It
 * includes searching for the part, moving the robot to the part's location,
 * changing the gripper if necessary, and attaching the part to the gripper.
 *
 * @param part_to_pick The part to be picked, specified by its type and color.
 * @return True if the part was successfully picked and attached to the gripper,
 * false otherwise.
 */
bool CompetitionARIAC::FloorRobotPickBinPart(
    ariac_msgs::msg::Part part_to_pick) {
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a "
                                       << p.part_colors_[part_to_pick.color]
                                       << " "
                                       << p.part_types_[part_to_pick.type]);

  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose;
  bool found_part = false;
  std::string bin_side;

  // Check left bins
  for (auto part : left_bins_parts_) {
    if (part.part.type == part_to_pick.type &&
        part.part.color == part_to_pick.color) {
      part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
      found_part = true;
      bin_side = "left_bins";
      break;
    }
  }
  // Check right bins
  if (!found_part) {
    for (auto part : right_bins_parts_) {
      if (part.part.type == part_to_pick.type &&
          part.part.color == part_to_pick.color) {
        part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "right_bins";
        break;
      }
    }
  }
  if (!found_part) {
    RCLCPP_ERROR(get_logger(), "Unable to locate part");
    return false;
  }

  double part_rotation = GetYaw(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper") {
    std::string station;
    if (part_pose.position.y < 0) {
      station = "kts1";
    } else {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1") {
      floor_robot_.setJointValueTarget(p.floor_kts1_js_);
    } else {
      floor_robot_.setJointValueTarget(p.floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    FloorRobotChangeGripper(station, "parts");
  }

  floor_robot_.setJointValueTarget("linear_actuator_joint",
                                   p.rail_positions_[bin_side]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.5,
                                SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z +
                                    p.part_heights_[part_to_pick.type] +
                                    p.pick_offset_,
                                SetRobotOrientation(part_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  FloorRobotSetGripperState(true);

  FloorRobotWaitForAttach(3.0);

  // Add part to planning scene
  std::string part_name = p.part_colors_[part_to_pick.color] + "_" +
                          p.part_types_[part_to_pick.type];
  AddModelToPlanningScene(part_name, p.part_types_[part_to_pick.type] + ".stl",
                          part_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + 0.3,
                                SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  return true;
}

/**
 * @brief Picks a part from a bin and places it in the robot's gripper.
 *
 * This function identifies and picks a specified part from one of the bins. It
 * includes searching for the part, moving the robot to the part's location,
 * changing the gripper if necessary, and attaching the part to the gripper.
 *
 * @param part_to_pick The part to be picked, specified by its type and color.
 * @return True if the part was successfully picked and attached to the gripper,
 * false otherwise.
 */
bool CompetitionARIAC::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant) {
  if (!floor_gripper_state_.attached) {
    RCLCPP_ERROR(get_logger(), "No part attached");
    return false;
  }

  // Move to agv
  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      p.rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose =
      FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = BuildPose(p.quad_offsets_[quadrant].first,
                                    p.quad_offsets_[quadrant].second, 0.0,
                                    geometry_msgs::msg::Quaternion());

  auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(
      BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

  waypoints.push_back(BuildPose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z +
          p.part_heights_[floor_robot_attached_part_.type] + p.drop_height_,
      SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  // Drop part in quadrant
  FloorRobotSetGripperState(false);

  std::string part_name = p.part_colors_[floor_robot_attached_part_.color] +
                          "_" + p.part_types_[floor_robot_attached_part_.type];
  floor_robot_.detachObject(part_name);

  waypoints.clear();
  waypoints.push_back(
      BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  return true;
}

/**
 * @brief Completes all the orders in the competition.
 *
 * This function processes and completes all the orders received during the
 * competition. It continues until all orders are completed or the competition
 * ends. It handles kitting tasks and AGV movements.
 *
 * @return True if all orders were successfully completed, false if the
 * competition ended prematurely.
 */
bool CompetitionARIAC::CompleteOrders() {
  while (orders_.size() == 0) {
  }

  bool success;
  while (true) {
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
      success = false;
      break;
    }

    if (orders_.size() == 0) {
      if (competition_state_ !=
          ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
        // wait for more orders
        RCLCPP_INFO(get_logger(), "Waiting for orders...");
        while (orders_.size() == 0) {
        }
      } else {
        RCLCPP_INFO(get_logger(), "Completed all orders");
        success = true;
        break;
      }
    }
    current_order_ = orders_.front();
    orders_.erase(orders_.begin());
    int kitting_agv_num = -1;

    if (current_order_.type == ariac_msgs::msg::Order::KITTING) {
      CompetitionARIAC::CompleteKittingTask(current_order_.kitting_task);
      kitting_agv_num = current_order_.kitting_task.agv_number;
    }
    // loop until the AGV is at the warehouse
    auto agv_location = -1;
    while (agv_location != ariac_msgs::msg::AGVStatus::WAREHOUSE) {
      if (kitting_agv_num == 1)
        agv_location = agv_locations_[1];
      else if (kitting_agv_num == 2)
        agv_location = agv_locations_[2];
      else if (kitting_agv_num == 3)
        agv_location = agv_locations_[3];
      else if (kitting_agv_num == 4)
        agv_location = agv_locations_[4];
    }

    CompetitionARIAC::SubmitOrder(current_order_.id);
  }
  return success;
}

/**
 * @brief Completes a kitting task.
 *
 * This function performs a kitting task by picking and placing a tray and
 * handling the parts for the kit. It includes picking parts from bins, placing
 * them on the kit tray, and performing quality checks.
 *
 * @param task The kitting task to be completed.
 * @return True if the kitting task is completed successfully.
 */
bool CompetitionARIAC::CompleteKittingTask(ariac_msgs::msg::KittingTask task) {
  FloorRobotSendHome();

  FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

  for (auto kit_part : task.parts) {
    FloorRobotPickBinPart(kit_part.part);
    FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
  }

  // Check quality
  auto request =
      std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = current_order_.id;
  auto result = quality_checker_->async_send_request(request);
  result.wait();

  if (!result.get()->all_passed) {
    RCLCPP_ERROR(get_logger(), "Issue with shipment");
  }

  MoveAGV(task.agv_number, task.destination);

  return true;
}

/**
 * @brief Starts the competition.
 *
 * This function starts the competition by sending a request to the competition
 * start service. It waits until the competition state is ready before sending
 * the request.
 *
 * @return True if the competition starts successfully, false otherwise.
 */
bool CompetitionARIAC::StartCompetition() {
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY) {
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

/**
 * @brief Ends the competition.
 *
 * This function ends the competition by sending a request to the competition
 * end service.
 *
 * @return True if the competition ends successfully, false otherwise.
 */
bool CompetitionARIAC::EndCompetition() {
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

/**
 * @brief Submits an order.
 *
 * This function submits an order by sending a request to the order submission
 * service with the order ID.
 *
 * @param order_id The ID of the order to submit.
 * @return True if the order is submitted successfully, false otherwise.
 */
bool CompetitionARIAC::SubmitOrder(std::string order_id) {
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

/**
 * @brief Moves an AGV to a specified destination.
 *
 * This function moves an AGV to a given destination by sending a request to the
 * AGV movement service.
 *
 * @param agv_num The number of the AGV to move.
 * @param destination The destination to move the AGV to.
 * @return True if the AGV is moved successfully, false otherwise.
 */
bool CompetitionARIAC::MoveAGV(int agv_num, int destination) {
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

  client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

/**
 * @brief Locks the tray on an AGV.
 *
 * This function locks the tray on an AGV by sending a request to the AGV tray
 * lock service.
 *
 * @param agv_num The number of the AGV whose tray is to be locked.
 * @return True if the tray is locked successfully, false otherwise.
 */
bool CompetitionARIAC::LockAGV(int agv_num) {
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}
