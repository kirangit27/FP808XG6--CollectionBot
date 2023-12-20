/**
 * @file ccs.hpp
 * @author Kiran S Patil (kpatil27@umd.edu)
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @author Surya Chappidi (chappidi@umd.edu)
 * @brief Manages interactions and functionalities in the Automated Robot
 * Industrial Automation Competition (ARIAC).
 * @version 0.1
 * @date 2023-12-6
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <shape_msgs/msg/mesh.h>
#include <tf2_kdl/tf2_kdl.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <kdl/frames.hpp>
#include <map>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>

#include "ariac_msgs/msg/assembly_task.hpp"
#include "ariac_msgs/msg/bin_info.hpp"
#include "ariac_msgs/msg/bin_parts.hpp"
#include "ariac_msgs/msg/break_beam_status.hpp"
#include "ariac_msgs/msg/competition_state.hpp"
#include "ariac_msgs/msg/conveyor_parts.hpp"
#include "ariac_msgs/msg/kitting_part.hpp"
#include "ariac_msgs/msg/kitting_task.hpp"
#include "ariac_msgs/msg/order.hpp"
#include "ariac_msgs/msg/part.hpp"
#include "ariac_msgs/msg/part_lot.hpp"
#include "ariac_msgs/srv/submit_order.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @class CompetitionARIAC
 * @brief This class manages the ARIAC competition workflow and robot
 * interactions.
 *
 * It subscribes to various topics to receive competition state updates, orders,
 * and sensor data. It also provides functionalities to control a robot for
 * tasks like picking, placing, and moving parts.
 */

class CompetitionARIAC : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new CompetitionARIAC object.
   *
   * Sets up ROS node, initializes MoveGroup for robot control, and subscribes
   * to necessary topics.
   */

  CompetitionARIAC()
      : Node("ccs_node"),
        floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)),
                     "floor_robot"),
        planning_scene_() {
    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);

    // Subscribe to topics
    rclcpp::SubscriptionOptions options;

    topic_cb_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    options.callback_group = topic_cb_group_;

    orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
        "/ariac/orders", 1,
        std::bind(&CompetitionARIAC::OrderCallback, this,
                  std::placeholders::_1),
        options);

    competition_state_sub_ =
        this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 1,
            std::bind(&CompetitionARIAC::CompetitionStateCallback, this,
                      std::placeholders::_1),
            options);

    kts1_camera_sub_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitionARIAC::KitTrayTable1Callback, this,
                      std::placeholders::_1),
            options);

    kts2_camera_sub_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitionARIAC::KitTrayTable2Callback, this,
                      std::placeholders::_1),
            options);

    left_bins_camera_sub_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitionARIAC::LeftBinsCameraCallback, this,
                      std::placeholders::_1),
            options);

    right_bins_camera_sub_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitionARIAC::RightBinsCameraCallback, this,
                      std::placeholders::_1),
            options);

    floor_gripper_state_sub_ =
        this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
            "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
            std::bind(&CompetitionARIAC::floor_gripper_state_cb, this,
                      std::placeholders::_1),
            options);

    // Initialize service clients
    quality_checker_ =
        this->create_client<ariac_msgs::srv::PerformQualityCheck>(
            "/ariac/perform_quality_check");
    pre_assembly_poses_getter_ =
        this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>(
            "/ariac/get_pre_assembly_poses");
    floor_robot_tool_changer_ =
        this->create_client<ariac_msgs::srv::ChangeGripper>(
            "/ariac/floor_robot_change_gripper");
    floor_robot_gripper_enable_ =
        this->create_client<ariac_msgs::srv::VacuumGripperControl>(
            "/ariac/floor_robot_enable_gripper");

    AddModelsToPlanningScene();

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
  }

  /**
   * @brief Destroy the CompetitionARIAC object.
   *
   * Ensures proper resource cleanup and node shutdown.
   */

  ~CompetitionARIAC() { floor_robot_.~MoveGroupInterface(); }

  // Floor Robot Public Functions
  /**
   * @brief Sends the robot to its home position.
   */
  void FloorRobotSendHome();

  /**
   * @brief Sets the state of the robot's gripper.
   *
   * @param enable A boolean value to enable or disable the gripper.
   * @return true If the gripper state was successfully set.
   * @return false If the gripper state could not be set.
   */
  bool FloorRobotSetGripperState(bool enable);

  /**
   * @brief Changes the gripper type of the robot.
   *
   * @param station The station where the gripper change is to be performed.
   * @param gripper_type The type of the gripper to be changed to.
   * @return true If the gripper was successfully changed.
   * @return false If the gripper change failed.
   */
  bool FloorRobotChangeGripper(std::string station, std::string gripper_type);

  /**
   * @brief Picks and places a tray on an AGV.
   *
   * This method controls the robot to pick a tray identified by tray_id
   * and place it on a specified AGV numbered agv_num.
   *
   * @param tray_id The identifier of the tray to be picked.
   * @param agv_num The number of the AGV where the tray is to be placed.
   * @return true If the tray is successfully picked and placed.
   * @return false If the operation fails.
   */
  bool FloorRobotPickandPlaceTray(int tray_id, int agv_num);

  /**
   * @brief Picks a part for kitting.
   *
   * This method is used to control the robot to pick a specified part for the
   * kitting process. The part to be picked is specified by the
   * order_::KittingPart structure.
   *
   * @param part_to_pick The KittingPart struct representing the part to be
   * picked.
   * @return true If the part is successfully picked.
   * @return false If the operation fails.
   */
  bool FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick);

  /**
   * @brief Places a part on the kitting tray.
   *
   * This method controls the robot to place a part on a specified quadrant of
   * the kitting tray located on an AGV numbered agv_num.
   *
   * @param agv_num The number of the AGV on which the kitting tray is located.
   * @param quadrant The quadrant of the kitting tray where the part is to be
   * placed.
   * @return true If the part is successfully placed.
   * @return false If the operation fails.
   */
  bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);

  /**
   * @brief Calls the service to start the competition.
   *
   * This function sends a request to the competition's start service.
   */
  bool StartCompetition();

  /**
   * @brief Calls the service to end the competition.
   *
   * This function sends a request to the competition's end service.
   */
  bool EndCompetition();

  /**
   * @brief Moves an AGV to a specified destination.
   *
   * This method sends a command to move an AGV, identified by agv_num, to a
   * specified destination.
   *
   * @param agv_num The number of the AGV to be moved.
   * @param destination The destination where the AGV is to be moved.
   * @return true If the AGV successfully moves to the destination.
   * @return false If the operation fails.
   */
  bool MoveAGV(int agv_num, int destination);

  /**
   * @brief Locks an AGV in its current position.
   *
   * This method is used to lock an AGV, identified by agv_num, preventing it
   * from moving.
   *
   * @param agv_num The number of the AGV to be locked.
   * @return true If the AGV is successfully locked.
   * @return false If the operation fails.
   */
  bool LockAGV(int agv_num);

  /**
   * @brief Calls the service to submit an order.
   *
   * @param order_id The order to be submitted.
   */

  bool SubmitOrder(std::string order_id);

  bool CompleteOrders();
  bool CompleteKittingTask(ariac_msgs::msg::KittingTask task);

 private:
  // Floor Robot Move Functions
  /**
   * @brief Moves the floor robot to a predefined target position.
   *
   * @return true If the robot successfully moves to the target.
   * @return false If the robot fails to reach the target.
   */
  bool FloorRobotMovetoTarget();

  /**
   * @brief Moves the floor robot along a specified Cartesian path.
   *
   * @param waypoints A vector of poses defining the path.
   * @param vsf Velocity scaling factor.
   * @param asf Acceleration scaling factor.
   * @return true If the movement is successful.
   * @return false If the movement fails.
   */
  bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints,
                               double vsf, double asf);

  /**
   * @brief Waits for a specified duration for the robot to attach to an object.
   *
   * @param timeout The maximum time to wait in seconds.
   */
  void FloorRobotWaitForAttach(double timeout);

  /**
   * @brief Sets the orientation of the robot based on a given rotation.
   *
   * @param rotation The desired rotation angle.
   * @return The new orientation as a quaternion.
   */
  geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

  /**
   * @brief Logs the pose of the robot.
   *
   * @param p The pose to be logged.
   */
  void LogPose(geometry_msgs::msg::Pose p);

  /**
   * @brief Multiplies two poses to calculate the resultant pose.
   *
   * @param p1 The first pose.
   * @param p2 The second pose.
   * @return The resultant pose.
   */
  geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1,
                                        geometry_msgs::msg::Pose p2);

  /**
   * @brief Builds a pose from specified x, y, z coordinates and an orientation.
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param z Z coordinate.
   * @param orientation The orientation as a quaternion.
   * @return The constructed pose.
   */
  geometry_msgs::msg::Pose BuildPose(
      double x, double y, double z, geometry_msgs::msg::Quaternion orientation);

  /**
   * @brief Retrieves the pose of a frame in the world coordinate system.
   *
   * @param frame_id The ID of the frame.
   * @return The pose of the frame.
   */
  geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);

  /**
   * @brief Gets the yaw (rotation around the z-axis) from a pose.
   *
   * @param pose The pose from which to extract the yaw.
   * @return The yaw value.
   */
  double GetYaw(geometry_msgs::msg::Pose pose);

  /**
   * @brief Gets the pitch (rotation around the y-axis) from a pose.
   *
   * @param pose The pose from which to extract the pitch.
   * @return The pitch value.
   */
  double GetPitch(geometry_msgs::msg::Pose pose);

  /**
   * @brief Creates a quaternion from roll, pitch, and yaw angles.
   *
   * @param r Roll angle.
   * @param p Pitch angle.
   * @param y Yaw angle.
   * @return The resulting quaternion.
   */
  geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p,
                                                   double y);

  /**
   * @brief Adds a model to the planning scene.
   *
   * @param name The name of the model.
   * @param mesh_file The file path of the mesh representing the model.
   * @param model_pose The pose of the model in the planning scene.
   */
  void AddModelToPlanningScene(std::string name, std::string mesh_file,
                               geometry_msgs::msg::Pose model_pose);

  /**
   * @brief Adds multiple models to the planning scene.
   *
   * This function is typically used to set up the initial state of the planning
   * scene with all required models.
   */
  void AddModelsToPlanningScene();

  // AGV location
  std::map<int, int> agv_locations_ = {{1, -1}, {2, -1}, {3, -1}, {4, -1}};

  // Callback Groups
  /**
   * @brief Shared pointer for the first callback group.
   *
   * Manages callbacks for ROS subscriptions and services.
   */
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  /**
   * @brief Shared pointer for the second callback group.
   */
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

  /**
   * @brief Interface to the MoveGroup for robot movement planning.
   */
  moveit::planning_interface::MoveGroupInterface floor_robot_;

  /**
   * @brief Interface to the planning scene for managing the robot's
   * understanding of the environment.
   */
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  /**
   * @brief Time-optimal trajectory generation for robot movement planning.
   */
  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

  // TF
  /**
   * @brief Unique pointer to a tf2_ros::Buffer.
   *
   * Used for managing transformations in the robot's coordinate frames.
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_unique<tf2_ros::Buffer>(get_clock());

  /**
   * @brief Shared pointer to a tf2_ros::TransformListener.
   *
   * Listens for transformations published to the tf2 buffer.
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // ROS Subscriptions
  /**
   * @brief Subscription to the competition state topic.
   *
   * Receives updates about the state of the ARIAC competition.
   */
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr
      competition_state_sub_;

  /**
   * @brief Subscription to the orders topic.
   *
   * Receives new orders for the ARIAC competition.
   */
  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr orders_sub_;

  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv1_status_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv2_status_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv3_status_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv4_status_sub_;

  /*!< Subscriber to camera image over kit tray table1. */
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      kts1_camera_sub_;
  /*!< Subscriber to camera image over kit tray table2. */
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      kts2_camera_sub_;
  /*!< Subscriber to camera image over left bins. */
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      left_bins_camera_sub_;
  /*!< Subscriber to camera image over right bins. */
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      right_bins_camera_sub_;
  /*!< Subscriber to floor gripper state */
  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr
      floor_gripper_state_sub_;

  // Orders List
  ariac_msgs::msg::Order current_order_;
  std::vector<ariac_msgs::msg::Order> orders_;

  unsigned int competition_state_;

  // Gripper State
  /**
   * @brief Current state of the vacuum gripper.
   */
  ariac_msgs::msg::VacuumGripperState floor_gripper_state_;

  /**
   * @brief Information about the kitting part currently attached to the floor
   * robot.
   */
  ariac_msgs::msg::Part floor_robot_attached_part_;

  /**
   * @brief Pose of the camera observing kit tray table 1.
   */
  geometry_msgs::msg::Pose kts1_camera_pose_;

  /**
   * @brief Pose of the camera observing kit tray table 2.
   */
  geometry_msgs::msg::Pose kts2_camera_pose_;

  /**
   * @brief Pose of the camera observing the left bins.
   */
  geometry_msgs::msg::Pose left_bins_camera_pose_;

  /**
   * @brief Pose of the camera observing the right bins.
   */
  geometry_msgs::msg::Pose right_bins_camera_pose_;

  /**
   * @brief List of kit tray poses in table 1.
   */
  std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;

  /**
   * @brief List of kit tray poses in table 2.
   */
  std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

  /**
   * @brief List of parts in the left bins, as detected by sensors.
   */
  std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;

  /**
   * @brief List of parts in the right bins, as detected by sensors.
   */
  std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;

  // Sensor Callbacks
  bool kts1_camera_recieved_data = false;
  bool kts2_camera_recieved_data = false;
  bool left_bins_camera_recieved_data = false;
  bool right_bins_camera_recieved_data = false;

  /**
   * @brief Callback function for the kit tray table 1 camera.
   *
   * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
   */
  void KitTrayTable1Callback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

  /**
   * @brief Callback function for the kit tray table 2 camera.
   *
   * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
   */
  void KitTrayTable2Callback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

  /**
   * @brief Callback function for the left bins camera.
   *
   * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
   */
  void LeftBinsCameraCallback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

  /**
   * @brief Callback function for the right bins camera.
   *
   * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
   */
  void RightBinsCameraCallback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

  /**
   * @brief Callback function for the state of the floor gripper.
   *
   * @param msg Shared pointer to the VacuumGripperState message.
   */
  void floor_gripper_state_cb(
      const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

  /**
   * @brief Callback function for receiving new orders.
   *
   * @param order_msg Shared pointer to the Order message.
   */
  void OrderCallback(const ariac_msgs::msg::Order::ConstSharedPtr msg);

  /**
   * @brief Callback function for competition state updates.
   *
   * @param msg Shared pointer to the CompetitionState message.
   */
  void CompetitionStateCallback(
      const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

  /**
   * @brief Client for performing quality checks on parts in the ARIAC
   * competition.
   *
   * This client sends requests to perform quality checks on assembled parts.
   */
  rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr
      quality_checker_;

  rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr
      pre_assembly_poses_getter_;

  /**
   * @brief Client for changing the gripper tool of the floor robot.
   *
   * This client communicates with the service that controls the changing of the
   * robot's gripper tool.
   */
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr
      floor_robot_tool_changer_;

  /**
   * @brief Client for controlling the vacuum gripper of the floor robot.
   *
   * This client enables or disables the vacuum gripper on the floor robot.
   */
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr
      floor_robot_gripper_enable_;
};

/**
 * @class Parts
 * @brief Class to manage part properties and configurations in the ARIAC
 * competition.
 *
 * This class includes constants and mappings related to parts used in the ARIAC
 * competition. It contains details like part types, colors, heights, and
 * specific configurations for handling parts with the robotic system.
 */
class Parts {
 public:
  /**
   * @brief Constants for part manipulation.
   */
  double kit_tray_thickness_ = 0.01;    ///< Thickness of the kit tray.
  double drop_height_ = 0.002;          ///< Height to drop parts onto the tray.
  double pick_offset_ = 0.003;          ///< Offset for picking parts.
  double battery_grip_offset_ = -0.05;  ///< Offset for gripping batteries.

  /**
   * @brief Map of part types.
   */
  std::map<int, std::string> part_types_ = {
      {ariac_msgs::msg::Part::BATTERY, "battery"},
      {ariac_msgs::msg::Part::PUMP, "pump"},
      {ariac_msgs::msg::Part::REGULATOR, "regulator"},
      {ariac_msgs::msg::Part::SENSOR, "sensor"}};

  /**
   * @brief Map of part colors.
   */
  std::map<int, std::string> part_colors_ = {
      {ariac_msgs::msg::Part::RED, "red"},
      {ariac_msgs::msg::Part::BLUE, "blue"},
      {ariac_msgs::msg::Part::GREEN, "green"},
      {ariac_msgs::msg::Part::ORANGE, "orange"},
      {ariac_msgs::msg::Part::PURPLE, "purple"}};

  /**
   * @brief Map of part heights.
   */
  std::map<int, double> part_heights_ = {
      {ariac_msgs::msg::Part::BATTERY, 0.04},
      {ariac_msgs::msg::Part::PUMP, 0.12},
      {ariac_msgs::msg::Part::REGULATOR, 0.07},
      {ariac_msgs::msg::Part::SENSOR, 0.07}};

  /**
   * @brief Map of quadrant offsets for part placement.
   */
  std::map<int, std::pair<double, double>> quad_offsets_ = {
      {ariac_msgs::msg::KittingPart::QUADRANT1,
       std::pair<double, double>(-0.08, 0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT2,
       std::pair<double, double>(0.08, 0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT3,
       std::pair<double, double>(-0.08, -0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT4,
       std::pair<double, double>(0.08, -0.12)}};

  /**
   * @brief Map of rail positions for AGVs and bins.
   */
  std::map<std::string, double> rail_positions_ = {
      {"agv1", -4.5}, {"agv2", -1.2},   {"agv3", 1.2},
      {"agv4", 4.5},  {"left_bins", 3}, {"right_bins", -3}};

  /**
   * @brief Joint value targets for kitting station 1.
   */
  std::map<std::string, double> floor_kts1_js_ = {
      {"linear_actuator_joint", 4.0},       {"floor_shoulder_pan_joint", 1.57},
      {"floor_shoulder_lift_joint", -1.57}, {"floor_elbow_joint", 1.57},
      {"floor_wrist_1_joint", -1.57},       {"floor_wrist_2_joint", -1.57},
      {"floor_wrist_3_joint", 0.0}};

  /**
   * @brief Joint value targets for kitting station 2.
   */
  std::map<std::string, double> floor_kts2_js_ = {
      {"linear_actuator_joint", -4.0},      {"floor_shoulder_pan_joint", -1.57},
      {"floor_shoulder_lift_joint", -1.57}, {"floor_elbow_joint", 1.57},
      {"floor_wrist_1_joint", -1.57},       {"floor_wrist_2_joint", -1.57},
      {"floor_wrist_3_joint", 0.0}};
};
