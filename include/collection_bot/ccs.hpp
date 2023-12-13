/**
 * @file ccs.hpp
 * @author Kiran S Patil (kpatil27@umd.edu)
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @author Surya Chappidi (chappidi@umd.edu)
 * @brief Manages interactions and functionalities in the Automated Robot Industrial Automation Competition (ARIAC).
 * @version 0.1
 * @date 2023-12-6
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <chrono>
#include <memory>
#include <thread>
#include <map>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ariac_msgs/msg/competition_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string>

#include "ariac_msgs/msg/order.hpp"
#include "ariac_msgs/msg/assembly_task.hpp"
#include "ariac_msgs/msg/kitting_task.hpp"
#include "ariac_msgs/msg/kitting_part.hpp"
#include "ariac_msgs/msg/bin_parts.hpp"
#include "ariac_msgs/msg/conveyor_parts.hpp"
#include "ariac_msgs/msg/bin_info.hpp"
#include "ariac_msgs/msg/part_lot.hpp"
#include "ariac_msgs/msg/part.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "ariac_msgs/msg/break_beam_status.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
#include <ariac_msgs/msg/agv_status.hpp>

#include "ariac_msgs/srv/submit_order.hpp"
#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <unistd.h>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace order_
{   
    struct KittingPart{
    uint8_t quadrant;
    uint8_t color;
    uint8_t type;
    bool part_status{0};
    };

    struct KittingType{
        uint8_t quadrant;
        uint8_t agv_number;
        int tray_id;
        uint8_t destination;
        std::vector<KittingPart> parts;
    };

    class Orders{
    public:
        std::string id;
        uint8_t type;
        bool priority;
        KittingType kitting_type;
        bool order_status{0};

        std::map<int,std::string> order_type={
            {0,"KITTING"},
            {1,"ASSEMBLY"},
            {2,"COMBINED"}};
    };
}

namespace color_
{   
    class ColorParts{
        
        public:

        std::map<int, std::string> COLOR = {
            {ariac_msgs::msg::Part::RED, "red"},
            {ariac_msgs::msg::Part::GREEN, "green"},
            {ariac_msgs::msg::Part::BLUE, "blue"},
            {ariac_msgs::msg::Part::ORANGE, "orange"},
            {ariac_msgs::msg::Part::PURPLE, "purple"}};

        std::map<int,std::string> PART={
            {ariac_msgs::msg::Part::BATTERY,"battery"},
            {ariac_msgs::msg::Part::PUMP,"pump"},
            {ariac_msgs::msg::Part::SENSOR,"sensor"},
            {ariac_msgs::msg::Part::REGULATOR,"regulator"}};

        std::map<int,std::string> DEST={
            {ariac_msgs::msg::KittingTask::KITTING, "KITTING"},
            {ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT, "ASSEMBLY_FRONT"},
            {ariac_msgs::msg::KittingTask::ASSEMBLY_BACK, "ASSEMBLY_BACK"},
            {ariac_msgs::msg::KittingTask::WAREHOUSE, "WAREHOUSE"}};
            
    };
    
}

namespace pick_part 
{   

    struct Part{
        uint8_t color;
        uint8_t type;
        geometry_msgs::msg::Pose pose;
    };

    struct PartInfo{
        uint8_t quantity;
        std::string pickup_type;
        uint8_t color;
        uint8_t type;
        uint8_t bin_number;
        geometry_msgs::msg::Pose pose;
    };

    class Parts {
        public:
        std::vector<PartInfo> parts;
    };
}

namespace constants
{   /**
    * @brief Class that stores constants
    * 
    */
    class Constants{

        public:

            double kit_tray_thickness_ = 0.01;        
            double drop_height_ = 0.002;
            double pick_offset_ = 0.007;
            double pick_offset_2 = 0.008;
            double battery_grip_offset_ = -0.05;

            // Part heights
            std::map<int, double> part_heights_ = {
                {ariac_msgs::msg::Part::BATTERY, 0.04},
                {ariac_msgs::msg::Part::PUMP, 0.12},
                {ariac_msgs::msg::Part::REGULATOR, 0.07},
                {ariac_msgs::msg::Part::SENSOR, 0.07}};

            // Part heights
            std::map<int, double> part_heights_kit_pick = {
                {ariac_msgs::msg::Part::BATTERY, 0.0},
                {ariac_msgs::msg::Part::PUMP, 0.08},
                {ariac_msgs::msg::Part::REGULATOR, 0.02},
                {ariac_msgs::msg::Part::SENSOR, 0.02}};

            // Quadrant Offsets
            std::map<int, std::pair<double, double>> quad_offsets_ = {
                {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
                {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
                {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
                {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)}};

            std::map<std::string, double> rail_positions_ = {
                {"agv1", -4.5},
                {"agv2", -1.2},
                {"agv3", 1.2},
                {"agv4", 4.5},
                {"left_bins", 3},
                {"right_bins", -3}};

            std::map<int, std::pair<int,int>> quad_parts_map = {
                {1, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::BATTERY)},
                {2, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::PUMP)},
                {3, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::REGULATOR)},
                {4, std::pair<int,int>(ariac_msgs::msg::Part::RED,ariac_msgs::msg::Part::SENSOR)}};
    
    };
}

/**
 * @class CompetitionARIAC
 * @brief This class manages the ARIAC competition workflow and robot interactions.
 *
 * It subscribes to various topics to receive competition state updates, orders, and sensor data.
 * It also provides functionalities to control a robot for tasks like picking, placing, and moving parts.
 */

class CompetitionARIAC : public rclcpp::Node
{
    public:

        /**
         * @brief Construct a new CompetitionARIAC object.
         *
         * Sets up ROS node, initializes MoveGroup for robot control, and subscribes to necessary topics.
         */

        CompetitionARIAC() : Node("competition_subscriber"),floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot")
        {
            floor_robot_.setMaxAccelerationScalingFactor(1.0);
            floor_robot_.setMaxVelocityScalingFactor(1.0);

            m_callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            //subscription callback groups
            auto subscription_option1 = rclcpp::SubscriptionOptions();
            subscription_option1.callback_group = m_callback_group_1;
            auto subscription_option2 = rclcpp::SubscriptionOptions();
            subscription_option2.callback_group = m_callback_group_2;
            auto subscription_option3 = rclcpp::SubscriptionOptions();
            subscription_option3.callback_group = m_callback_group_3;

            auto bin_cameras_option = rclcpp::SubscriptionOptions();
            bin_cameras_option.callback_group = cb_group_bin_cameras_;
            auto kit_tray_cameras_option = rclcpp::SubscriptionOptions();
            kit_tray_cameras_option.callback_group = cb_group_kit_tray_cameras_;

            auto subscription_option6 = rclcpp::SubscriptionOptions();
            subscription_option6.callback_group = m_callback_group_6;

            // Subscriber objects            
            comp_state_sub = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
                                                                        std::bind(&CompetitionARIAC::CompetitionStateCallback, this, std::placeholders::_1),subscription_option1);
        
            order_sub = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
                                                                        std::bind(&CompetitionARIAC::OrderCallback, this, std::placeholders::_1),subscription_option2);
            
            bin_part_sub = this->create_subscription<ariac_msgs::msg::BinParts>("/ariac/bin_parts", 10, 
                                                                            std::bind(&CompetitionARIAC::BinPartCallback, this, std::placeholders::_1),subscription_option3); 

            kit_tray_table1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::KitTrayTable1Callback, this, std::placeholders::_1), kit_tray_cameras_option);

            kit_tray_table2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::KitTrayTable2Callback, this, std::placeholders::_1), kit_tray_cameras_option);

            left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::LeftBinsCameraCallback, this, std::placeholders::_1), bin_cameras_option);

            right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::RightBinsCameraCallback, this, std::placeholders::_1), bin_cameras_option);    
            
            floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>("/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
                                                                    std::bind(&CompetitionARIAC::floor_gripper_state_cb, this, std::placeholders::_1), subscription_option6);
            
            submit_order_client_ = create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");

            quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");


            AddModelsToPlanningScene();
        };

        /**
         * @brief Destroy the CompetitionARIAC object.
         *
         * Ensures proper resource cleanup and node shutdown.
         */

        ~CompetitionARIAC()
        {
            floor_robot_.~MoveGroupInterface();
        }

        // Member Variables
        bool order_retrived{false}; ///< Indicates if an order has been retrieved.
        bool bin_flag{false};       ///< Status flag for bins.
        bool tray_flag{false};      ///< Status flag for trays.
        bool quality_check_flag{false}; ///< Indicates if a quality check is needed.

        std::vector<int> bin_space{1,2,3,4,5,6,7,8}; ///< List of bin spaces.
        std::vector<int> p_bins{6,5,2,1,7,8,3,4};    ///< Priority bins.
        std::vector<int> empty_bins;                 ///< List of empty bins.
        int empty_bin{0};                            ///< Identifier for an empty bin.
        int order_counter{0};                        ///< Counter for the number of orders processed.
        std::vector<int> order_bins;                 ///< List of bins associated with orders.
        std::vector<bool> quality_check_vec{};       ///< Vector indicating the status of quality checks.
        int slot{2};                                 ///< Slot number for a task.

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
         * This method is used to control the robot to pick a specified part for the kitting process.
         * The part to be picked is specified by the order_::KittingPart structure.
         * 
         * @param part_to_pick The KittingPart struct representing the part to be picked.
         * @return true If the part is successfully picked.
         * @return false If the operation fails.
         */
        bool FloorRobotPickBinPart(order_::KittingPart part_to_pick);

        /**
         * @brief Places a part on the kitting tray.
         * 
         * This method controls the robot to place a part on a specified quadrant of the kitting tray
         * located on an AGV numbered agv_num.
         * 
         * @param agv_num The number of the AGV on which the kitting tray is located.
         * @param quadrant The quadrant of the kitting tray where the part is to be placed.
         * @return true If the part is successfully placed.
         * @return false If the operation fails.
         */
        bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);

        /**
         * @brief Moves an AGV to a specified destination.
         * 
         * This method sends a command to move an AGV, identified by agv_num, to a specified destination.
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
         * This method is used to lock an AGV, identified by agv_num, preventing it from moving.
         * 
         * @param agv_num The number of the AGV to be locked.
         * @return true If the AGV is successfully locked.
         * @return false If the operation fails.
         */
        bool LockAGV(int agv_num);

	private:
	    /**
	     * @brief Indicates if the competition has started.
	     */
	    bool comp_start{false};

	    /**
	     * @brief Flag to indicate whether an order has been submitted.
	     */
	    bool order_submitted{false};

	    /**
	     * @brief Flag to indicate if the current order is a priority order.
	     */
	    bool priority_order = false;

	    /**
	     * @brief Index of the priority order in the order list.
	     */
	    int priority_index = -1;

	    /**
	     * @brief Size of the orders list.
	     */
	    int list_size{0};

	    /**
	     * @brief List of orders currently being processed.
	     */
	    std::vector<order_::Orders> orders_list;

	    /**
	     * @brief List of orders ready to be submitted.
	     */
	    std::vector<order_::Orders> orders_list_submit;

	    // Gripper State
	    /**
	     * @brief Current state of the vacuum gripper.
	     */
	    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;

	    /**
	     * @brief Information about the part currently attached to the floor robot.
	     */
	    ariac_msgs::msg::Part floor_robot_attached_part_comb;

	    /**
	     * @brief Information about the kitting part currently attached to the floor robot.
	     */
	    order_::KittingPart floor_robot_attached_part_;

	    /**
	     * @brief Interface to the MoveGroup for robot movement planning.
	     */
	    moveit::planning_interface::MoveGroupInterface floor_robot_;

	    /**
	     * @brief Interface to the planning scene for managing the robot's understanding of the environment.
	     */
	    moveit::planning_interface::PlanningSceneInterface planning_scene_;

	    /**
	     * @brief Time-optimal trajectory generation for robot movement planning.
	     */
	    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

	    /**
	     * @brief List of parts in the left bins, as detected by sensors.
	     */
	    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;

	    /**
	     * @brief List of parts in the right bins, as detected by sensors.
	     */
	    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;

	    /**
	     * @brief Pose of the left bin.
	     */
	    geometry_msgs::msg::Pose lbin_pose;

	    /**
	     * @brief Pose of the right bin.
	     */
	    geometry_msgs::msg::Pose rbin_pose;

	    /**
	     * @brief List of kit tray poses in table 1.
	     */
	    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;

	    /**
	     * @brief List of kit tray poses in table 2.
	     */
	    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

	    /**
	     * @brief Pose of kit tray 1.
	     */
	    geometry_msgs::msg::Pose kit1_pose;

	    /**
	     * @brief Pose of kit tray 2.
	     */
	    geometry_msgs::msg::Pose kit2_pose;

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
	     * @brief Pose of the camera observing the conveyor.
	     */
	    geometry_msgs::msg::Pose conveyor_camera_pose_;

	    /**
	     * @brief Pose for picking parts from the conveyor.
	     */
	    geometry_msgs::msg::Pose conveyor_camera_pick_pose_;

    private:

        bool comp_start{false};
        bool order_submitted{false};
        bool priority_order = false;
        int priority_index = -1;
        int list_size{0};
        
        std::vector<order_::Orders> orders_list;
        std::vector<order_::Orders> orders_list_submit;

        // Gripper State
        ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
        ariac_msgs::msg::Part floor_robot_attached_part_comb;
        order_::KittingPart floor_robot_attached_part_;

        moveit::planning_interface::MoveGroupInterface floor_robot_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_;
        trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

        std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
        std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;
        geometry_msgs::msg::Pose lbin_pose;
        geometry_msgs::msg::Pose rbin_pose;
        std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
        std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;
        geometry_msgs::msg::Pose kit1_pose;
        geometry_msgs::msg::Pose kit2_pose;
        geometry_msgs::msg::Pose kts1_camera_pose_;
        geometry_msgs::msg::Pose kts2_camera_pose_;
        geometry_msgs::msg::Pose left_bins_camera_pose_;
        geometry_msgs::msg::Pose right_bins_camera_pose_;
        geometry_msgs::msg::Pose conveyor_camera_pose_;
        geometry_msgs::msg::Pose conveyor_camera_pick_pose_;

        // TF
        /**
         * @brief Unique pointer to a tf2_ros::Buffer.
         * 
         * Used for managing transformations in the robot's coordinate frames.
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());

        /**
         * @brief Shared pointer to a tf2_ros::TransformListener.
         * 
         * Listens for transformations published to the tf2 buffer.
         */
        std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Callback Groups
        /**
         * @brief Shared pointer for the first callback group.
         * 
         * Manages callbacks for ROS subscriptions and services.
         */
        rclcpp::CallbackGroup::SharedPtr m_callback_group_1;

        /**
         * @brief Shared pointer for the second callback group.
         */
        rclcpp::CallbackGroup::SharedPtr m_callback_group_2;

        /**
         * @brief Shared pointer for the third callback group.
         */
        rclcpp::CallbackGroup::SharedPtr m_callback_group_3;

        /**
         * @brief Shared pointer for the callback group for bin cameras.
         */
        rclcpp::CallbackGroup::SharedPtr cb_group_bin_cameras_;

        /**
         * @brief Shared pointer for the callback group for kit tray cameras.
         */
        rclcpp::CallbackGroup::SharedPtr cb_group_kit_tray_cameras_;

        /**
         * @brief Shared pointer for the sixth callback group.
         */
        rclcpp::CallbackGroup::SharedPtr m_callback_group_6;

        // ROS Subscriptions
        /**
         * @brief Subscription to the competition state topic.
         * 
         * Receives updates about the state of the ARIAC competition.
         */
        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr comp_state_sub;

        /**
         * @brief Subscription to the orders topic.
         * 
         * Receives new orders for the ARIAC competition.
         */
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub;

        /**
         * @brief Subscription to the bin parts topic.
         * 
         * Receives updates about the parts available in bins.
         */
        rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_part_sub;



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
        bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);

        /**
         * @brief Waits for a specified duration for the robot to attach to an object.
         * 
         * @param timeout The maximum time to wait in seconds.
         */
        void FloorRobotWaitForAttach(double timeout);

        /**
         * @brief Waits for a specified duration for the robot to attach to a part on the kit tray.
         * 
         * @param timeout The maximum time to wait in seconds.
         */
        void FloorRobotWaitForAttachKitTrayPart(double timeout);

        /**
         * @brief Waits for a specified duration for a part to drop.
         * 
         * @param timeout The maximum time to wait in seconds.
         */
        void FloorRobotWaitForDrop(double timeout);

        /**
         * @brief Moves the floor robot upwards to a predefined position.
         */
        void FloorRobotMoveUp();

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
        geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);

        /**
         * @brief Builds a pose from specified x, y, z coordinates and an orientation.
         * 
         * @param x X coordinate.
         * @param y Y coordinate.
         * @param z Z coordinate.
         * @param orientation The orientation as a quaternion.
         * @return The constructed pose.
         */
        geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);

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
        geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);




        /**
         * @brief Adds a model to the planning scene.
         * 
         * @param name The name of the model.
         * @param mesh_file The file path of the mesh representing the model.
         * @param model_pose The pose of the model in the planning scene.
         */
        void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);

        /**
         * @brief Adds multiple models to the planning scene.
         * 
         * This function is typically used to set up the initial state of the planning scene with all required models.
         */
        void AddModelsToPlanningScene();

        /**
         * @brief Callback function for competition state updates.
         * 
         * @param msg Shared pointer to the CompetitionState message.
         */
        void CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);

        /**
         * @brief Callback function for receiving new orders.
         * 
         * @param order_msg Shared pointer to the Order message.
         */
        void OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg);

        /**
         * @brief Callback function for bin part updates.
         * 
         * @param bin_part_msg Shared pointer to the BinParts message.
         */
        void BinPartCallback(const ariac_msgs::msg::BinParts::SharedPtr bin_part_msg);

        /**
         * @brief Callback function for the kit tray table 1 camera.
         * 
         * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
         */
        void KitTrayTable1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

        /**
         * @brief Callback function for the kit tray table 2 camera.
         * 
         * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
         */
        void KitTrayTable2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

        /**
         * @brief Callback function for the left bins camera.
         * 
         * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
         */
        void LeftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

        /**
         * @brief Callback function for the right bins camera.
         * 
         * @param msg Shared pointer to the AdvancedLogicalCameraImage message.
         */
        void RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

        /**
         * @brief Callback function for the state of the floor gripper.
         * 
         * @param msg Shared pointer to the VacuumGripperState message.
         */
        void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

        /**
         * @brief Calls the service to start the competition.
         * 
         * This function sends a request to the competition's start service.
         */
        void callServiceStart();

        /**
         * @brief Calls the service to end the competition.
         * 
         * This function sends a request to the competition's end service.
         */
        void callServiceEnd();

        /**
         * @brief Calls the service to submit an order.
         * 
         * @param order The order to be submitted.
         */
        void callService_submit(std::string order);


        // ARIAC Services
        /**
         * @brief Client for the service to start the ARIAC competition.
         * 
         * This client communicates with the competition's service to initiate the start of the competition.
         */
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_comp_client_;

        /**
         * @brief Client for the service to end the ARIAC competition.
         * 
         * This client communicates with the competition's service to initiate the end of the competition.
         */
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_comp_client_;

        /**
         * @brief Client for submitting orders in the ARIAC competition.
         * 
         * This client sends orders to be processed during the competition.
         */
        rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_client_;

        /**
         * @brief Client for changing the gripper tool of the floor robot.
         * 
         * This client communicates with the service that controls the changing of the robot's gripper tool.
         */
        rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;

        /**
         * @brief Client for controlling the vacuum gripper of the floor robot.
         * 
         * This client enables or disables the vacuum gripper on the floor robot.
         */
        rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;

        /**
         * @brief Client for performing quality checks on parts in the ARIAC competition.
         * 
         * This client sends requests to perform quality checks on assembled parts.
         */
        rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;


};

