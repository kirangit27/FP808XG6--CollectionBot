#pragma once
//Added necessary header files
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

    // struct AssemCombPart{
    //     uint8_t color;
    //     uint8_t type;
    //     geometry_msgs::msg::Pose pose;
    //     geometry_msgs::msg::PoseStamped pose_stamp;
    //     geometry_msgs::msg::Vector3 install_direction;
    // };

    // struct AssemCombType{
    //     uint8_t station;
    //     std::vector<uint8_t> agv_numbers;
    //     std::vector<AssemCombPart> parts;
    // };

    class Orders{
    public:
        std::string id;
        uint8_t type;
        bool priority;
        KittingType kitting_type;
        bool order_status{0};
        // AssemCombType AssemComb_type;

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


class CompetitionARIAC : public rclcpp::Node
{
    public:

    CompetitionARIAC() : Node("competition_subscriber"){
        m_callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //subscription callback groups
        auto subscription_option1 = rclcpp::SubscriptionOptions();
        subscription_option1.callback_group = m_callback_group_1;
        auto subscription_option2 = rclcpp::SubscriptionOptions();
        subscription_option2.callback_group = m_callback_group_2;

        // Subscriber objects            
        comp_state_sub = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
                                                                    std::bind(&CompetitionARIAC::CompetitionStateCallback, this, std::placeholders::_1),subscription_option1);
        order_sub = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
                                                                    std::bind(&CompetitionARIAC::OrderCallback, this, std::placeholders::_1),subscription_option2);
        submit_order_client_ = create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
    };

    private:

        bool comp_start{false};
        bool order_submitted{false};
        bool priority_order = false;
        int priority_index = -1;
        int order_counter{0};
        int list_size{0};
        std::vector<order_::Orders> orders_list;
        std::vector<order_::Orders> orders_list_submit;
         
        rclcpp::CallbackGroup::SharedPtr m_callback_group_1;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_2;

        rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr comp_state_sub;
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub;

        void CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
        void OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg); 
        
        void callServiceStart();
        void callServiceEnd();
        void callService_submit(std::string order);

        // ARIAC Services
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_comp_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_comp_client_;
        rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_client_;


};

