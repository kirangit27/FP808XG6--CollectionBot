/**
 * @file ccs.cpp
 * @author Kiran S Patil (kpatil27@umd.edu)
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @author Surya Chappidi (chappidi@umd.edu)
 * @brief Implementation file for managing interactions and functionalities in the Automated Robot Industrial Automation Competition (ARIAC).
 * @version 0.1
 * @date 2023-12-6
 * 
 * This file contains the implementation of the CompetitionARIAC class methods. It includes functionalities
 * for handling competition states, processing orders, and controlling robots for various tasks in the ARIAC.
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/ariac_collection_bot/ccs.hpp"

/**
 * @brief Callback function for the Competition State in ARIAC.
 * 
 * This function handles updates to the competition state. If the competition state is READY, it triggers the
 * start of the competition. It logs the current state and performs actions based on the competition's state changes.
 * 
 * @param msg Shared pointer to the CompetitionState message, containing the current state of the competition.
 */
 
void CompetitionARIAC::CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::SharedPtr msg)
{
    if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("CompState Subscriber"), "Competition state is 1, calling service client to Start Competition...");
        callService_start();
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::STARTED &&  comp_start == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 2, Competition Started...\n");
        comp_start = true;
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && order_submitted == false)
    {
          
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 3, all orders retrived...\n");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Submitting Orders ...");
        for (int i = 0; i < int(orders_list_submit.size()); i++)
        {
            if (orders_list_submit[i].priority == 1)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Order submitted with Priority 1 :" + orders_list[i].id);
                callService_submit(orders_list_submit[i].id);
            }
        }
        for (int i = 0; i < int(orders_list_submit.size()); i++)
        {
            if (orders_list_submit[i].priority == 0)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("Order Submission    "), "Order submitted with Priority 0 :" + orders_list[i].id);
                callService_submit(orders_list_submit[i].id);
            }
        }
        order_submitted = true;  
          
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && order_submitted == true)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 3, all orders order_submitted, calling service client to End Competition...");
        callService_end();
        comp_state_sub.reset();
    }
    else if (msg->competition_state == ariac_msgs::msg::CompetitionState::ENDED)
    {
        RCLCPP_INFO(rclcpp::get_logger("CompState Subscriber"), "Competition state is 4, Competition Ended...");
    }
}

void CompetitionARIAC::OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Orders    Subscriber"), " ");
    RCLCPP_INFO(rclcpp::get_logger("Orders    Subscriber"), "Received order !");
    
    order_::Orders order;
    order.id = order_msg->id;
    order.type = order_msg->type;
    order.priority = order_msg->priority;

    if (order_msg->type == ariac_msgs::msg::Order::KITTING)
    {
        const auto &kitting_task = order_msg->kitting_task;
        order.kitting_type.agv_number = static_cast<int>(kitting_task.agv_number);

        order.kitting_type.tray_id = static_cast<int>(kitting_task.tray_id);
        order.kitting_type.destination = kitting_task.destination;
        for (const auto &kitting_part : order_msg->kitting_task.parts)
        {
            order_::KittingPart ki_part;
            ki_part.quadrant = kitting_part.quadrant;
            ki_part.color = kitting_part.part.color;
            ki_part.type = kitting_part.part.type;
            order.kitting_type.parts.push_back(ki_part);
        }
    }
    else if (order_msg->type == ariac_msgs::msg::Order::ASSEMBLY)
    {
        const auto &assembly_task = order_msg->assembly_task;
        order.AssemComb_type.station = static_cast<int>(assembly_task.station);

        for (size_t i = 0; i < assembly_task.agv_numbers.size(); ++i)
        {
            order.AssemComb_type.agv_numbers.push_back(static_cast<int>(assembly_task.agv_numbers[i]));
        }
        for (const auto &assem_part : assembly_task.parts)
        {
            order_::AssemCombPart ac_part;
            ac_part.color = assem_part.part.color;
            ac_part.type = assem_part.part.type;
            ac_part.pose_stamp = assem_part.assembled_pose;
            ac_part.install_direction = assem_part.install_direction;
            order.AssemComb_type.parts.push_back(ac_part);
        }
    }
    else
    {
        const auto &combined_task = order_msg->combined_task;
        order.AssemComb_type.station = static_cast<int>(combined_task.station);
        for (const auto &assem_part : combined_task.parts)
        {
            order_::AssemCombPart ac_part;
            ac_part.color = assem_part.part.color;
            ac_part.type = assem_part.part.type;
            ac_part.pose_stamp = assem_part.assembled_pose;
            ac_part.install_direction = assem_part.install_direction;
            order.AssemComb_type.parts.push_back(ac_part);
        }
    }
    order_counter++;
    
    if(order.priority)
        priority_index = order_counter - 1;
    priority_order = order.priority;
    order_::Orders order_obj;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "  Orders details - ");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tOrder ID      : " + order.id);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tOrder type    : " + order_obj.order_type[order.type]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\tPriority      : " + std::to_string(order.priority));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Orders    Subscriber"), "\torder_counter : " + std::to_string(order_counter)+"\n");

    orders_list.push_back(order);
    orders_list_submit.push_back(order);
    list_size = orders_list.size();

}

void CompetitionARIAC::BinPartCallback(const ariac_msgs::msg::BinParts::SharedPtr bin_part_msg)
{

}

void CompetitionARIAC::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
    {

    }

void CompetitionARIAC::AddModelsToPlanningScene()
{

}

bool CompetitionARIAC::FloorRobotMovetoTarget()
{

}

bool CompetitionARIAC::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
    {

    }
void CompetitionARIAC::FloorRobotWaitForAttach(double timeout)
{

}

void CompetitionARIAC::FloorRobotWaitForAttachKitTrayPart(double timeout)
{

}

void CompetitionARIAC::FloorRobotWaitForDrop(double timeout)
{

}

void CompetitionARIAC::FloorRobotMoveUp()
{

}

void CompetitionARIAC::FloorRobotSendHome()
{

}

bool CompetitionARIAC::FloorRobotSetGripperState(bool enable)
{
    
}

bool CompetitionARIAC::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{

}

bool CompetitionARIAC::FloorRobotPickandPlaceTray(int tray_id, int agv_num)
{

}

bool CompetitionARIAC::FloorRobotPickBinPart(order_::KittingPart part_to_pick)
{

}

bool CompetitionARIAC::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{

}

geometry_msgs::msg::Quaternion CompetitionARIAC::SetRobotOrientation(double rotation)
{

}

void CompetitionARIAC::LogPose(geometry_msgs::msg::Pose p)
{

}

geometry_msgs::msg::Pose CompetitionARIAC::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
    {

    }

geometry_msgs::msg::Pose CompetitionARIAC::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
    {

    }

geometry_msgs::msg::Pose CompetitionARIAC::FrameWorldPose(std::string frame_id)
{

} 

double CompetitionARIAC::GetYaw(geometry_msgs::msg::Pose pose)
{

}

double CompetitionARIAC::GetPitch(geometry_msgs::msg::Pose pose)
{

}

geometry_msgs::msg::Quaternion CompetitionARIAC::QuaternionFromRPY(double r, double p, double y)
{
    
}

void CompetitionARIAC::KitTrayTable1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{

}

void CompetitionARIAC::KitTrayTable2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{

}

void CompetitionARIAC::LeftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{

}

void CompetitionARIAC::RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{

}

void CompetitionARIAC::callServiceStart()
{
    auto client = create_client<std_srvs::srv::Trigger>("/ariac/start_competition");

    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(get_logger(), "Service not available");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
}


void CompetitionARIAC::callServiceEnd()
{
    auto client_ = create_client<std_srvs::srv::Trigger>("/ariac/end_competition");

    if (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(get_logger(), "Service not available");
        return;
    }

    auto request_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_ = client_->async_send_request(request_);
}


void CompetitionARIAC::callService_submit(std::string order)
{
    auto submit_order_request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    submit_order_request->order_id = order;

    auto submit_order_future = submit_order_client_->async_send_request(submit_order_request);

}

bool CompetitionARIAC::MoveAGV(int agv_num , int destination)
{

}
        
bool CompetitionARIAC::LockAGV(int agv_num)
{
    
}



