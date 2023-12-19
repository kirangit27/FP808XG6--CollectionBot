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


void CompetitionARIAC::OrderCallback(const ariac_msgs::msg::Order::SharedPtr order_msg)
{
     orders_.push_back(*msg);
    
}

void CompetitionARIAC::CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
    competition_state_ = msg->competition_state;
}


void CompetitionARIAC::KitTrayTable1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts1_camera_recieved_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
    kts1_camera_recieved_data = true;
  }

  kts1_trays_ = msg->tray_poses;
  kts1_camera_pose_ = msg->sensor_pose;

}

void CompetitionARIAC::KitTrayTable2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts2_camera_recieved_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
    kts2_camera_recieved_data = true;
  }

  kts2_trays_ = msg->tray_poses;
  kts2_camera_pose_ = msg->sensor_pose;

}

void CompetitionARIAC::LeftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!left_bins_camera_recieved_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from left bins camera");
    left_bins_camera_recieved_data = true;
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;

}

void CompetitionARIAC::RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!right_bins_camera_recieved_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from right bins camera");
    right_bins_camera_recieved_data = true;
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}

void CompetitionARIAC::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    floor_gripper_state_ = *msg;
}

geometry_msgs::msg::Pose CompetitionARIAC::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
    {

    }

void CompetitionARIAC::LogPose(geometry_msgs::msg::Pose p)
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


void CompetitionARIAC::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
    {

    }

void CompetitionARIAC::AddModelsToPlanningScene()
{

}

geometry_msgs::msg::Quaternion CompetitionARIAC::SetRobotOrientation(double rotation)
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

bool CompetitionARIAC::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
{

}

bool CompetitionARIAC::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{

}

bool CompetitionARIAC::CompleteOrders()
{
    while (orders_.size() == 0)
    {
    }

    bool success;
    while (true)
    {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
        success = false;
        break;
        }

        if (orders_.size() == 0)
        {
        if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
        {
            // wait for more orders
            RCLCPP_INFO(get_logger(), "Waiting for orders...");
            while (orders_.size() == 0)
            {
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Completed all orders");
            success = true;
            break;
        }
        }
        current_order_ = orders_.front();
        orders_.erase(orders_.begin());
        int kitting_agv_num = -1;

        if (current_order_.type == ariac_msgs::msg::Order::KITTING)
        {
        CompetitionARIAC::CompleteKittingTask(current_order_.kitting_task);
        kitting_agv_num = current_order_.kitting_task.agv_number;
        }
         // loop until the AGV is at the warehouse
        auto agv_location = -1;
        while (agv_location != ariac_msgs::msg::AGVStatus::WAREHOUSE)
        {
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

bool CompetitionARIAC::CompleteKittingTask(ariac_msgs::msg::KittingTask task)
{
    for (auto s : task.parts)
  {
    //FloorRobot methods
  }

  // Check quality
  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = current_order_.id;
  auto result = quality_checker_->async_send_request(request);
  result.wait();

}

bool CompetitionARIAC::StartCompetition()
{
      while (competition_state_ != ariac_msgs::msg::CompetitionState::READY)
  {
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;

}


bool CompetitionARIAC::EndCompetition()
{
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

    std::string srv_name = "/ariac/end_competition";

    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = client->async_send_request(request);
    result.wait();

    return result.get()->success;

}


bool CompetitionARIAC::SubmitOrder(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;

}

bool CompetitionARIAC::MoveAGV(int agv_num , int destination)
{

}
        
bool CompetitionARIAC::LockAGV(int agv_num)
{
    
}



