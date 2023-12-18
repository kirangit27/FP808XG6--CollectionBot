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

void CompetitionARIAC::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{

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

}

bool CompetitionARIAC::CompleteKittingTask(ariac_msgs::msg::KittingTask task)
{

}

bool CompetitionARIAC::StartCompetition()
{

}


bool CompetitionARIAC::EndCompetition()
{

}


bool CompetitionARIAC::SubmitOrder(std::string order_id)
{

}

bool CompetitionARIAC::MoveAGV(int agv_num , int destination)
{

}
        
bool CompetitionARIAC::LockAGV(int agv_num)
{
    
}



