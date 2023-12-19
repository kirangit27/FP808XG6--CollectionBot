/**
 * @file ccs_main.cpp
 * @author Kiran S Patil (kpatil27@umd.edu)
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @author Surya Chappidi (chappidi@umd.edu)
 * @brief Main file for managing interactions and functionalities in the Automated Robot Industrial Automation Competition (ARIAC).
 * @version 0.1
 * @date 2023-12-6
 *
 * This file contains the main function which initializes and runs the CompetitionARIAC node 
 * within the ROS2 environment for the ARIAC competition.
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/ariac_collection_bot/ccs.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Main function for the ARIAC competition node.
 * 
 * Initializes ROS2, creates an instance of the CompetitionARIAC node, and runs the node using a
 * multi-threaded executor. This setup allows the node to manage various functionalities required
 * for the ARIAC competition, such as handling orders, controlling robots, and processing sensor inputs.
 * 
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

  auto test_competitor = std::make_shared<CompetitionARIAC>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_competitor);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Start Competition
  test_competitor->StartCompetition();

  // Move Robots to Home Poses
  test_competitor->FloorRobotSendHome();

  // Complete Orders
  test_competitor->CompleteOrders();

  test_competitor->EndCompetition();

  rclcpp::shutdown();

}
