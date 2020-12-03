#include <string>
#include <memory>
#include <cmath>
#include "behavior_tree_plugins/reset_kalman_action.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "person_follower_interfaces/action/kalman.hpp"

#include "iostream"
#include <cstdio>

namespace tiago_person_following
{
  ResetKalmanAction::ResetKalmanAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & config)
     : BtActionNode<person_follower_interfaces::action::Kalman>(xml_tag_name, action_name, config)
  {
  }

  void ResetKalmanAction::on_tick() //what the node has to do everyime it runs
  {
    RCLCPP_INFO(node_->get_logger(), "ResetKalmanAction: Requesting reset of Kalman");
    goal_.remove_filter = true;  //and this should send the ID to the action server 
  }  

  //code that runs when waiting for result
  void ResetKalmanAction::on_wait_for_result()
  {
  }

  //code that runs when the action server returns a success result
  BT::NodeStatus ResetKalmanAction::on_success()
  {
    RCLCPP_INFO(node_->get_logger(), "ResetKalman: Got response from Action Server");
      return BT::NodeStatus::SUCCESS;
  }

  //code that runs when the action server returns an aborted result
  BT::NodeStatus ResetKalmanAction::on_aborted()
  {
    RCLCPP_INFO(node_->get_logger(), "Action aborted: Reset kalman");
    return BT::NodeStatus::SUCCESS;
  }

  //code that runs when the actions server returns a cancelled result
  BT::NodeStatus ResetKalmanAction::on_cancelled()
  {
    RCLCPP_INFO(node_->get_logger(), "Action cancelled: Reset kalman");
    return BT::NodeStatus::FAILURE;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<tiago_person_following::ResetKalmanAction>(
        name, "find_human", config);
    };

  factory.registerBuilder<tiago_person_following::ResetKalmanAction>(
    "ResetKalman", builder);
}
