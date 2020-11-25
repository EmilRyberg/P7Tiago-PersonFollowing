#include <string>
#include <memory>
#include <cmath>
#include "behavior_tree_plugins/logger_action.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "person_follower_interfaces/action/kalman.hpp"
//#include "person_follower_interfaces/msg/PersonInfo.msg"

#include "iostream"
#include <cstdio>

namespace tiago_person_following
{
  LoggerAction::LoggerAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & config)
     : BtActionNode<person_follower_interfaces::action::Kalman>(xml_tag_name, action_name, config)
  {
  }

  void LoggerAction::on_tick() //what the node has to do everyime it runs
  {
    getInput("goal", pose);
    RCLCPP_INFO(node_->get_logger(), "LoggerAction: Pose x: %f, pose y: %f", pose.pose.position.x, pose.pose.position.y);
  }  

  //code that runs when waiting for result
  void LoggerAction::on_wait_for_result()
  {
  }

  //code that runs when the action server returns a success result
  BT::NodeStatus LoggerAction::on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  //code that runs when the action server returns an aborted result
  BT::NodeStatus LoggerAction::on_aborted()
  {
    RCLCPP_INFO(node_->get_logger(), "Action aborted: Find Person");
    return BT::NodeStatus::FAILURE;
  }

  //code that runs when the actions server returns a cancelled result
  BT::NodeStatus LoggerAction::on_cancelled()
  {
    RCLCPP_INFO(node_->get_logger(), "Action cancelled: Find Person");
    return BT::NodeStatus::FAILURE;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<tiago_person_following::LoggerAction>(
        name, "find_human", config);
    };

  factory.registerBuilder<tiago_person_following::LoggerAction>(
    "LoggerAction", builder);
}
