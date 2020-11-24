#include <string>
#include <memory>
#include <cmath>
#include "behavior_tree_plugins/find_human_action.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "person_follower_interfaces/action/kalman.hpp"
//#include "person_follower_interfaces/msg/PersonInfo.msg"

#include "iostream"
#include <cstdio>

namespace tiago_person_following
{
  FindHumanAction::FindHumanAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & config)
     : BtActionNode<person_follower_interfaces::action::Kalman>(xml_tag_name, action_name, config)
  {
    RCLCPP_INFO(node_->get_logger(), "abc");
    look_for_id = -1; //i.e. dont look for a specific ID, but look and track a human and return with the ID
  }

  void FindHumanAction::on_tick() //what the node has to do everyime it runs
  {
    RCLCPP_INFO(node_->get_logger(), "TICK TOCK but in on_tick");
    goal_.id = look_for_id;  //and this should send the ID to the action server (hopefully we will only have to run this node once, so this is fine to have in tick??)
  }  

  //code that runs when waiting for result
  void FindHumanAction::on_wait_for_result()
  {
    std::cerr << "heyo3" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "TICK TOCK 123");
  }

  //code that runs when the action server returns a success result
  BT::NodeStatus FindHumanAction::on_success()
  {
    std::cerr << "heyo4" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "Action success: Find Person");

    pose = result_.result->pose;
    person_id = result_.result->tracked_id;

    setOutput("current_id", person_id);
    setOutput("person_info", result_.result->pose);
    RCLCPP_INFO(node_->get_logger(), "Pose x: %f", pose.pose.position.x);
    setOutput("goal", result_.result->pose);
    setOutput("found", true);
    setOutput("first_run_flag", false);
    return BT::NodeStatus::SUCCESS;
  }

  //code that runs when the action server returns an aborted result
  BT::NodeStatus FindHumanAction::on_aborted()
  {
    RCLCPP_INFO(node_->get_logger(), "Action aborted: Find Person");
    return BT::NodeStatus::FAILURE;
  }

  //code that runs when the actions server returns a cancelled result
  BT::NodeStatus FindHumanAction::on_cancelled()
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
      return std::make_unique<tiago_person_following::FindHumanAction>(
        name, "find_human", config);
    };

  factory.registerBuilder<tiago_person_following::FindHumanAction>(
    "FindHuman", builder);
}
