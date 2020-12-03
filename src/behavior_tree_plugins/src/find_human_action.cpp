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
    //getInput("target_id", look_for_id); //i.e. dont look for a specific ID, but look and track a human and return with the ID
  }

  void FindHumanAction::on_tick() //what the node has to do everyime it runs
  {
    getInput("target_id", look_for_id);
    RCLCPP_INFO(node_->get_logger(), "FindHumanAction: Sending goal: %d", look_for_id);
    goal_.id = look_for_id;  //and this should send the ID to the action server 
  }  

  //code that runs when waiting for result
  void FindHumanAction::on_wait_for_result()
  {
  }

  //code that runs when the action server returns a success result
  BT::NodeStatus FindHumanAction::on_success()
  {
    RCLCPP_INFO(node_->get_logger(), "Action success: Find Person");

    person_id = result_.result->tracked_id; 

    //now we could theoretically use the findHumanAction for both a new human and same human
    if(look_for_id == -1) //should only execute on first run (since look_for_id = -1 on first run only)
    {
      look_for_id = result_.result->tracked_id;
      setOutput("current_id", result_.result->tracked_id);
      setOutput("person_info", result_.result->pose);
      RCLCPP_INFO(node_->get_logger(), "Pose x: %f", result_.result->pose.pose.position.x);
      setOutput("goal", result_.result->pose);
      setOutput("found", true);
      setOutput("got_initial_goal_output", true);
      return BT::NodeStatus::SUCCESS;
    } 
    else if(result_.result->is_tracked)  //should only run when the requested ID is the ID we recieve 
    {
      RCLCPP_INFO(node_->get_logger(), "Action success: Found same person");
      setOutput("person_info", result_.result->pose);
      setOutput("goal", result_.result->pose);
      setOutput("found", true);
      return BT::NodeStatus::SUCCESS;
    }
    else //this is for if the ID recieved from the action server is not the same as the ID the logic needs to track
    {
      RCLCPP_INFO(node_->get_logger(), "Could not find same person");
      setOutput("found", false);
      return BT::NodeStatus::FAILURE;
    }
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
