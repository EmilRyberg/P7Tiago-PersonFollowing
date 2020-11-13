#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "person_follower_interfaces/msg/PersonInfo.msg"

namespace tiago_person_following
{
  FindHumanAction::FindHumanAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & config)
     : BtActionNode<nav2_msgs::action::FindHuman>(xml_tag_name, action_name, conf)
  {
    getInput("current_id", goal_);  //sends the request to find person with id, this line may have to be in the on_tick() function instead
  }

  void FindHumanAction::on_tick() //what the node has to do everyime it runs
  {
  }  

  //code that runs when waiting for result
  void FindHumanAction::on_wait_for_results()
  {
  }

  //code that runs when the action server returns a success result
  void FindHumanAction::on_success()
  {
    if(result_.person_id != goal_)
    {
        RCLCPP_INFO(node_->get_logger(), "Could not find same person");
        setOutpu("found_flag", false);
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Action success: Found same person");

    pose = result_.pose
    setOutput("person_info", pose);
    setOutout("found_flag", true);
    return NodeStatus::SUCCESS;
  }

  //code that runs when the action server returns an aborted result
  void FindHumanAction::on_aborted()
  {
    RCLCPP_INFO(node_->get_logger(), "Action aborted: Find Person");
  }

  //code that runs when the actions server returns a cancelled result
  void FindHumanAction::on_cancelled()
  {
    RCLCPP_INFO(node_->get_logger(), "Action cancelled: Find Person");
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FindHumanAction>(
        name, "find_human", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FindHumanAction>(
    "FindHuman", builder);
}