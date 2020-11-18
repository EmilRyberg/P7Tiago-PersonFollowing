#include <string>
#include <memory>
#include <cmath>
#include "behavior_tree_plugins/find_same_human.h"
#include "person_follower_interfaces/action/kalman.hpp"


#include "nav2_behavior_tree/bt_action_node.hpp"

namespace tiago_person_following
{
  FindSameHumanAction::FindSameHumanAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & config)
     : BtActionNode<person_follower_interfaces::action::Kalman>(xml_tag_name, action_name, config)
  {
    getInput("current_id", current_id);  //sends the request to find person with id, this line may have to be in the on_tick() function instead
    goal_.id = current_id;
  }

  void FindSameHumanAction::on_tick() //what the node has to do everyime it runs
  {
    goal_.id = current_id;
  }  

  //code that runs when waiting for result
  void FindSameHumanAction::on_wait_for_result()
  {
  }

  //code that runs when the action server returns a success result
  BT::NodeStatus FindSameHumanAction::on_success()
  {
    if(result_.result->tracked_id != current_id)
    {
        RCLCPP_INFO(node_->get_logger(), "Could not find same person");
        setOutput("found_flag", false);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Action success: Found same person");

    point = result_.result->point;
    setOutput("person_info", point);
    setOutput("found_flag", true);
    return BT::NodeStatus::SUCCESS;
  }

  //code that runs when the action server returns an aborted result
  BT::NodeStatus FindSameHumanAction::on_aborted()
  {
    RCLCPP_INFO(node_->get_logger(), "Action aborted: Find Person");
    return BT::NodeStatus::FAILURE;
  }

  //code that runs when the actions server returns a cancelled result
  BT::NodeStatus FindSameHumanAction::on_cancelled()
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
      return std::make_unique<tiago_person_following::FindSameHumanAction>(
        name, "find_same_human", config);
    };

  factory.registerBuilder<tiago_person_following::FindSameHumanAction>(
    "FindSameHuman", builder);
}