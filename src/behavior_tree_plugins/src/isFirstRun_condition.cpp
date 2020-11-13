#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behavior_tree_plugins/is_human_found.hpp"
#include <string>

//Not done yet
namespace tiago_person_following
{
  IsFirstRunCondition::IsFirstRunCondition(
  const std::string& condition_name,
  const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf)
  {
    //Might need to be some initializing at some point here.
  }

  BT::NodeStatus IsFirstRunCondition::on_tick()
  {
    is_human_found_ = getInput("is_first_run")

    if(is_human_found_)
    {
      RCLCPP_INFO(node_->get_logger(), "This is the first run");
      return NodesStatus::SUCCES;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "This is not the first run");
      return NodeStatus::FAILURE;
    }
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsFirstRunCondition>("IsFirstRun");  //update when we know the real path
}