#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include "behavior_tree_plugins/is_first_run_condition.h"
#include "behaviortree_cpp_v3/bt_factory.h"


//Not done yet
namespace tiago_person_following
{
  IsFirstRunCondition::IsFirstRunCondition(
  const std::string& condition_name,
  const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf)
  {
    //Might need to be some initializing at some point here.
  }

  BT::NodeStatus IsFirstRunCondition::tick()
  {
    getInput("is_first_run", is_first_run_);

    if(is_first_run_)
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

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsFirstRunCondition>("IsFirstRun");  //update when we know the real path
}