#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behavior_tree_plugins/is_human_found.hpp"
#include <string>

namespace tiago_person_following
{
  IsHumanFoundCondition::IsHumanFoundCondition(
  const std::string& condition_name,
  const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf)
  {
    //Might need to be some initializing at some point here.
  }

  BT::NodeStatus IsHumanFoundCondition::on_tick()
  {
    is_human_found_ = getInput("found_flag")

    if(is_human_found_)
    {
      RCLCPP_INFO(node_->get_logger(), "Human was found");
      return NodesStatus::SUCCES;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Human was not found");
      return NodeStatus::FAILURE;
    }
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsHumanFoundCondition>("IsHumanFound");  //update when we know the real path
}