#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include "behavior_tree_plugins/is_human_found_condition.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace tiago_person_following
{
  IsHumanFoundCondition::IsHumanFoundCondition(
  const std::string& condition_name,
  const BT::NodeConfiguration& conf) 
  : BT::ConditionNode(condition_name, conf)
  {
    //Might need to be some initializing at some point here.
  }

  BT::NodeStatus IsHumanFoundCondition::tick()
  {
    getInput("found_flag", is_human_found_);

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

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tiago_person_following::IsHumanFoundCondition>("IsHumanFound");  //update when we know the real path
}