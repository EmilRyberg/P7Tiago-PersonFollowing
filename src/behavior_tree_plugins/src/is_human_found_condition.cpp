#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include "behavior_tree_plugins/is_human_found_condition.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "iostream"


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
    getInput("found", is_human_found_);

    if(is_human_found_)
    {
      std::cerr << "humanFoundCondition: True" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      std::cerr << "humanFoundCondition: False" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tiago_person_following::IsHumanFoundCondition>("IsHumanFound");  //update when we know the real path
}