#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behavior_tree_plugins/is_human_found.hpp"
#include <string>

//Not done yet
namespace tiago_person_following
{
  IsHumanFoundCondition::IsHumanFoundCondition(
  const std::string& condition_name,
  const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf)
  {
  }

  BNodeStatus IsHumanFoundCondition::on_tick()
  {
    is_human_found_ = getInput("found_flag")

    if(is_human_found_ == false)
    {
      return_status_ = NodeStatus::FAILURE;
    }
    else if(is_human_found_ == true)
    {
      return_status_ = NodesStatus::SUCCES;
    }
    else
    {
      return_status_ = NodeStatus::FAILURE;
    }
    
    return expected_result_;
}
}