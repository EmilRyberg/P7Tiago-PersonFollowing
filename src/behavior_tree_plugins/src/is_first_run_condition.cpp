#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include "behavior_tree_plugins/is_first_run_condition.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/geometry_utils.hpp"

#include "iostream"

//Not done yet
namespace tiago_person_following
{
  IsFirstRunCondition::IsFirstRunCondition(
  const std::string& condition_name,
  const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf)
  {
    /*
    config().blackboard->set<bool>("found", false);
    config().blackboard->set<int32_t>("current_id", -1);
    config().blackboard->set<bool>("first_run_flag", true);
    config().blackboard->set<bool>("has_moved", false);
    geometry_msgs::msg::PoseStamped dummy;
    config().blackboard->set<geometry_msgs::msg::PoseStamped>("person_info", dummy);
    */
    
    first_run_var_ = true;
    got_initial_goal_ = false;
    std::cerr << "abcabc " << std::endl;
  }

  BT::NodeStatus IsFirstRunCondition::tick()
  {
    std::cerr << "Is first run? " << first_run_var_ << std::endl;
    if(first_run_var_){
      
      setOutput("found", false);
      geometry_msgs::msg::PoseStamped dummy;
      setOutput("person_info", dummy);
      setOutput("current_id", -1);
      setOutput("moved_flag", false);
      setOutput("got_initial_goal_output", false);
      
      first_run_var_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      getInput("got_initial_goal_input", got_initial_goal_);
      std::cerr << "Initial goal? " << got_initial_goal_ << std::endl;
      if (got_initial_goal_) {
        return BT::NodeStatus::FAILURE;
      }

      return BT::NodeStatus::SUCCESS;
    }
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tiago_person_following::IsFirstRunCondition>("IsFirstRun");  //update when we know the real path
}