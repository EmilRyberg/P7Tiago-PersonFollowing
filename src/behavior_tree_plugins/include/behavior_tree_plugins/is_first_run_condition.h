#ifndef CONDITIONTEST_H
#define CONDITIONTEST_H

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_util/geometry_utils.hpp"


//Not done yet
namespace tiago_person_following
{
  class IsFirstRunCondition : public BT::ConditionNode
  {
    public:
      IsFirstRunCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts()
    {
      // Any port must have a name. The type is optional.
      return { 
        BT::OutputPort<bool>("found"),
        BT::OutputPort<bool>("moved_flag"),
        BT::OutputPort<int32_t>("current_id"),
        BT::OutputPort<bool>("got_initial_goal_output"),
        BT::InputPort<bool>("got_initial_goal_input"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("person_info")
       };
    }

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

    private:
      bool is_first_run_ = false;
      bool got_initial_goal_ = false;
      bool first_run_var_ = true;
  };
}

#endif