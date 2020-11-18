#ifndef CONDITIONTEST_H
#define CONDITIONTEST_H

#include "behaviortree_cpp_v3/condition_node.h"


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
      return { BT::InputPort<bool>("first_run_flag") };
    }

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

    private:
      bool is_first_run_ = false;
  };
}

#endif