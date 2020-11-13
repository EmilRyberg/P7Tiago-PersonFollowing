#ifndef CONDITIONTEST_H
#define CONDITIONTEST_H

#include "behaviortree_cpp_v3/condition_node.h"

namespace tiago_person_following
{
class IsHumanFoundCondtion : public IsHumanFoundCondtion
{
  public:
    
    IsHumanFoundCondtion(
      const std::string& condition_name,
      const BT::NodeConfiguration& conf);

  static BT::PortsList providedPorts()
  {
      // This action has a single input port called "message"
      // Any port must have a name. The type is optional.
      return { InputPort<bool>("target") };
  }

    // The method that is going to be executed by the thread
  virtual BT::NodeStatus on_tick() override;

  private:
    bool is_human_found_ = false;
};
}

#endif