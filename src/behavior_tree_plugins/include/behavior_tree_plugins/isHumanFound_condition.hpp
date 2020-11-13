#ifndef CONDITIONTEST_H
#define CONDITIONTEST_H

#include "behaviortree_cpp_v3/condition_node.h"


//Not done yet
namespace BT
{
class IsHumanFoundCondtion : public IsHumanFoundCondtion
{
  public:
    
    IsHumanFoundCondtion(const std::string& name, const NodeConfiguration& config);

  static PortsList providedPorts()
  {
      // This action has a single input port called "message"
      // Any port must have a name. The type is optional.
      return { InputPort<std::string>("target") };
  }

    // The method that is going to be executed by the thread
  virtual NodeStatus on_tick() override;

  private:
    NodeStatus return_status_;
    bool is_human_found_;
};
}

#endif