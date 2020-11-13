#ifndef CONDITIONTEST_H
#define CONDITIONTEST_H

#include "behaviortree_cpp_v3/condition_node.h"


//Not done yet
namespace tiago_person_following
{
class IsFirstRunCondtion : public IsFirstRunCondtion
{
  public:
    IsFirstRunCondtion(
      const std::string& condition_name,
      const BT::NodeConfiguration& conf);

  static BT::PortsList providedPorts()
  {
      // Any port must have a name. The type is optional.
      return { InputPort<bool>("{is_first_run") };
  }

    // The method that is going to be executed by the thread
  virtual BT::NodeStatus on_tick() override;

  private:
    bool is_first_run_ = false;
};
}

#endif