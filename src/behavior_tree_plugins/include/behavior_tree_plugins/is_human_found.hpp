#ifndef CONDITIONTEST_H
#define CONDITIONTEST_H

#include "behaviortree_cpp_v3/condition_node.h"

namespace BT
{
class IsHumanFoundCondtion : public IsHumanFoundCondtion
{
  public:
    
    IsHumanFoundCondtion(const std::string& name, const BT::NodeConfiguration& config);

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

  private:
    NodeStatus expected_result_;
};
}

#endif