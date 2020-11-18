#ifndef CONDITIONTEST_H
#define CONDITIONTEST_H

#include "behaviortree_cpp_v3/condition_node.h"


//Not done yet
namespace tiago_person_following
{
  class HasMovedCondition : public BT::ConditionNode
  {
    public:
      HasMovedCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf);

	static BT::PortsList providedPorts()
	{
		return {BT::InputPort<bool>("moved_flag", "Has the robot moved to the predicted position")};
	}

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

    private:
      bool has_robot_moved;
  };
}

#endif