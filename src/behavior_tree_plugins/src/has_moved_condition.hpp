#include <string>
#include <memory>
#include "behaviortree_cpp_v3/condition_node.h"

namespace tiago_person_following
{

class HasMovedCondition : public BT::ConditionNode
{
public:
	HasMovedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

	static BT::PortsList providedPorts()
	{
		return {BT::InputPort<bool>("moved_flag", "Has the robot moved to the predicted position")};
	}

	BT::NodeStatus tick() override
	{
		has_robot_moved = getInput("moved_flag");

		if (has_robot_moved) return BT::NodeStatus::SUCCESS;
		else return BT::NodeStatus::FAILURE;
	}
};

}  // namespace tiago_person_following
