#include <string>
#include <memory>
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "has_moved_condition.h"

namespace tiago_person_following
{
	HasMovedCondition::HasMovedCondition(
	const std::string & condition_name,
	const BT::NodeConfiguration & conf): BT::ConditionNode(condition_name, conf)
	{
	}

	BT::NodeStatus HasMovedCondition::tick()
	{
		getInput("moved_flag", has_robot_moved);

		if (has_robot_moved){
			std::cerr << "The robot has already moved to last predicted position." << std::endl;
			return BT::NodeStatus::SUCCESS;
		}
		else
		{
			std::cerr << "The robot has not moved to last predicted position. Moving." << std::endl;
			return BT::NodeStatus::FAILURE;
		}
	}
}  // namespace tiago_person_following

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tiago_person_following::HasMovedCondition>("HasMoved");  //update when we know the real path
}