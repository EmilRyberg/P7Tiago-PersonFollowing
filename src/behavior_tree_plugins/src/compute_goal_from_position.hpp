#include <inttypes.h>
#include <memory>
#include <iostream>
#include <string>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

namespace tiago_person_following
{

class GetGoalNode : public BT::SyncActionNode
{
public:
	GetGoalNode::GetGoalNode(const std::string & name, const BT::NodeConfiguration & config)
		: SyncActionNode(name,config)
  {
		//code to initialize the ros nodes (call the action server)
		//expected return is pos and vel
  }

  static PortsList providedPorts()
  {
        return {
        	BT::InputPort<int>("current_id"),
			BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
        }
  }

  BT::NodeStatus tick() override
  {
	  Optional<int> current_id = getInput<int>("current_id");
	  if (!current_id) {
		  throw BT::RunTimeError("Missing required input in ComputeGoalNode", current_id.error());
	  }

	  //No idea what I'm doing from here
	  rclcpp::init();

	  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("GetGoal");
	  rclcpp::Client<ACTIONSERVER>::SharedPtr client = node->create_client<ACTIONSERVER>::GetGoal;

	  auto request = std::make_shared<ACTIONSERVER::GetGoal::Request>();
	  request->current_id = current_id;
	  while (!client->wait_for_service(1s)) {
	    if (!rclcpp::ok()) {
	      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	      return 0;
	    }
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	  }

	  auto result = client->async_send_request(request);

	  auto result = client->async_send_request(request);
	  // Wait for the result.
	  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
	  {
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
	  }
	  else
	  {
	    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
	  }

	  rclcpp::shutdown();

	  return NodeStatus::SUCCESS;
  }

};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__FOLLOW_PATH_NODE_HPP_
