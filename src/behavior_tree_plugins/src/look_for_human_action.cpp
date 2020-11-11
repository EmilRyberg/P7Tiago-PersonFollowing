#include <string>
#include <memory>
#include "behavior_tree_plugins/look_for_human_action.hpp"
#include "marathon_ros2/src/navigation2/nav2_behavior_tree/plugins/action/wait_action.hpp"


#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

class LookForHumanAction : public BT::SyncActionNode
{
  public:
  using GoalHandleImage = rclcpp_action::ClientGoalHandle<Image>;

  LookForHumanAction::LookForHumanAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
  {
        //code to initialize the ros nodes (call the action server)
        //expected return is pos and vel
  }

  void LookForHumanAction::GetImageClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("minimal_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<Image>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "get_image");
  }

  bool LookForHumanAction::is_goal_done() const
  {
    return this->goal_done_;
  }

  void LookForHumanAction::send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Image::Goal();
    goal_msg.order = 1;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Image>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&LookForHumanAction::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&LookForHumanAction::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&LookForHumanAction::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  static PortsList providedPorts()
  {
      // This action has a single input port called "message"
      // Any port must have a name. The type is optional.
      return { OutputPort<std::string>("output")};
  }

  void LookForHumanAction::tick()
  {

      this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&LookForHumanAction::send_goal, this));

    setOutput("output", image);
    return NodeStatus::SUCCESS;
  }

  private:
  rclcpp_action::Client<Image>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(std::shared_future<GoalHandleImage::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleImage::SharedPtr,
    const std::shared_ptr<const Image::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Taking image",
      feedback->sequence.back());
  }

  void result_callback(const GoalHandleImage::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
      RCLCPP_INFO(this->get_logger(), "Image has been taken";
    }
  }
};
}
}  