#include <string>
#include <memory>

#include "behavior_tree_plugins/look_for_human_action.hpp"
#include "marathon_ros2/src/navigation2/nav2_behavior_tree/plugins/action/wait_action.hpp"

namespace pf
{

  LookForHumanAction::LookForHumanAction(
    const std::string & xml_tag_name, 
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
  {
    int duration;
    getInput("wait_duration", duration);
    if (duration <= 0) {
      RCLCPP_WARN(
        node_->get_logger(), "Wait duration is negative or zero "
        "(%i). Setting to positive.", duration);
      duration *= -1;
    }

    goal_.time.sec = duration;
  }

  void LookForHumanAction::on_tick()
  {
    increment_recovery_count();
  }

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::WaitAction>(name, "wait", config);
    };

  factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);
}