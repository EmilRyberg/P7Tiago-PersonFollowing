#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "person_follower_interfaces/msg/PersonInfo.msg"

namespace tiago_person_following
{

  FindHumanAction::FindHumanAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & config)
     : BtActionNode<nav2_msgs::action::FindHuman>(xml_tag_name, action_name, conf)
  {
    config.blackboard->set("found_flag", false)
  }

  void FindHumanAction::on_tick() //what the node has to do everyime it runs
  {
    data = FindHumanAction::converFromString(this)

    setOutput("current_id", data.ID);
    setOutput("person_info", data);
    setOutout("found_flag", 1);
    return NodeStatus::SUCCESS;
  }  

  void FindHumanAction::on_wait_for_results()
  {

  }

  void FindHumanAction::on_success()
  {
    
  }

  void FindHumanAction::on_aborted()
  {
    
  }

  void FindHumanAction::on_cancelled()
  {
    
  }

  // this is probably a redundant and not working function
  // made to split up the incoming response from the action server, if it was a string
  PersonInfo FindHumanAction::convertFromString(StringView str)
    {
      // We expect real numbers separated by semicolons
      auto parts = splitString(str, ',');

      PersonInfo output;
      output.ID = convertFromString<string>(parts[0]);
      output.x = convertFromString<float>(parts[2]);
      output.y = convertFromString<float>(parts[3]);
      output.dx = convertFromString<float>(parts[4]);
      output.dy = convertFromString<float>(parts[5]);
      return output;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FindHumanAction>(
        name, "find_human", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FindHumanAction>(
    "FindHuman", builder);
}