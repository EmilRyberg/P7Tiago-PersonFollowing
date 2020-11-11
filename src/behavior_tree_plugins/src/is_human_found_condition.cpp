#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behavior_tree_plugins/is_human_found.hpp"
#include <string>

bool is_human_found_;
MAT image;
namespace BT
{

class IsHumanFoundCondition : public IsHumanFoundCondtion
{
  public:

  IsHumanFoundCondition::IsHumanFoundCondition(
  const std::string& name,
  const NodeConfiguration& config)
    : IsHumanFoundCondition::IsHumanFoundCondition(name, {})
  {
  image = getInput("target");
  //GetBoundingBox() (function that finds a human in the image, should return bb)
  //is_human_found_ = IsBBHuman()  (true/false)
  }

  static PortsList providedPorts()
  {
      // This action has a single input port called "message"
      // Any port must have a name. The type is optional.
      return { InputPort<std::string>("target") };
  }

  BNodeStatus IsHumanFoundCondition::tick()
  {

    if(is_human_found_ == false)
    {
      expected_results_ = NodeStatus::FAILURE;
    }
    else if(is_human_found_ == true)
    {
      expected_results_ = NodesStatus::SUCCES;
    }
    else
    {
      expected_results: = NodeStatus::FAILURE;
    }
    
    return expected_result_;
}}}