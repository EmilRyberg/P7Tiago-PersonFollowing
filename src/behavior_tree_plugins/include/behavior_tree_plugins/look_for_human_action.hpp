
#ifndef PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_
#define PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

using namespace nav2_behavior_tree;

namespace pf
{
    class LookForHumanAction : public BtActionNode<nav2_msgs::action::Wait>
    {
        public:
        LookForHumanAction(
            const std::string & xml_tag_name,
            const std::string & action_name,
            const BT::NodeConfiguration & conf);

        void on_tick() override;

        // Any BT node that accepts parameters must provide a requiredNodeParameters method
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
            {
                BT::InputPort<int>("wait_duration", 1, "Wait time")
            });
        }
    };
}  // namespace pf
#endif  // PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_

