
#ifndef PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_
#define PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/action/wait.h"

namespace tiago_person_following
{
    class FindSameHumanAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait>
    {
        public:
        FindSameHumanAction(
            const std::string& xml_tag_name,
            const std::string & action_name,
            const BT::NodeConfiguration& config);
        
        void on_tick() override;

        void on_wait_for_result() override;

        BT::NodeStatus on_success() override;

        BT::NodeStatus on_aborted() override;

        BT::NodeStatus on_cancelled() override;
        
        // Any BT node that accepts parameters must provide a requiredNodeParameters method
        static PortsList providedPorts()
        {
            return providedBasicPorts(
            {
                BT::InputPort<int32>("current_id");
                BT::OutputPort<int>("found_flag");
                BT::OutputPort<nav_msgs::msgs::PoseStamped>("person_info");
            });
        }

        // prolly this this part wrong because i am a dumbass
        private:
        int32 person_id;
        nav_msgs::msg::PoseStamped pose;

    };
}
#endif  // PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_