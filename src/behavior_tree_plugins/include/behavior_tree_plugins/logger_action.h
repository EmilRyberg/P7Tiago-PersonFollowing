
#ifndef PERSON_FOLLOW_LOGGER_ACTION_ACTION_HPP_
#define PERSON_FOLLOW_LOGGER_ACTION_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "person_follower_interfaces/action/kalman.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace tiago_person_following
{
    class LoggerAction : public nav2_behavior_tree::BtActionNode<person_follower_interfaces::action::Kalman>
    {
        public:
        LoggerAction(
            const std::string& xml_tag_name,
            const std::string & action_name,
            const BT::NodeConfiguration& config);
        
        void on_tick() override;

        void on_wait_for_result() override;

        BT::NodeStatus on_success() override;

        BT::NodeStatus on_aborted() override;

        BT::NodeStatus on_cancelled() override;
        
        // Any BT node that accepts parameters must provide a requiredNodeParameters method
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
            {
                BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
            });
        }

        // prolly this this part wrong because i am a dumbass
        private:
        int32_t look_for_id;
        int32_t person_id;
        geometry_msgs::msg::PoseStamped pose;
    };
}
#endif  // PERSON_FOLLOW_LOGGER_ACTION_ACTION_HPP_