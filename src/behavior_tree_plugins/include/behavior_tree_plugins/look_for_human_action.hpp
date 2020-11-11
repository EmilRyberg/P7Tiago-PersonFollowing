
#ifndef PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_
#define PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"


using namespace BT;
    class LookForHumanAction : public BT::SyncActionNode
    {
        public:
        LookForHumanAction(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config);
        
        void LookForHumanAction::GetImageClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

        void tick() override;

        // Any BT node that accepts parameters must provide a requiredNodeParameters method
        /*
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts(
            {
                BT::InputPort<int>("wait_duration", 1, "Wait time")
            });
        }*/
    };
  // PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_

