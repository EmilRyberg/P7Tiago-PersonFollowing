
#ifndef PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_
#define PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

//leftover from the string splitter
struct PersonInfo 
{ 
  string ID;
  float x;
  float y;
  float dx;
  float dy;
};

namespace tiago_follow_person
{
    class FindHumanAction : public BtActionNode<nav2_msgs::action::FindHuman>
    {
        public:
        FindHumanAction(
            const std::string& xml_tag_name,
            const std::string & action_name,
            const BT::NodeConfiguration& config);
        
        void on_tick() override;

        void on_wait_for_result() override;

        void on_success() override;

        void on_aborted() override;

        void on_cancelled() override;

        PersonInfo converFromString(StringView str);

        // Any BT node that accepts parameters must provide a requiredNodeParameters method
        static PortsList providedPorts()
        {
            return providedBasicPorts(
            {
                OutputPort<string>("current_id");
                OutputPort<int>("found_flag");
                OutputPort<PersonInfo>("person_info");
            });
        }
    };
}
#endif  // PERSON_FOLLOW_LOOK_FOR_HUMAN_ACTION_HPP_