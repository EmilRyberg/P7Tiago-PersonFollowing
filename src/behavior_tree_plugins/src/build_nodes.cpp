#include "behaviortree_cpp_v3/bt_factory.h"

// file that contains the custom nodes definitions
#include "findHuman_action.hpp"
#include "has_moved_condition.hpp"
#include "isFirstRun_condition.hpp"
#include "isHumanFound_condition.hpp"

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // Note: the name used to register should be the same used in the XML.
    using namespace tiago_person_following;

    factory.registerNodeType<tiago_person_following::FindHumanAction>("findhuman");
    factory.registerNodeType<tiago_person_following::HasMovedCondition>("hasmoved");
    factory.registerNodeType<tiago_person_following::IsFirstRunCondition>("firstrun");
    factory.registerNodeType<tiago_person_following::IsHumanFoundCondition>("found");
    factory.registerNodeType<tiago_person_following::FindSameHumanAction>("findsamehuman");
    factory.registerNodeType<tiago_person_following::IsDoneCondition>("isdone");

    // IMPORTANT: when the object "tree" goes out of scope, all the
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("./UpdatedBehaviourTree.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}
