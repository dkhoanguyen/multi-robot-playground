#include "mrp_behavior_tree/plugins/action/follow_path_action.hpp"

namespace mrp_behavior_tree
{
  FollowPathAction::FollowPathAction(
      const std::string &xml_tag_name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : BtActionNode<mrp_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
  {
    double speed;
    getInput("speed",speed);
    std::string planner;
    getInput("planner", planner);
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<mrp_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<mrp_behavior_tree::FollowPathAction>("FollowPath", builder);
}
