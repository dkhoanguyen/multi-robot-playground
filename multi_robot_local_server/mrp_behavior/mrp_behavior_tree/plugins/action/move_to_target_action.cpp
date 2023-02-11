#include "mrp_behavior_tree/plugins/action/move_to_target_action.hpp"

namespace mrp_behavior_tree
{
  MoveToTargetAction::MoveToTargetAction(
      const std::string &xml_tag_name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
  {
    double speed;
    getInput("speed",speed);
    std::string planner;
    getInput("planner", planner);
  }
}