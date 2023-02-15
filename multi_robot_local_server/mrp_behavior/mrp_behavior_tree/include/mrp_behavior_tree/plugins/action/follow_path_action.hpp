#ifndef MRP_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define MRP_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <string>

#include "mrp_behavior_tree/bt_action_client_node.hpp"
#include "mrp_msgs/action/follow_path.hpp"

namespace mrp_behavior_tree
{
  class FollowPathAction : public BtActionNode<mrp_msgs::action::FollowPath>
  {
  public:
    FollowPathAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf);

    virtual ~FollowPathAction(){};

    void onTick() {};

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
          {BT::InputPort<double>("speed", 0.15, "Max speed to travel to target"),
           BT::InputPort<std::string>("planner", "mrp_orca", "Motion planner to use")});
    }

  protected:
  };
} // namespace mrp_beha_tree`

#endif
