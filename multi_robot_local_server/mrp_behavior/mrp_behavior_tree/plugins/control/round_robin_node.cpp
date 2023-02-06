#include <string>

#include "mrp_behavior_tree/plugins/control/round_robin_node.hpp"

namespace mrp_behavior_tree
{

  RoundRobinNode::RoundRobinNode(const std::string &name)
      : BT::ControlNode::ControlNode(name, {})
  {
  }

  RoundRobinNode::RoundRobinNode(
      const std::string &name,
      const BT::NodeConfiguration &config)
      : BT::ControlNode(name, config)
  {
  }

  BT::NodeStatus RoundRobinNode::tick()
  {
    const auto num_children = children_nodes_.size();

    setStatus(BT::NodeStatus::RUNNING);

    while (num_failed_children_ < num_children)
    {
      TreeNode *child_node = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = child_node->executeTick();

      switch (child_status)
      {
      case BT::NodeStatus::SUCCESS:
      {
        // Wrap around to the first child
        if (++current_child_idx_ >= num_children)
        {
          current_child_idx_ = 0;
        }
        num_failed_children_ = 0;
        ControlNode::haltChildren();
        return BT::NodeStatus::SUCCESS;
      }

      case BT::NodeStatus::FAILURE:
      {
        if (++current_child_idx_ >= num_children)
        {
          current_child_idx_ = 0;
        }
        num_failed_children_++;
        break;
      }

      case BT::NodeStatus::RUNNING:
      {
        return BT::NodeStatus::RUNNING;
      }

      default:
      {
        throw BT::LogicError("Invalid status return from BT node");
      }
      }
    }

    halt();
    return BT::NodeStatus::FAILURE;
  }

  void RoundRobinNode::halt()
  {
    ControlNode::halt();
    current_child_idx_ = 0;
    num_failed_children_ = 0;
  }

} // namespace mrp_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mrp_behavior_tree::RoundRobinNode>("RoundRobin");
}