#include "mrp_behavior_tree/plugins/control/recovery_node.hpp"

namespace mrp_behavior_tree
{
  RecoveryNode::RecoveryNode(
      const std::string &name,
      const BT::NodeConfiguration &conf)
      : BT::ControlNode::ControlNode(name, conf),
        current_child_idx_(0),
        number_of_retries_(1),
        retry_count_(0)
  {
    getInput("number_of_retries", number_of_retries_);
  }

  BT::NodeStatus RecoveryNode::tick()
  {
    const unsigned children_count = children_nodes_.size();

    if (children_count != 2)
    {
      throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
    }

    setStatus(BT::NodeStatus::RUNNING);

    while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_)
    {
      TreeNode *child_node = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = child_node->executeTick();

      if (current_child_idx_ == 0)
      {
        switch (child_status)
        {
        case BT::NodeStatus::SUCCESS:
        {
          retry_count_ = 0;
          halt();
          return BT::NodeStatus::SUCCESS;
        }

        case BT::NodeStatus::FAILURE:
        {
          // tick second child
          if (retry_count_ < number_of_retries_)
          {
            current_child_idx_++;
            break;
          }
          else
          {
            ControlNode::haltChildren();
            return BT::NodeStatus::FAILURE;
          }
        }

        case BT::NodeStatus::RUNNING:
        {
          return BT::NodeStatus::RUNNING;
        }

        default:
        {
          halt();
          return BT::NodeStatus::FAILURE;
        }
        } // end switch
      }
      else if (current_child_idx_ == 1)
      {
        switch (child_status)
        {
        case BT::NodeStatus::SUCCESS:
        {
          retry_count_++;
          current_child_idx_--;
          ControlNode::haltChildren();
        }
        break;

        case BT::NodeStatus::FAILURE:
        {
          current_child_idx_--;
          retry_count_ = 0;
          halt();
          return BT::NodeStatus::FAILURE;
        }

        case BT::NodeStatus::RUNNING:
        {
          return BT::NodeStatus::RUNNING;
        }

        default:
        {
          halt();
          return BT::NodeStatus::FAILURE;
        }
        } // end switch
      }
    } // end while loop
    retry_count_ = 0;
    halt();
    return BT::NodeStatus::FAILURE;
  }

  void RecoveryNode::halt()
  {
    ControlNode::halt();
    current_child_idx_ = 0;
    retry_count_ = 0;
  }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mrp_behavior_tree::RecoveryNode>("RecoveryNode");
}