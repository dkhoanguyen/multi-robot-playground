#ifndef MRP_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
#define MRP_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace mrp_behavior_tree
{
  class RecoveryNode : public BT::ControlNode
  {
    /**
     * @brief The RecoveryNode has only two children and returns SUCCESS if and only if the first child
     * returns SUCCESS.
     *
     * - If the first child returns FAILURE, the second child will be executed.  After that the first
     * child is executed again if the second child returns SUCCESS.
     *
     * - If the first or second child returns RUNNING, this node returns RUNNING.
     *
     * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
     *
     */

  public:
    RecoveryNode(
        const std::string &name,
        const BT::NodeConfiguration &conf);

    ~RecoveryNode() override = default;

    // Any BT node that accepts parameters must provide a requiredNodeParameters method
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int>("number_of_retries", 1, "Number of retries")};
    }

  private:
    unsigned int current_child_idx_;
    unsigned int number_of_retries_;
    unsigned int retry_count_;

    BT::NodeStatus tick() override;
    void halt() override;
  };
}

#endif