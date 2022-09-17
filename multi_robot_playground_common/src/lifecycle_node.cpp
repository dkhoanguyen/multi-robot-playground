#include "multi_robot_playground_common/lifecycle_node.hpp"

namespace mrp_common
{
  LifecycleNode::LifecycleNode(
      const std::string &node_name,
      const std::string &ns,
      const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
  {
  }

  LifecycleNode::~LifecycleNode()
  {
  }
}