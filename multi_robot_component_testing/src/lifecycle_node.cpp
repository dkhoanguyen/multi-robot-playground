#include "multi_robot_component_testing/lifecycle_node.hpp"

namespace multi_robot_component_testing
{
  LifecycleNode::LifecycleNode(const std::string name,
                               const std::string ns,
                               const bool auto_start,
                               const bool health_check,
                               const std::chrono::milliseconds heartbeat_interval)
      : mrp_common::LifecycleNode(name, ns, auto_start, health_check, heartbeat_interval)
  {
  }

  LifecycleNode::~LifecycleNode()
  {
  }

}