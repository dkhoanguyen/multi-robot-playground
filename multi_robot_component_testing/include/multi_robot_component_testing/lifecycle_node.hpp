#ifndef MULTI_ROBOT_COMPONENT_TESTING__LIFECYCLE_NODE_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__LIFECYCLE_NODE_HPP_

#include "mrp_common/lifecycle_node.hpp"

namespace multi_robot_component_testing
{
  class LifecycleNode : public mrp_common::LifecycleNode
  {
  public:
    LifecycleNode(const std::string name,
                  const std::string ns,
                  const bool auto_start,
                  const bool health_check,
                  const std::chrono::milliseconds heartbeat_interval);
    ~LifecycleNode();
  };
}

#endif