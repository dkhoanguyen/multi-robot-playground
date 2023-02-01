#ifndef MRP_LOCAL_SERVER_CORE__LOCAL_BEHAVIOR_HPP_
#define MRP_LOCAL_SERVER_CORE__LOCAL_BEHAVIOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mrp_local_server_core
{
  class Behavior
  {
    virtual ~Behavior(){};
    virtual void initialise(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent) = 0;
    virtual void cleanup() = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;
  };
} // namespace mrp_local_server_core

#endif