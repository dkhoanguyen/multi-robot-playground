#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mrp_common
{  
  class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    LifecycleNode(const std::string &node_name,
                  const std::string &ns = "",
                  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~LifecycleNode();
  };
}

#endif