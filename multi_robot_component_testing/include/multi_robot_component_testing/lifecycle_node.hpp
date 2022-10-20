#ifndef MULTI_ROBOT_COMPONENT_TESTING__LIFECYCLE_NODE_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__LIFECYCLE_NODE_HPP_

#include "mrp_common/lifecycle_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mrp_common_msgs/msg/heartbeat.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace multi_robot_component_testing
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class LifecycleNode : public mrp_common::LifecycleNode
  {
  public:
    LifecycleNode(const std::string name,
                  const std::string ns,
                  const bool auto_start,
                  const bool health_check,
                  const std::chrono::milliseconds heartbeat_interval);
    ~LifecycleNode();

    // CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    // CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    // CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    // CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    // CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  };
}

#endif