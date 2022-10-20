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

  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // LifecycleNode::on_configure(const rclcpp_lifecycle::State &)
  // {
  //   std::cout << "Configuring" << std::endl;
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  // }


  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // LifecycleNode::on_activate(const rclcpp_lifecycle::State &)
  // {
  //   std::cout << "Activating" << std::endl;
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  // }

  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // LifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
  // {
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  // }

  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // LifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
  // {
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  // }

  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // LifecycleNode::on_shutdown(const rclcpp_lifecycle::State & state)
  // {
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  // }
    
  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // LifecycleNode::on_error(const rclcpp_lifecycle::State &)
  // {
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  // }

}