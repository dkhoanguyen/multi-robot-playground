#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__LIFECYLE_MANAGER_CLIENT_HPP_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__LIFECYLE_MANAGER_CLIENT_HPP_

#include <chrono>

#include "mrp_common/service_client.hpp"
#include "mrp_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace mrp_lifecycle_manager
{
  class LifecycleManagerClient
  {
  public:
    LifecycleManagerClient(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                           std::string managed_node_name);
    ~LifecycleManagerClient();

    bool requestTransition(mrp_common::LifecycleNode::Transition transition,
                           const std::chrono::nanoseconds timeout);
    bool requestConfigure(const std::chrono::nanoseconds timeout);
    bool requestCleanup(const std::chrono::nanoseconds timeout);
    bool requestActivate(const std::chrono::nanoseconds timeout);
    bool requestDeactivate(const std::chrono::nanoseconds timeout);
    bool requestUnconfiguredShutdown(const std::chrono::nanoseconds timeout);
    bool requestInactiveShutdown(const std::chrono::nanoseconds timeout);
    bool requestActiveShutdown(const std::chrono::nanoseconds timeout);

    mrp_common::LifecycleNode::State getNodeState(const std::chrono::nanoseconds timeout);

  protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string change_state_service_name_;
    std::string get_state_service_name_;

    std::shared_ptr<mrp_common::ServiceClient<lifecycle_msgs::srv::ChangeState>> change_state_client_;
    std::shared_ptr<mrp_common::ServiceClient<lifecycle_msgs::srv::GetState>> get_state_client_;
  };

}

#endif