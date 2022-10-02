#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_

#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mrp_common_msgs/msg/heartbeat.hpp"
#include "multi_robot_lifecycle_manager/visibility_control.h"

namespace mrp_lifecycle_manager
{
  class LifecyleManager : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    LifecyleManager(const rclcpp::NodeOptions &options);
    virtual ~LifecyleManager();

    bool registerWatchdog(const std::string &node_name);

  protected:
    class Watchdog
    {
      public:
        Watchdog();
        ~Watchdog();
      
    };
    /// The lease duration granted to the remote (heartbeat) publisher
    std::chrono::milliseconds lease_duration_;
    rclcpp::Subscription<mrp_common_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_ = nullptr;
    rclcpp::QoS qos_profile_;
    rclcpp::SubscriptionOptions heartbeat_sub_options_;
  };
} // namespace multi_robot_lifecycle_manager

#endif // MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_
