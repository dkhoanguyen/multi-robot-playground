#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_

#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common_msgs/msg/heartbeat.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "mrp_lifecycle_manager/lifecycle_manager_client.hpp"

#include "mrp_lifecycle_manager/visibility_control.h"

namespace mrp_lifecycle_manager
{
  class LifecycleManager : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    LifecycleManager(const rclcpp::NodeOptions &options,
                    std::chrono::milliseconds heartbeat_timeout);
    virtual ~LifecycleManager();

    bool changeNodeState(const std::string &node_name,
                         mrp_common::LifecycleNode::Transition transition);

    bool registerLifecycleNode(const std::string &node_name,
                               const std::chrono::milliseconds &heartbeat_interval);

    bool registerLifecycleNodes(
        const std::vector<std::string> &monitored_node_names,
        const std::vector<std::chrono::milliseconds> &heartbeat_intervals);

    bool startNodeHealthMonitor(const std::string &node_name);
    bool startNodesHealthMonitor(const std::vector<std::string> &monitored_node_names);

  protected:
    class HealthMonitor
    {
      enum class NodeHeartbeat
      {
        HEALTHY = 0,
        UNHEALTHY = 1,
        UNKNOWN = 2
      };

    public:
      HealthMonitor(
          const std::string node_name,
          std::chrono::milliseconds heartbeat_timeout,
          std::chrono::milliseconds heartbeat_interval,
          rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
          rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
          rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
          rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface);
      ~HealthMonitor();

      NodeHeartbeat getStatus();
      bool initialiseHealthMonitor();
      void startMonitoring();
      void stopMonitoring();

    protected:
      /// The lease duration granted to the remote (heartbeat) publisher
      std::chrono::milliseconds heartbeat_timeout_;
      std::chrono::milliseconds heartbeat_interval_;
      rclcpp::Subscription<mrp_common_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
      rclcpp::QoS qos_profile_;
      rclcpp::SubscriptionOptions heartbeat_sub_options_;

      double last_monitored_time_;
      double heartbeat_interval_ms_;
      mutable std::recursive_mutex status_mutex_;
      NodeHeartbeat status_;

      std::string node_name_;
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface_;
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface_;
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;

      void healthCallback(const typename mrp_common_msgs::msg::Heartbeat::SharedPtr msg);
    };

    struct MonitoredNode
    {
      std::shared_ptr<HealthMonitor> heartbeat_ptr_;
      std::shared_ptr<LifecycleManagerClient> lifecyle_manager_client_;
    };

    std::chrono::milliseconds heartbeat_timeout_;
    std::map<std::string, MonitoredNode> monitored_node_map_;
    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  };
} // namespace mrp_lifecycle_manager

#endif // MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_
