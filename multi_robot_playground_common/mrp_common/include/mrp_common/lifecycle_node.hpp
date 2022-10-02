#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mrp_common_msgs/msg/heartbeat.hpp"

namespace mrp_common
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    LifecycleNode(const std::string node_name,
                  const std::string ns,
                  const bool auto_start,
                  const bool health_check,
                  const std::chrono::milliseconds heartbeat_interval,
                  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~LifecycleNode();

    void initialiseHeartbeat();
    void startHeartbeat();
    void stopHeartbeat();

    std::shared_ptr<mrp_common::LifecycleNode> shared_from_this();

  protected:
    class Heartbeat
    {
    public:
      Heartbeat(std::chrono::milliseconds interval,
                rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
                rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
                rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
                rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface);
      ~Heartbeat();

      void createHeartbeat();
      void startBeating();
      void stopBeating();

    protected:
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface_;
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface_;
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
      // Heartbeats
      std::chrono::milliseconds interval_;
      rclcpp::Publisher<mrp_common_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_{nullptr};
      rclcpp::TimerBase::SharedPtr heartbeat_timer_{nullptr};

      void heartbeatCallback();
    };

    bool health_check_;
    bool auto_start_;

    std::shared_ptr<Heartbeat> heartbeat_ptr_;
    std::chrono::milliseconds heartbeat_interval_;
  };
}

#endif