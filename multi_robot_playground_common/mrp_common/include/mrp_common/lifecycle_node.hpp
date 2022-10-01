#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mrp_common_msgs/msg/heartbeat.hpp"

namespace mrp_common
{
  class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    LifecycleNode(const std::string &node_name,
                  const std::string &ns = "",
                  const bool &auto_start = false,
                  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~LifecycleNode();

    void createHeartbeatPublisher();
    void startHeartbeatPublisher();
    void stopHeartbeatPublisher();

  protected:
    // Heartbeats
    std::chrono::milliseconds heartbeat_period_;
    rclcpp::Publisher<mrp_common_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    void heartbeatCallback();
  };
}

#endif