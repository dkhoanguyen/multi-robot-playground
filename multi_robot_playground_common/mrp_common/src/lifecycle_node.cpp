#include "mrp_common/lifecycle_node.hpp"

namespace mrp_common
{
  LifecycleNode::LifecycleNode(
      const std::string &node_name,
      const std::string &ns,
      const bool &auto_start,
      const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
  {
  }

  LifecycleNode::~LifecycleNode()
  {
  }

  void LifecycleNode::createHeartbeatPublisher()
  {
    // The granted lease is essentially infite here, i.e., only reader/watchdog will notify
    // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life > heartbeat.
    rclcpp::QoS qos_profile(1);
    qos_profile
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC) // The topic itself has to be alive
        .liveliness_lease_duration(heartbeat_period_)          //
        .deadline(heartbeat_period_);                          //
    // assert liveliness on the 'heartbeat' topic
    // Minimal publisher for publishing heartbeats
    heartbeat_publisher_ = this->create_publisher<mrp_common_msgs::msg::Heartbeat>("heartbeat", qos_profile);
    heartbeat_timer_ = this->create_wall_timer(heartbeat_period_,
                                               std::bind(&LifecycleNode::heartbeatCallback, this));
  }

  void LifecycleNode::startHeartbeatPublisher()
  {
    if (this->heartbeat_timer_->is_canceled())
    {
      this->heartbeat_timer_->reset();
    }
  }

  void LifecycleNode::stopHeartbeatPublisher()
  {
    // Stop timer
    this->heartbeat_timer_->cancel();
  }

  void LifecycleNode::heartbeatCallback()
  {
    auto message = mrp_common_msgs::msg::Heartbeat();
    rclcpp::Time now = this->get_clock()->now();
    message.stamp = now;
    RCLCPP_INFO(this->get_logger(), "Publishing heartbeat, sent at [%f]", now.seconds());
    heartbeat_publisher_->publish(message);
  }
}
