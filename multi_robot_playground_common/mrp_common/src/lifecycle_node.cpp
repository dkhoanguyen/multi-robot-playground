#include "mrp_common/lifecycle_node.hpp"

namespace mrp_common
{
  LifecycleNode::LifecycleNode(
      const std::string node_name,
      const std::string ns,
      const bool auto_start,
      const bool health_check,
      const std::chrono::milliseconds heartbeat_interval,
      const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode(node_name, ns, options),
        auto_start_(auto_start),
        health_check_(health_check),
        heartbeat_interval_(heartbeat_interval)
  {
    std::cout << health_check_ << std::endl;
  }

  LifecycleNode::~LifecycleNode()
  {
  }

  void LifecycleNode::initialiseHeartbeat()
  {
    std::cout << health_check_ << std::endl;
    if (health_check_)
    {
      std::cout << "Initializing health check" << std::endl;
      this->heartbeat_ptr_ = std::make_shared<Heartbeat>(
          heartbeat_interval_,
          this->get_node_topics_interface(),
          this->get_node_base_interface(),
          this->get_node_timers_interface(),
          this->get_node_clock_interface());
      this->heartbeat_ptr_->createHeartbeat();
    }
  }

  void LifecycleNode::startHeartbeat()
  {
    return this->heartbeat_ptr_->startBeating();
  }

  void LifecycleNode::stopHeartbeat()
  {
    return this->heartbeat_ptr_->stopBeating();
  }

  std::shared_ptr<mrp_common::LifecycleNode> LifecycleNode::shared_from_this()
  {
    return std::static_pointer_cast<mrp_common::LifecycleNode>(
        rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  CallbackReturn LifecycleNode::on_configure(const rclcpp_lifecycle::State &state)
  {
  }

  CallbackReturn LifecycleNode::on_activate(const rclcpp_lifecycle::State &state)
  {
  }

  CallbackReturn LifecycleNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
  }

  CallbackReturn LifecycleNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
  }
  CallbackReturn LifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
  }

  LifecycleNode::Heartbeat::Heartbeat(
      std::chrono::milliseconds interval,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface)
      : interval_(interval),
        node_topic_interface_(node_topic_interface),
        node_base_interface_(node_base_interface),
        node_timer_interface_(node_timer_interface),
        node_clock_interface_(node_clock_interface)
  {
  }

  LifecycleNode::Heartbeat::~Heartbeat()
  {
  }

  void LifecycleNode::Heartbeat::createHeartbeat()
  {
    if (!this->heartbeat_publisher_)
    {
      // The granted lease is essentially infite here, i.e., only reader/watchdog will notify
      // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life > heartbeat.
      rclcpp::QoS qos_profile(1);
      qos_profile
          .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC) // The topic itself has to be alive
          .liveliness_lease_duration(interval_)                  //
          .deadline(interval_);                                  //
      // assert liveliness on the 'heartbeat' topic
      // Minimal publisher for publishing heartbeats
      this->heartbeat_publisher_ = rclcpp::create_publisher<mrp_common_msgs::msg::Heartbeat>(
          this->node_topic_interface_, "heartbeat", qos_profile);
    }
    if (!this->heartbeat_timer_)
    {
      this->heartbeat_timer_ = rclcpp::create_wall_timer(
          interval_,
          std::bind(&LifecycleNode::Heartbeat::heartbeatCallback, this),
          nullptr,
          this->node_base_interface_.get(),
          this->node_timer_interface_.get());

      this->stopBeating();
    }
  }

  void LifecycleNode::Heartbeat::startBeating()
  {
    if (this->heartbeat_timer_->is_canceled())
    {
      this->heartbeat_timer_->reset();
    }
  }

  void LifecycleNode::Heartbeat::stopBeating()
  {
    // this->heartbeat_timer_->cancel();
  }

  void LifecycleNode::Heartbeat::heartbeatCallback()
  {
    auto message = mrp_common_msgs::msg::Heartbeat();
    rclcpp::Time now = node_clock_interface_->get_clock()->now();
    message.stamp = now;
    // RCLCPP_INFO(node_ptr_->get_logger(), "Publishing heartbeat, sent at [%f]", now.seconds());
    heartbeat_publisher_->publish(message);
  }
}
