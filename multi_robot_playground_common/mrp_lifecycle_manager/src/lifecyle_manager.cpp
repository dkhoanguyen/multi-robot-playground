#include "mrp_lifecycle_manager/lifecyle_manager.hpp"

namespace mrp_lifecycle_manager
{

  LifecycleManager::LifecycleManager(const rclcpp::NodeOptions &options,
                                     std::chrono::milliseconds heartbeat_timeout)
      : rclcpp_lifecycle::LifecycleNode("lifecycle_manager", options),
        heartbeat_timeout_(heartbeat_timeout)
  {
  }

  LifecycleManager::~LifecycleManager()
  {
  }

  bool LifecycleManager::registerLifecycleNode(const std::string &node_name,
                                               const std::chrono::milliseconds &heartbeat_interval)
  {
    if (monitored_node_map_.find(node_name) == monitored_node_map_.end() &&
        heartbeat_timeout_.count() > 0.0)
    {
      MonitoredNode monitored_node;
      monitored_node.heartbeat_ptr_ = std::make_shared<LifecycleManager::HealthMonitor>(
          node_name,
          heartbeat_timeout_,
          heartbeat_interval,
          this->get_node_topics_interface(),
          this->get_node_base_interface(),
          this->get_node_timers_interface(),
          this->get_node_clock_interface());

      // Create lifecycle_manager_client for requesting state changes
      monitored_node.lifecyle_manager_client_ = std::make_shared<LifecycleManagerClient>(
          this->shared_from_this(),
          node_name);

      // Add to the monitored map
      monitored_node_map_[node_name] = monitored_node;

      return true;
    }
    return false;
  }

  bool LifecycleManager::registerLifecycleNodes(
      const std::vector<std::string> &monitored_node_names,
      const std::vector<std::chrono::milliseconds> &heartbeat_intervals)
  {
    for (unsigned int idx = 0; idx < monitored_node_names.size(); idx++)
    {
      // Manually register each lifecycle node
      registerLifecycleNode(monitored_node_names.at(idx), heartbeat_intervals.at(idx));
    }
  }

  bool LifecycleManager::startNodeHealthMonitor(const std::string &node_name)
  {
    // Lifecycle Nodes are designed to broadcast heartbeat automatically on creation
    // So we should start with checking if the node is healthy
    bool unhealthy_node_exist = false;
    if (!monitored_node_map_[node_name].heartbeat_ptr_->initialiseHealthMonitor())
    {
      // Log error message
      unhealthy_node_exist = true;
      // Move on to the next node and flag this node
    }
    return unhealthy_node_exist;
  }

  bool LifecycleManager::startNodesHealthMonitor(const std::vector<std::string> &monitored_node_names)
  {
    bool all_node_healthy = true;
    for (const std::string &node_name : monitored_node_names)
    {
      // Manually register each lifecycle node
      all_node_healthy = startNodeHealthMonitor(node_name);
    }
    return all_node_healthy;
  }

  bool LifecycleManager::changeNodeState(const std::string &node_name,
                                         mrp_common::LifecycleNode::Transition transition)
  {
  }

  //=================================================//
  //                                                 //
  // ================================================//

  LifecycleManager::HealthMonitor::HealthMonitor(
      const std::string node_name,
      std::chrono::milliseconds heartbeat_timeout,
      std::chrono::milliseconds heartbeat_interval,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface)
      : node_name_(node_name),
        heartbeat_timeout_(heartbeat_timeout),
        heartbeat_interval_(heartbeat_interval),
        node_topic_interface_(node_topic_interface),
        node_base_interface_(node_base_interface),
        node_timer_interface_(node_timer_interface),
        node_clock_interface_(node_clock_interface),
        qos_profile_(10)
  {
    heartbeat_interval_ms_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(heartbeat_interval_).count();
  }

  LifecycleManager::HealthMonitor::~HealthMonitor()
  {
  }

  bool LifecycleManager::HealthMonitor::initialiseHealthMonitor()
  {
    qos_profile_
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
        .liveliness_lease_duration(heartbeat_interval_);
    heartbeat_sub_options_.event_callbacks.liveliness_callback =
        [this](rclcpp::QOSLivelinessChangedInfo &event) -> void
    {
      printf("Reader Liveliness changed event: \n");
      printf("  alive_count: %d\n", event.alive_count);
      printf("  not_alive_count: %d\n", event.not_alive_count);
      printf("  alive_count_change: %d\n", event.alive_count_change);
      printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
      if (event.alive_count == 0)
      {
        status_ = NodeHeartbeat::UNKNOWN;
      }
    };
    std::string topic_name = node_name_ + "/heartbeat";
    heartbeat_sub_ = rclcpp::create_subscription<mrp_common_msgs::msg::Heartbeat>(
        node_topic_interface_,
        topic_name,
        qos_profile_,
        std::bind(&LifecycleManager::HealthMonitor::healthCallback, this, std::placeholders::_1));
    last_monitored_time_ = node_clock_interface_->get_clock()->now().nanoseconds();
  }

  void LifecycleManager::HealthMonitor::startMonitoring()
  {
    // Spin executor
  }

  void LifecycleManager::HealthMonitor::healthCallback(const typename mrp_common_msgs::msg::Heartbeat::SharedPtr msg)
  {
    std::lock_guard<std::recursive_mutex> lck_guard(status_mutex_);
    if ((last_monitored_time_ - msg->stamp.nanosec) / 1e6 < heartbeat_interval_ms_)
    {
      status_ = NodeHeartbeat::HEALTHY;
      return;
    }
    status_ = NodeHeartbeat::UNHEALTHY;
    return;
  }

  LifecycleManager::HealthMonitor::NodeHeartbeat LifecycleManager::HealthMonitor::getStatus()
  {
    std::lock_guard<std::recursive_mutex> lck_guard(status_mutex_);
    return status_;
  }
} // namespace mrp_lifecycle_manager
