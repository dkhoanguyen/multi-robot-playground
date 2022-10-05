#include "mrp_lifecycle_manager/lifecyle_manager.hpp"

namespace mrp_lifecycle_manager
{

  LifecyleManager::LifecyleManager(const rclcpp::NodeOptions &options,
                                   std::chrono::milliseconds heartbeat_timeout)
      : rclcpp_lifecycle::LifecycleNode("lifecycle_manager", options),
        heartbeat_timeout_(heartbeat_timeout)
  {
  }

  LifecyleManager::~LifecyleManager()
  {
  }

  void LifecyleManager::setMonitoredNodeList(std::vector<std::string> monitored_node_names)
  {
    monitored_node_names_ = monitored_node_names;
    for (std::string &node_name : monitored_node_names_)
    {
      // Create a lifecyle_manager client
      std::shared_ptr<mrp_lifecycle_manager::LifecycleManagerClient> client =
          std::make_shared<mrp_lifecycle_manager::LifecycleManagerClient>(
              this->shared_from_this(),
              node_name);
      client_map_[node_name] = client;


    }
  }

  bool LifecyleManager::changeNodeState(const std::string &node_name,
                                        mrp_common::LifecycleNode::Transition transition)
  {
  }

  bool LifecyleManager::registerLifecycleNode(const std::string &node_name,
                                              const std::chrono::milliseconds &heartbeat_interval)
  {
    const double timeout_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_timeout_).count();
    const double timeout_s = timeout_ns / 1e9;

    // Create a health monitor instance to this node and record it if a similiar instance does
    // not exist in the map
    if (monitored_map_.find(node_name) == monitored_map_.end() && heartbeat_timeout_.count() > 0.0)
    {
      monitored_map_[node_name] =
          std::make_shared<LifecyleManager::HealthMonitor>(
              node_name,
              heartbeat_timeout_,
              heartbeat_interval,
              this->get_node_topics_interface(),
              this->get_node_base_interface(),
              this->get_node_timers_interface(),
              this->get_node_clock_interface());

      if (!monitored_map_[node_name]->initialiseHealthMonitor())
      {
        RCLCPP_ERROR(
            get_logger(),
            "Server %s was unable to be reached after %0.2fs by bond. "
            "This server may be misconfigured.",
            node_name.c_str(), timeout_s);
        return false;
      }
    }
    return true;
  }

  LifecyleManager::HealthMonitor::HealthMonitor(
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

  LifecyleManager::HealthMonitor::~HealthMonitor()
  {
  }

  bool LifecyleManager::HealthMonitor::initialiseHealthMonitor()
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
        status_ = NodeStatus::UNKNOWN;
      }
    };
    std::string topic_name = node_name_ + "/heartbeat";
    heartbeat_sub_ = rclcpp::create_subscription<mrp_common_msgs::msg::Heartbeat>(
        node_topic_interface_,
        topic_name,
        qos_profile_,
        std::bind(&LifecyleManager::HealthMonitor::healthCallback, this, std::placeholders::_1));
    last_monitored_time_ = node_clock_interface_->get_clock()->now().nanoseconds();
  }

  void LifecyleManager::HealthMonitor::startMonitoring()
  {
    // Spin executor
  }

  void LifecyleManager::HealthMonitor::healthCallback(const typename mrp_common_msgs::msg::Heartbeat::SharedPtr msg)
  {
    std::lock_guard<std::recursive_mutex> lck_guard(status_mutex_);
    if ((last_monitored_time_ - msg->stamp.nanosec) / 1e6 < heartbeat_interval_ms_)
    {
      status_ = NodeStatus::HEALTHY;
      return;
    }
    status_ = NodeStatus::UNHEALTHY;
    return;
  }

  LifecyleManager::HealthMonitor::NodeStatus LifecyleManager::HealthMonitor::getStatus()
  {
    std::lock_guard<std::recursive_mutex> lck_guard(status_mutex_);
    return status_;
  }
} // namespace mrp_lifecycle_manager
