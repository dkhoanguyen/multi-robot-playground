#include "multi_robot_lifecycle_manager/lifecyle_manager.hpp"

namespace mrp_lifecycle_manager
{

  LifecyleManager::LifecyleManager(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("mrp_lifecycle_manager", options),
        qos_profile_(10)
  {
  }

  LifecyleManager::~LifecyleManager()
  {
  }

  bool LifecyleManager::registerWatchdog(const std::string &node_name)
  {
    qos_profile_
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
        .liveliness_lease_duration(lease_duration_);
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
        // Transition lifecycle to deactivated state
        // deactivate();
      }
    };

    heartbeat_sub_ = create_subscription<mrp_common_msgs::msg::Heartbeat>(
        node_name + "/hearbeat",
        qos_profile_,
        [this](const typename mrp_common_msgs::msg::Heartbeat::SharedPtr msg) -> void
        {
          RCLCPP_INFO(get_logger(), "Watchdog raised, heartbeat sent at [%d.x]", msg->stamp.sec);
        },
        heartbeat_sub_options_);

    return true;
  }

  LifecyleManager::Watchdog::Watchdog()
  {
  }

  LifecyleManager::Watchdog::~Watchdog()
  {
  }
} // namespace multi_robot_lifecycle_manager
