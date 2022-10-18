#include "mrp_lifecycle_manager/lifecyle_manager.hpp"

namespace mrp_lifecycle_manager
{
  // Aliases for ease of use
  using LifecycleTransition = mrp_common::LifecycleNode::Transition;
  using LifecycleState = mrp_common::LifecycleNode::State;

  using namespace std::chrono_literals;

  LifecycleManager::LifecycleManager(const rclcpp::NodeOptions &options,
                                     std::chrono::milliseconds heartbeat_timeout)
      : rclcpp_lifecycle::LifecycleNode("lifecycle_manager", options),
        heartbeat_timeout_(heartbeat_timeout)
  {
    // Main normal state
    transition_state_map_[LifecycleTransition::CREATE] = LifecycleState::PRIMARY_STATE_UNCONFIGURED;
    transition_state_map_[LifecycleTransition::CONFIGURE] = LifecycleState::PRIMARY_STATE_INACTIVE;
    transition_state_map_[LifecycleTransition::ACTIVATE] = LifecycleState::PRIMARY_STATE_ACTIVE;
    transition_state_map_[LifecycleTransition::DEACTIVATE] = LifecycleState::PRIMARY_STATE_INACTIVE;
    transition_state_map_[LifecycleTransition::ACTIVE_SHUTDOWN] = LifecycleState::PRIMARY_STATE_UNKNOWN;

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(get_node_base_interface(), false);
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
          false,
          executor_,
          get_node_topics_interface(),
          get_node_base_interface(),
          get_node_timers_interface(),
          get_node_clock_interface());

      // Create lifecycle_manager_client for requesting state changes
      monitored_node.lifecyle_manager_client_ = std::make_shared<LifecycleManagerClient>(
          shared_from_this(),
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

  LifecycleManager::TransitionRequestStatus LifecycleManager::transitionNode(const std::string &node_name,
                                                                             LifecycleTransition transition,
                                                                             std::chrono::nanoseconds timeout)
  {
    if (!monitored_node_map_[node_name].lifecyle_manager_client_->requestTransition(transition, timeout))
    {
      // What should we do here when it fails to set state transition ?
      return TransitionRequestStatus::CANNOT_CHANGE_STATE;
    }
    // Compare with the desired target state
    // Since the timeout has be incorporate in the service call, let's just check for the state at
    // this point
    if (getNodeState(node_name, timeout) != transition_state_map_[transition])
    {
      return TransitionRequestStatus::WRONG_END_STATE;
    }
    return TransitionRequestStatus::NO_ERROR;
  }

  bool LifecycleManager::transitionNodes(const std::vector<std::string> &monitored_node_names,
                                         LifecycleTransition transition,
                                         std::chrono::nanoseconds timeout)
  {
    for (const std::string &node_name : monitored_node_names)
    {
      if (transitionNode(node_name, transition, timeout) != TransitionRequestStatus::NO_ERROR)
      {
        return false;
      }
    }

    return true;
  }

  mrp_common::LifecycleNode::State LifecycleManager::getNodeState(const std::string &node_name,
                                                                  std::chrono::nanoseconds timeout)
  {
    return monitored_node_map_[node_name].lifecyle_manager_client_->getNodeState(timeout);
  }

  bool LifecycleManager::configureNodes(const std::vector<std::string> &monitored_node_names,
                                        std::chrono::nanoseconds timeout)
  {
    return transitionNodes(monitored_node_names,
                           LifecycleTransition::CONFIGURE, timeout);
  }
  bool LifecycleManager::activateNodes(const std::vector<std::string> &monitored_node_names,
                                       std::chrono::nanoseconds timeout)
  {
    return transitionNodes(monitored_node_names,
                           LifecycleTransition::ACTIVATE, timeout);
  }
  bool LifecycleManager::pauseNodes(const std::vector<std::string> &monitored_node_names,
                                    std::chrono::nanoseconds timeout)
  {
    return transitionNodes(monitored_node_names,
                           LifecycleTransition::CONFIGURE, timeout);
  }
  bool LifecycleManager::resumeNodes(const std::vector<std::string> &monitored_node_names,
                                     std::chrono::nanoseconds timeout)
  {
    return transitionNodes(monitored_node_names,
                           LifecycleTransition::CONFIGURE, timeout);
  }
  bool LifecycleManager::shutdownNodes(const std::vector<std::string> &monitored_node_names,
                                       std::chrono::nanoseconds timeout)
  {
    return transitionNodes(monitored_node_names,
                           LifecycleTransition::CONFIGURE, timeout);
  }

  // LIFECYCLE NODE STATE TRANSITION CALLBACK
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_configure(const rclcpp_lifecycle::State &state)
  {
    // Request params from local params server
    mrp_common::Log::basicInfo(get_node_logging_interface(),
                               "Configuring lifecycle manager");

    // Get all parameters
    loadParameters();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_activate(const rclcpp_lifecycle::State &state)
  {
    // Create wall timer for health monitoring
    if (!createMonitorTimer())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_cleanup(const rclcpp_lifecycle::State &state)
  {
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_error(const rclcpp_lifecycle::State &state)
  {
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_shutdown(const rclcpp_lifecycle::State &state)
  {
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_exit(const rclcpp_lifecycle::State &state)
  {
  }

  void LifecycleManager::monitorNodes()
  {
    for (auto const &monitored_node : monitored_node_map_)
    {
      if (!rclcpp::ok())
      {
        return;
      }
      // Check heartbeat if healthy or not
      if (monitored_node.second.heartbeat_ptr_->getStatus() != HealthMonitor::NodeHeartbeat::HEALTHY)
      {
        // It is not healthy at all what should we do
        mrp_common::Log::basicError(get_node_logging_interface(),
                                    std::string("Have not received a heartbeat from " + monitored_node.first + "."));
      }
    }
  }

  void LifecycleManager::loadParameters()
  {
    declare_parameter<std::vector<std::string>>("node_names", std::vector<std::string>());
    declare_parameter<bool>("autostart", false);
    declare_parameter<std::vector<double>>("heartbat_interval", std::vector<double>());
    declare_parameter<double>("bond_respawn_max_duration", 10.0);
    declare_parameter<bool>("attempt_respawn_reconnection", true);
  }

  bool LifecycleManager::createMonitorTimer()
  {
    // Destroy the old timer
    destroyMonitorTimer();
    // Load parameters
    loadParameters();
    // Get necessary parameters
    std::vector<std::string> node_names = get_parameter("node_names").as_string_array();
    bool auto_start = false;
    get_parameter("autostart", auto_start);
    std::vector<double> heartbeat_interval;
    get_parameter("heartbat_interval", heartbeat_interval);

    // Convert raw duration in double to chrono duration
    std::vector<std::chrono::milliseconds> converted_heartbeat;
    for (const auto heartbeat : heartbeat_interval)
    {
      converted_heartbeat.push_back(std::chrono::duration<int64_t, std::milli>((int64_t)heartbeat));
    }

    if (converted_heartbeat.size() != node_names.size())
    {
      mrp_common::Log::basicError(get_node_logging_interface(),
                                  "Unable to register monitor timer due to: \
                                        Mismatch number of nodes and heartbeat values. \
                                        Exitting...");
      return false;
    }
    // Register lifecycle nodes
    if (!registerLifecycleNodes(node_names, converted_heartbeat))
    {
      mrp_common::Log::basicError(get_node_logging_interface(),
                                  "Unable to register monitor timer");
      return false;
    }

    // Start executor beyond this point

    // Let's monitor once every 1s
    monitor_timer_ = create_wall_timer(
        1s,
        [this]() -> void
        {
          // Start monitoring
          monitorNodes();
        },
        callback_group_);
    return true;
  }

  bool LifecycleManager::destroyMonitorTimer()
  {
    if (monitor_timer_)
    {
      // mrp_common::Log::basicInfo("Terminating timer...");
      monitor_timer_->cancel();
      monitor_timer_.reset();
    }
    return true;
  }

  //=================================================//
  //                                                 //
  // ================================================//

  LifecycleManager::HealthMonitor::HealthMonitor(
      const std::string node_name,
      std::chrono::milliseconds heartbeat_timeout,
      std::chrono::milliseconds heartbeat_interval,
      bool isolated_spin,
      rclcpp::executors::SingleThreadedExecutor::SharedPtr executor,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface)
      : node_name_(node_name),
        heartbeat_timeout_(heartbeat_timeout),
        heartbeat_interval_(heartbeat_interval),
        isolated_spin_(isolated_spin),
        executor_(executor),
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
    // Spin a default executor if isolate spin
    if (isolated_spin_)
    {
      return;
    }
    // Attach
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
