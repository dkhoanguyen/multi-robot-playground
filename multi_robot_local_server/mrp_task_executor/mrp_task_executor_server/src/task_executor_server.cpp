#include "mrp_task_executor_server/task_executor_server.hpp"

namespace mrp_task_executor
{
  TaskExecutorServer::TaskExecutorServer()
      : mrp_common::LifecycleNode(
            "task_executor_server",
            "robot", true, true, std::chrono::milliseconds(1000))
  {
  }

  TaskExecutorServer::~TaskExecutorServer()
  {
  }

  bool TaskExecutorServer::initialise()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Configuring task_executor_server...");
    mrp_common::Log::basicInfo(
        get_node_logging_interface(), "Extracting parameters from parameter server");

    robot_name_ = get_namespace();
    robot_cmd_vel_topic_name_ = robot_name_ + "/cmd_vel";
    robot_odom_topic_name_ = robot_name_ + "/odom";
    robot_scan_topic_name_ = robot_name_ + "/scan";
    member_state_topic_name_ = robot_name_ + "/member_state";

    // Initialise heartbeat
    initialiseHeartbeat();

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "task_executor_server initialises successfully");

    return true;
  }

  bool TaskExecutorServer::start()
  {
    // Start beating
    startHeartbeat();
    return true;
  }

  bool TaskExecutorServer::stop()
  {
    // Stop heartbeat
    stopHeartbeat();
    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  TaskExecutorServer::on_configure(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Initialising robot comms server");
    if (!initialise())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  TaskExecutorServer::on_activate(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Activating robot comms server");
    if (!start())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  TaskExecutorServer::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Deactivating robot comms server");
    if (!stop())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  TaskExecutorServer::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Cleaning up robot comms server");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  TaskExecutorServer::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Shutting down robot comms server");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void TaskExecutorServer::createMemberStatePublisher()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating member state publisher for robot " + robot_name_);
    member_state_pub_ = create_publisher<mrp_comms_msgs::msg::MemberState>(member_state_topic_name_, 10);
  }

  void TaskExecutorServer::createFollowPathActionClient()
  {
  }
}
