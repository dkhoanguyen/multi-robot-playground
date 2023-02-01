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
    member_state_topic_name_ = robot_name_ + "/member_state";
    follow_path_action_name_ = robot_name_ + "/follow_path";

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
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating follow path action client for robot " + robot_name_);
    try
    {
      follow_path_client_ = std::make_shared<mrp_common::ActionClient<nav2_msgs::action::FollowPath>>(
          shared_from_this(),
          follow_path_action_name_,
          std::bind(&TaskExecutorServer::followPathFeedbackCallback, this, std::placeholders::_1), // Feedback callback
          nullptr,                                                                                 // Result callback
          false);

      mrp_common::Log::basicInfo(
          get_node_logging_interface(),
          "follow_path action client for " + robot_name_ + " created successfully!");
    }
    catch (const std::exception &e) // Maybe it's better not to catch all exception here
    {
      mrp_common::Log::basicError(
          get_node_logging_interface(),
          "Unable to create follow_path action server for " + robot_name_ + " due to err " + std::string(e.what()));
    }
  }

  void TaskExecutorServer::createExecuteTaskServer()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating execute task action server for robot " + robot_name_);
    try
    {
      execute_task_server_ = std::make_shared<mrp_common::ActionServer<mrp_task_msgs::action::ExecuteTask>>(
          shared_from_this(),
          "follow_path",
          std::bind(&TaskExecutorServer::executeTask, this),
          nullptr,
          std::chrono::milliseconds(100), // Execution frequency
          false,
          rcl_action_server_get_default_options());
    }
    catch (const std::exception &e) // Maybe it's better not to catch all exception here
    {
      mrp_common::Log::basicError(
          get_node_logging_interface(),
          "Unable to create execute task action server for " + robot_name_ + " due to err " + std::string(e.what()));
    }
  }

  void TaskExecutorServer::followPathFeedbackCallback(const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback)
  {
  }

  void TaskExecutorServer::followPathResultCallback(const std::shared_ptr<const nav2_msgs::action::FollowPath::Result> result)
  {
  }

  void TaskExecutorServer::executeTask()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Begin executing assigned task");
    try
    {
      // Start behaviour tree
      // Get task
      mrp_task_msgs::msg::Task current_task = execute_task_server_->getCurrentGoal()->task;

      // Get destination from task
      geometry_msgs::msg::PoseStamped destination = current_task.location;

      // Request the motion planner server to move to the destination
      nav2_msgs::action::FollowPath::Goal destination_req = nav2_msgs::action::FollowPath::Goal();
      destination_req.path.poses = std::vector<geometry_msgs::msg::PoseStamped>({destination});

      follow_path_client_->sendGoal(destination_req);
      while(rclcpp::ok())
      {
        // Wait until destination is reached 
      }
      
    }
    catch (const std::exception &e)
    {
      // Catch error here
    }
  }
}
