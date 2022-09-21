#include "multi_robot_playground_common/action_server.hpp"

namespace mrp_common
{

  template <typename ActionType>
  ActionServer<ActionType>::ActionServer(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
                                         rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
                                         rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
                                         rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
                                         const std::string &action_name,
                                         ExecuteCallback execute_callback,
                                         CompletionCallback completion_callback,
                                         std::chrono::milliseconds server_timeout,
                                         bool spin_thread,
                                         const rcl_action_server_options_t &options)
      : node_base_interface_(node_base_interface),
        node_clock_interface_(node_clock_interface),
        node_logging_interface_(node_logging_interface),
        node_waitables_interface_(node_waitables_interface),
        action_name_(action_name),
        execute_callback_(execute_callback),
        completion_callback_(completion_callback)
  {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<ActionType>(
        node_base_interface_,
        node_clock_interface_,
        node_logging_interface_,
        node_waitables_interface_,
        action_name_,
        std::bind(&ActionServer::handleGoal, this, _1, _2),
        std::bind(&ActionServer::handleCancel, this, _1),
        std::bind(&ActionServer::handleAccepted, this, _1),
        options);
  }

  template <typename ActionType>
  rclcpp_action::GoalResponse ActionServer<ActionType>::handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const typename ActionType::Goal> goal)
  {
  }

  template <typename ActionType>
  rclcpp_action::CancelResponse ActionServer<ActionType>::handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle)
  {
  }

  template <typename ActionType>
  void ActionServer<ActionType>::handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle)
  {
  }

  template <typename ActionType>
  void ActionServer<ActionType>::execute()
  {
  }
}