#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_SERVER_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mrp_common
{
  template <typename ActionType>
  class ActionServer
  {
  public:
    // Callback function to complete main work. This should itself deal with its
    // own exceptions, but if for some reason one is thrown, it will be caught
    // in SimpleActionServer and terminate the action itself.
    typedef std::function<void()> ExecuteCallback;

    // Callback function to notify the user that an exception was thrown that
    // the simple action server caught (or another failure) and the action was
    // terminated. To avoid using, catch exceptions in your application such that
    // the SimpleActionServer will never need to terminate based on failed action
    // ExecuteCallback.
    typedef std::function<void()> CompletionCallback;

    template <typename NodeType>
    explicit ActionServer(
        NodeType &node,
        const std::string &action_name,
        ExecuteCallback execute_callback,
        CompletionCallback completion_callback = nullptr,
        const rcl_action_server_options_t &options = rcl_action_server_get_default_options())
        : ActionServer(
              node->get_node_base_interface(),
              node->get_node_clock_interface(),
              node->get_node_logging_interface(),
              node->get_node_waitables_interface(),
              action_name, execute_callback, completion_callback){};

    explicit ActionServer(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
        const std::string &action_name,
        ExecuteCallback execute_callback,
        CompletionCallback completion_callback = nullptr,
        std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
        bool spin_thread = false,
        const rcl_action_server_options_t &options = rcl_action_server_get_default_options());

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const typename ActionType::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle);

    void execute();

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;

    typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;

    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;

    std::string action_name_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle_;
  };
}

#endif