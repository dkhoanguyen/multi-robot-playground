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

#include "logging.hpp"

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

    explicit ActionServer(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
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
          completion_callback_(completion_callback),
          server_active_(false),
          stop_execution_(false),
          preempt_requested_(false);
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
              action_name, execute_callback, completion_callback)
    {
    }

    virtual ~ActionServer()
    {
    }

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const typename ActionType::Goal> goal)
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      if (!server_active_)
      {
        MRPLogging::basicWarn("Received goal but server is inactive, so reject this goal.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      MRPLogging::basicInfo("Received request for goal acceptance.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle)
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      if (!handle->is_active())
      {
        MRPLogging::basicWarn("Received request for goal cancellation,"
                              "but the handle is inactive, so reject the request");
        return rclcpp_action::CancelResponse::REJECT;
      }
      MRPLogging::basicInfo("Received request for goal cancellation");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle)
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      MRPLogging::basicInfo("Receiving a new goal");

      if (isActive(current_handle_) || isRunning())
      {
        MRPLogging::basicInfo("An older goal is active, moving the new goal to a pending slot.");
        if (isActive(pending_handle_))
        {
          MRPLogging::basicInfo(
              "The pending slot is occupied."
              " The previous pending goal will be terminated and replaced.");
          terminate(pending_handle_);
        }
        pending_handle_ = handle;
        preempt_requested_ = true;
      }
      else
      {
        if (isActive(pending_handle_))
        {
          // Shouldn't reach a state with a pending goal but no current one.
          MRPLogging::basicError("Forgot to handle a preemption. Terminating the pending goal.");
          terminate(pending_handle_);
          preempt_requested_ = false;
        }

        current_handle_ = handle;
        MRPLogging::basicInfo("Executing goal asynchronously.");
        execution_future_ = std::async(std::launch::async, [this]()
                                       { execute(); });
      }
    }

    void execute()
    {
      while (rclcpp::ok() && !stop_execution_ && isActive(current_handle_))
      {
        MRPLogging::basicInfo("Executing the goal...");
        try
        {
          execute_callback_();
        }
        catch(const std::exception& e)
        {
          completion_callback_();
          return;
        }
        
      }
    }

    void activate()
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = true;
    }

    void deactivate()
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = false;
    }

    bool isRunning()
    {
      return execution_future_.valid() &&
             (execution_future_.wait_for(std::chrono::milliseconds(0)) ==
              std::future_status::timeout);
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
    std::string action_name_;

    typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;

    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;
    std::future<void> execution_future_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> current_handle_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> pending_handle_;

    mutable std::recursive_mutex update_mutex_;
    bool server_active_;
    bool stop_execution_;
    bool preempt_requested_;
    std::chrono::milliseconds server_timeout_;

    constexpr bool isActive(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle) const
    {
      return handle != nullptr && handle->is_active();
    }

    void terminate(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle,
                   typename std::shared_ptr<typename ActionType::Result> result =
                       std::make_shared<typename ActionType::Result>())
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);

      if (isActive(handle))
      {
        if (handle->is_canceling())
        {
          MRPLogging::basicWarn("Client requested to cancel the goal. Cancelling.");
          handle->canceled(result);
        }
        else
        {
          MRPLogging::basicWarn("Aborting handle.");
          handle->abort(result);
        }
        handle.reset();
      }
    }
  };
}

#endif