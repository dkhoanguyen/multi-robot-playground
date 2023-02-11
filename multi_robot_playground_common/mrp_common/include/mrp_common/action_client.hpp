#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_CLIENT_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_CLIENT_HPP_

#include <chrono> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "logging.hpp"
#include "utils.hpp"

namespace mrp_common
{
  using namespace std::chrono_literals; // NOLINT

  template <typename ActionType>
  class ActionClient
  {
  public:
    typedef std::function<void(const std::shared_ptr<const typename ActionType::Feedback>)> FeedbackCallback;
    typedef std::function<void(const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult)> ResultCallback;

    template <typename NodeSharedPtrType>
    ActionClient(NodeSharedPtrType node,
                 const std::string &action_name,
                 FeedbackCallback feedback_callback,
                 ResultCallback result_callback,
                 bool spin_thread)
        : node_base_interface_(node->get_node_base_interface()),
          node_logging_interface_(node->get_node_logging_interface()),
          action_name_(action_name),
          spin_thread_(spin_thread),
          feedback_callback_(feedback_callback),
          result_callback_(result_callback)
    {
      if (node)
      {
        action_client_ = rclcpp_action::create_client<ActionType>(
            node->get_node_base_interface(),
            node->get_node_graph_interface(),
            node->get_node_logging_interface(),
            node->get_node_waitables_interface(),
            action_name_,
            nullptr);
      }
      else
      {
        node_ = ROSUtils::generateInternalNode(action_name + "_action_client_Node");
        action_client_ = rclcpp_action::create_client<ActionType>(
            node_,
            action_name);

        node_base_interface_ = node->get_node_base_interface();
        node_logging_interface_ = node->get_node_logging_interface();
      }
    }

    ActionClient(const std::string parent_name,
                 const std::string action_name)
        : action_name_(action_name)
    {
      node_ = ROSUtils::generateInternalNode(
          parent_name + std::string("_") + action_name + "_action_client");
      action_client_ = rclcpp_action::create_client<ActionType>(
          node_,
          action_name);
    }

    virtual ~ActionClient()
    {
      if (spin_thread_)
      {
        // Stop executor
        callback_group_executor_->cancel();
      }
    }

    void waitForServer()
    {
      if (!action_client_->wait_for_action_server(1s))
      {
        Log::basicError(
            node_logging_interface_,
            "Action server for action " + action_name_ + " not available after waiting");
      }
    }

    bool sendGoal(
        const typename ActionType::Goal &goal,
        const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max())
    {
      Log::basicInfo(node_logging_interface_, "Sending goal");
      goal_result_available_ = false;

      auto send_goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();

      send_goal_options.goal_response_callback =
          std::bind(&ActionClient::defaultGoalResponseCallback, this, std::placeholders::_1);
      send_goal_options.feedback_callback =
          std::bind(&ActionClient::defaultFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
          std::bind(&ActionClient::defaultResultCallback, this, std::placeholders::_1);
      auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);

      if (rclcpp::spin_until_future_complete(node_base_interface_, future_goal_handle) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        Log::basicError(
            node_logging_interface_,
            action_name_ + " action client: async_send_goal failed");
        return false;
      }
      current_handle_ = future_goal_handle.get();

      return true;
    }

    bool waitForResult()
    {
      auto future_result = action_client_->async_get_result(current_handle_);
      if (rclcpp::spin_until_future_complete(node_base_interface_, future_result) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        Log::basicError(
            node_logging_interface_,
            action_name_ + " action client: async_get_result failed");
        return false;
      }

      result_ = future_result.get();
      return true;
    }

    bool waitAndCheckForResult()
    {
      rclcpp::spin_some(node_base_interface_);
      if(!goal_result_available_)
      {
        return false;
      }
      return true;
    }

    bool cancelCurrentGoal()
    {
      if (!shouldCancelGoal())
      {
        return false;
      }
      auto future_goal_cancel = action_client_->async_cancel_goal(current_handle_);

      if (rclcpp::spin_until_future_complete(node_base_interface_, future_goal_cancel) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        Log::basicError(
            node_logging_interface_,
            action_name_ + " action client: async_cancel_goal failed");
        return false;
      }

      auto cancel_status = current_handle_.get()->get_status();
      if (cancel_status != rclcpp_action::GoalStatus::STATUS_CANCELING)
      {
        Log::basicError(
            node_logging_interface_,
            action_name_ + " action client: Goal is not cancelled by server");
        return false;
      }
      return true;
    }

    const rclcpp_action::GoalUUID
    getGoalID() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(client_mutex_);
      return current_handle_->get_goal();
    }

    const std::shared_ptr<const typename ActionType::Feedback> getFeedback() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(client_mutex_);
      return feedback_;
    }

    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult
    getResult() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(client_mutex_);
      return result_;
    }

    const typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr getGoalHandle() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(client_mutex_);
      return current_handle_;
    }

  protected:
    rclcpp::Node::SharedPtr node_; // Internal node associated with this action client

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    std::string action_name_;

    typename rclcpp_action::Client<ActionType>::SharedPtr action_client_;
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr current_handle_;

    std::shared_ptr<typename ActionType::Feedback> feedback_;
    typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult result_;

    std::future<void> spin_future_;
    FeedbackCallback feedback_callback_;
    ResultCallback result_callback_;

    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;

    mutable std::recursive_mutex client_mutex_;
    bool spin_thread_;

    std::atomic<bool> goal_result_available_{false};

    bool shouldCancelGoal()
    {
      rclcpp::spin_some(node_base_interface_);
      auto status = current_handle_->get_status();

      // Check if the goal is still executing
      return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
             status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    }

    void defaultGoalResponseCallback(
        std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr> goal_future)
    {
      auto goal_handle = goal_future.get();
      if (!goal_handle)
      {
        Log::basicError(node_logging_interface_, "Goal was rejected by server");
      }
      else
      {
        Log::basicInfo(node_logging_interface_, "Goal accepted by server, waiting for result");
      }
    }

    void defaultFeedbackCallback(typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr handle,
                                 const std::shared_ptr<const typename ActionType::Feedback> feedback)
    {
      Log::basicInfo(node_logging_interface_, "Receiving feedback from server");
      try
      {
        feedback_callback_(feedback);
      }
      catch (const std::exception &ex)
      {
        if (feedback_callback_ != nullptr)
        {
          Log::basicWarn(node_logging_interface_, "Failed to execute custom feedback callback");
        }
        else
        {
          Log::basicDebug(node_logging_interface_, "Feedback callback is null");
        }
      }
    }

    void defaultResultCallback(
        const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult &result)
    {
      Log::basicInfo(node_logging_interface_, "Received result from server");
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        Log::basicInfo(node_logging_interface_, "Goal was successfully completed");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        Log::basicError(node_logging_interface_, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        Log::basicError(node_logging_interface_, "Goal was canceled");
        break;
      default:
        Log::basicError(node_logging_interface_, "Unknown result code");
        break;
      }
      try
      {
        result_callback_(result);
      }
      catch (const std::exception &e)
      {
        if (result_callback_ != nullptr)
        {
          Log::basicWarn(node_logging_interface_, "Failed to execute custom result callback");
        }
        else
        {
          Log::basicDebug(node_logging_interface_, "Result callback is null");
        }
      }
      result_ = result;
      goal_result_available_ = true;
    }
  };
}

#endif