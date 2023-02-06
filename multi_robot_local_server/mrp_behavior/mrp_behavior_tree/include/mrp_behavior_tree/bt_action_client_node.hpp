#ifndef MRP_BEHAVIOR_TREE__BT_ACTION_CLIENT_HPP_
#define MRP_BEHAVIOR_TREE__BT_ACTION_CLIENT_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp_action/rclcpp_action.hpp"
#include "mrp_common/action_client.hpp"

namespace mrp_behavior_tree
{

  using namespace std::chrono_literals; // NOLINT

  template <class ActionType>
  class BtActionNode : public BT::ActionNodeBase
  {
  public:
    BtActionNode(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::ActionNodeBase(xml_tag_name, conf),
          action_name_(action_name)
    {
      node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

      // Get the required items from the blackboard
      server_timeout_ =
          config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
      getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

      // Initialize the input and output messages
      goal_ = typename ActionT::Goal();
      result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

      std::string remapped_action_name;
      if (getInput("server_name", remapped_action_name))
      {
        action_name_ = remapped_action_name;
      }
      createActionClient(action_name_);

      // Give the derive class a chance to do any initialization
      RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
    };

    virtual ~BtActionNode(){};

    void createActionClient(const std::string &action_name)
    {
      action_client_ = std::make_shared<mrp_common::ActionClient<typename ActionType>>(
          node_,
          action_name,
          false,
          nullptr,
          nullptr);

      action_client_->waitForServer();
    };

    /**
     * @brief Any subclass of BtActionNode that accepts parameters must provide a
     * providedPorts method and call providedBasicPorts in it.
     * @param addition Additional ports to add to BT port list
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
      BT::PortsList basic = {
          BT::InputPort<std::string>("server_name", "Action server name"),
          BT::InputPort<std::chrono::milliseconds>("server_timeout")};
      basic.insert(addition.begin(), addition.end());

      return basic;
    }

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({});
    }

    // Derived classes can override any of the following methods to hook into the
    // processing for the action: onTick, onWaitForResult, and onSuccess

    /**
     * @brief Function to perform some user-defined operation on tick
     * Could do dynamic checks, such as getting updates to values on the blackboard
     */
    virtual void onTick()
    {
    }

    /**
     * @brief Function to perform some user-defined operation after a timeout
     * waiting for a result that hasn't been received yet. Also provides access to
     * the latest feedback message from the action server. Feedback will be nullptr
     * in subsequent calls to this function if no new feedback is received while waiting for a result.
     * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
     */
    virtual void onWaitForResult(std::shared_ptr<const typename ActionType::Feedback> /*feedback*/)
    {
    }

    /**
     * @brief Function to perform some user-defined operation upon successful
     * completion of the action. Could put a value on the blackboard.
     * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
     */
    virtual BT::NodeStatus onSuccess()
    {
      return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Function to perform some user-defined operation whe the action is aborted.
     * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
     */
    virtual BT::NodeStatus onAborted()
    {
      return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Function to perform some user-defined operation when the action is cancelled.
     * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
     */
    virtual BT::NodeStatus onCancelled()
    {
      return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override
    {
      // first step to be done only at the beginning of the Action
      if (status() == BT::NodeStatus::IDLE)
      {
        // setting the status to RUNNING to notify the BT Loggers (if any)
        setStatus(BT::NodeStatus::RUNNING);

        // user defined callback
        onTick();

        onNewGoalReceived();
      }

      // The following code corresponds to the "RUNNING" loop
      if (rclcpp::ok() && !goal_result_available_)
      {
        // user defined callback. May modify the value of "goal_updated_"
        onWaitForResult();

        auto goal_status = goal_handle_->get_status();
        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                              goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          onNewGoalReceived();
        }

        rclcpp::spin_some(node_);

        // check if, after invoking spin_some(), we finally received the result
        if (!goal_result_available_)
        {
          // Yield this Action, returning RUNNING
          return BT::NodeStatus::RUNNING;
        }
      }

      switch (result_.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return onSuccess();

      case rclcpp_action::ResultCode::ABORTED:
        return onAborted();

      case rclcpp_action::ResultCode::CANCELED:
        return onCancelled();

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
      }
    }

    // The other (optional) override required by a BT action. In this case, we
    // make sure to cancel the ROS2 action if it is still running.
    void halt() override
    {
      if (shouldCancelGoal())
      {
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        if (rclcpp::spin_until_future_complete(node_, future_cancel, server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(
              node_->get_logger(),
              "Failed to cancel action server for %s", action_name_.c_str());
        }
      }

      setStatus(BT::NodeStatus::IDLE);
    }

  protected:
    std::string action_name_;
    typename mrp_common::ActionClient<ActionType> action_client_;

    // All ROS2 actions have a goal and a result
    typename ActionType::Goal goal_;
    bool goal_updated_{false};
    bool goal_result_available_{false};
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle_;
    typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult result_;

    // The node that will be used for any ROS operations
    rclcpp::Node::SharedPtr node_;

    // The timeout value while waiting for response from a server when a
    // new action goal is sent or canceled
    std::chrono::milliseconds server_timeout_;

    bool shouldCancelGoal()
    {
      // Shut the node down if it is currently running
      if (status() != BT::NodeStatus::RUNNING)
      {
        return false;
      }

      rclcpp::spin_some(node_);
      auto status = goal_handle_->get_status();

      // Check if the goal is still executing
      return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
             status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    }

    void onNewGoalReceived()
    {
      goal_result_available_ = false;
      auto send_goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
      send_goal_options.result_callback =
          [this](const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult &result)
      {
        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        if (this->goal_handle_->get_goal_id() == result.goal_id)
        {
          goal_result_available_ = true;
          result_ = result;
        }
      };

      auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

      if (rclcpp::spin_until_future_complete(node_, future_goal_handle, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        throw std::runtime_error("send_goal failed");
      }

      goal_handle_ = future_goal_handle.get();
      if (!goal_handle_)
      {
        throw std::runtime_error("Goal was rejected by the action server");
      }
    }

    void incrementRecoveryCount()
    {
      int recovery_count = 0;
      config().blackboard->template get<int>("number_recoveries", recovery_count); // NOLINT
      recovery_count += 1;
      config().blackboard->template set<int>("number_recoveries", recovery_count); // NOLINT
    }
  };
} // namespace mrp_behavior_tree

#endif