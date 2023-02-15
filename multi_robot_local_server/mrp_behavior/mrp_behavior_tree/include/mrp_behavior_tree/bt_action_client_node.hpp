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
      // This might not be  a good practice to do
      // Derived classes should handle the initialisation of Goal and Result
      goal_ = typename ActionType::Goal();
      result_ = typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult();

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
      bool spin_thread = false;
      action_client_ = std::make_shared<mrp_common::ActionClient<ActionType>>(
          node_,
          action_name,
          std::bind(&BtActionNode::onWaitForResult, this, std::placeholders::_1),
          std::bind(&BtActionNode::onReceiveResult, this, std::placeholders::_1),
          false);
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
    virtual void onWaitForResult(std::shared_ptr<const typename ActionType::Feedback> feedback)
    {
      std::cout << "Base BTActionNode" << std::endl;
    }

    virtual void onReceiveResult(typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult result)
    {
      std::cout << "Base BTActionNode" << std::endl;
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

        // Send goal
        action_client_->sendGoal(goal_);
      }

      // The following code corresponds to the "RUNNING" loop
      if (rclcpp::ok() && !action_client_->waitAndCheckForResult())
      {
        goal_handle_ = action_client_->getGoalHandle();
        auto goal_status = goal_handle_->get_status();

        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                              goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          action_client_->sendGoal(goal_);
        }

        // check if, after invoking waitAndCheckForResult(), we finally received the result
        if (!action_client_->waitAndCheckForResult())
        {
          // Yield this Action, returning RUNNING
          return BT::NodeStatus::RUNNING;
        }
      }

      std::lock_guard<std::recursive_mutex> lck_guard(result_mutex_);
      // Get result
      auto result = action_client_->getResult();
      switch (result.code)
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
      // Cancel current goal
      action_client_->cancelCurrentGoal();
      setStatus(BT::NodeStatus::IDLE);
    }

  protected:
    std::string action_name_;
    typename std::shared_ptr<mrp_common::ActionClient<ActionType>> action_client_;

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

    mutable std::recursive_mutex result_mutex_;

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