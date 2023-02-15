#ifndef MRP_BEHAVIOR_TREE__TEST__TEST_ACTION_SERVER_HPP_
#define MRP_BEHAVIOR_TREE__TEST__TEST_ACTION_SERVER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

template <class ActionT>
class TestActionServer : public rclcpp::Node
{
public:
  explicit TestActionServer(
      std::string action_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("test_action_server", options)
  {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<ActionT>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        action_name,
        std::bind(&TestActionServer::handleGoal, this, _1, _2),
        std::bind(&TestActionServer::handleCancel, this, _1),
        std::bind(&TestActionServer::handleAccepted, this, _1));
  };

  std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return current_goal_;
  }

protected:
  virtual rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const typename ActionT::Goal> goal)
  {
    current_goal_ = goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  virtual rclcpp_action::CancelResponse handleCancel(
      const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void execute(
      const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) = 0;

  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    using namespace std::placeholders; // NOLINT
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TestActionServer::execute, this, _1), goal_handle}.detach();
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  std::shared_ptr<const typename ActionT::Goal> current_goal_;
};

#endif