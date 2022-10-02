#include "multi_robot_component_testing/minimal_action_server.hpp"

namespace mrp_component_testing
{
  MinimalActionServer::MinimalActionServer(rclcpp::Node::SharedPtr &node,
                                           std::string action_name)
      : ActionServer<turtlesim::action::RotateAbsolute>(
            node, action_name,
            std::bind(&MinimalActionServer::executionCallback, this),
            std::bind(&MinimalActionServer::completionCallback, this),
            std::chrono::milliseconds(500),
            true, // Spin executor
            rcl_action_server_get_default_options()),
        server_frequency_(1)
  {
    // server_frequency_ = 10;
  }

  MinimalActionServer::~MinimalActionServer()
  {
  }

  void MinimalActionServer::executionCallback()
  {
    std::shared_ptr<const turtlesim::action::RotateAbsolute::Goal> current_goal =
        current_handle_->get_goal();
    std::shared_ptr<turtlesim::action::RotateAbsolute::Result> result =
        std::make_shared<turtlesim::action::RotateAbsolute::Result>();
    std::shared_ptr<turtlesim::action::RotateAbsolute::Feedback> feedback =
        std::make_shared<turtlesim::action::RotateAbsolute::Feedback>();

    mrp_common::Log::basicInfo(
        node_logging_interface_,
        "MinimalActionServer");

    double counter = 0;
    double target = (double)current_goal->theta;

    std::cout << "Target:" << target << std::endl;

    rclcpp::WallRate loop_rate(server_frequency_);

    while (rclcpp::ok())
    {
      // std::cout << rclcpp::ok() << std::endl;
      std::cout << counter << std::endl;
      if (isCancelRequested())
      {
        terminateCurrent(result);
        return;
      }
      if (counter < target)
      {
        counter++;
        feedback->remaining = target - counter;
        publishFeedback(feedback);
      }
      else
      {
        result->delta = target - counter;
        succeededCurrent();
        return;
      }

      loop_rate.sleep();
    }
  }

  void MinimalActionServer::completionCallback()
  {
  }
}