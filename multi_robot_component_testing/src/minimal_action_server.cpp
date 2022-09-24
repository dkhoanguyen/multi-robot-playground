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

    double counter = 0;
    double target = (double)current_goal->theta;

    rclcpp::WallRate loop_rate(server_frequency_);
    while (rclcpp::ok())
    {
      if(current_handle_->is_canceling())
      {
        
      }
      if(counter < target)
      {
        counter++;
        feedback->remaining = target - counter;
        current_handle_->publish_feedback(feedback);
      }
      else
      {
        result->delta = target - counter;
        current_handle_->succeed(result);
      }

      if (!loop_rate.sleep())
      {
      }
    }
  }

  void MinimalActionServer::completionCallback()
  {
  }
}