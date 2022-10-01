#ifndef MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_ACTION_SERVER_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_robot_playground_common/action_server.hpp"
#include "multi_robot_playground_common/logging.hpp"

#include "turtlesim/action/rotate_absolute.hpp"

namespace mrp_component_testing
{
  class MinimalActionServer : public mrp_common::ActionServer<turtlesim::action::RotateAbsolute>
  {
  public:
    MinimalActionServer(rclcpp::Node::SharedPtr &node,
                        std::string action_name);
    ~MinimalActionServer();

    void executionCallback();
    void completionCallback();

  protected:
    double server_frequency_;
    // rclcpp::WallRate loop_rate_;
  };
} // namespace mrp_common

#endif
