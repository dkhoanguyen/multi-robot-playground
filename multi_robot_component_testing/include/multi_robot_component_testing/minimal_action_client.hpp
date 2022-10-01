#ifndef MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_ACTION_SERVER_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_robot_playground_common/action_client.hpp"
#include "multi_robot_playground_common/logging.hpp"

#include "turtlesim/action/rotate_absolute.hpp"

namespace mrp_component_testing
{
  class MinimalActionClient : public mrp_common::ActionClient<turtlesim::action::RotateAbsolute>
  {
  public:
    MinimalActionClient(rclcpp::Node::SharedPtr &node,
                        std::string action_name);
    ~MinimalActionClient();
  };
} // namespace name

#endif