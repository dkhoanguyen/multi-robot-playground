#include "multi_robot_component_testing/minimal_action_client.hpp"

namespace mrp_component_testing
{
  MinimalActionClient::MinimalActionClient(rclcpp::Node::SharedPtr &node,
                                           std::string action_name)
      : ActionClient<turtlesim::action::RotateAbsolute>(node, action_name, 
                                                        nullptr, nullptr, true)
  {
  }
  MinimalActionClient::~MinimalActionClient()
  {
    
  }
}