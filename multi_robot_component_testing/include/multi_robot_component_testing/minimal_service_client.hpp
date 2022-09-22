#ifndef MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_CLIENT_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_CLIENT_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "multi_robot_playground_common/service_client.hpp"

namespace mrp_component_testing
{
  class MinimalServiceClient : public mrp_common::ServiceClient<std_srvs::srv::SetBool>
  {
  public:
    MinimalServiceClient(rclcpp::Node::SharedPtr node,
                         std::string service_name = "minimal_service");
    ~MinimalServiceClient();
  };
}

#endif