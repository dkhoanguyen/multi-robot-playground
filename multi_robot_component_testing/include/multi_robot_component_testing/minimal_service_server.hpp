#ifndef MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_SERVER_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_SERVER_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "multi_robot_playground_common/service_server.hpp"

namespace mrp_component_testing
{
  class MinimalServiceServer : public mrp_common::ServiceServer<std_srvs::srv::SetBool>
  {
  public:
    MinimalServiceServer(rclcpp::Node::SharedPtr node,
                         std::string service_name = "minimal_service");
    ~MinimalServiceServer();

    void handleRequest(std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> &response);

    void execute(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  };
}

#endif