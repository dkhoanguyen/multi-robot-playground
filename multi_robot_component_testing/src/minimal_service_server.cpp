#include "multi_robot_component_testing/minimal_service_server.hpp"

namespace mrp_component_testing
{
  MinimalServiceServer::MinimalServiceServer(rclcpp::Node::SharedPtr node, std::string service_name)
      : ServiceServer<std_srvs::srv::SetBool>(node, service_name, nullptr, false, rcl_service_get_default_options())
  {
  }

  MinimalServiceServer::~MinimalServiceServer()
  {
  }

  void MinimalServiceServer::execute(std::shared_ptr<std_srvs::srv::SetBool::Request> &request,
                                     std::shared_ptr<std_srvs::srv::SetBool::Response> &response)
  {
    std::cout << "MinimalServiceServer" << std::endl;
    std::cout << request->data << std::endl;
    response->success = true;
    response->message = "Hello";
    std::cout << response->message << std::endl;
  }
}