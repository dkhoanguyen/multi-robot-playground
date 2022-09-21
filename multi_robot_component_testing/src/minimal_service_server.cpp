#include "multi_robot_component_testing/minimal_service_server.hpp"

namespace mrp_component_testing
{
  MinimalServiceServer::MinimalServiceServer(rclcpp::Node::SharedPtr node, std::string service_name)
      : ServiceServer<std_srvs::srv::SetBool>(node, service_name, rcl_service_get_default_options())
  {
    // using namespace std::placeholders;
    // service_server_ = rclcpp::create_service<std_srvs::srv::SetBool>(
    //     node_base_interface_,
    //     node_services_interface_,
    //     service_name_,
    //     std::bind(&MinimalServiceServer::execute, this, _1, _2),
    //     rmw_qos_profile_default,
    //     callback_group_);
  }

  MinimalServiceServer::~MinimalServiceServer()
  {
  }

  void MinimalServiceServer::execute(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                     std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::cout << request->data << std::endl;
    response->success = true;
    response->message = "Hello";
  }
}