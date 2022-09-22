#include "multi_robot_component_testing/minimal_service_client.hpp"

namespace mrp_component_testing
{
  MinimalServiceClient::MinimalServiceClient(rclcpp::Node::SharedPtr node, std::string service_name)
      : ServiceClient<std_srvs::srv::SetBool>(node, service_name, rcl_service_get_default_options())
  {
  }

  MinimalServiceClient::~MinimalServiceClient()
  {
  }
}