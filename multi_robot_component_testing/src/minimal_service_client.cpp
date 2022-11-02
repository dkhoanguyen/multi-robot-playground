#include "multi_robot_component_testing/minimal_service_client.hpp"

namespace mrp_component_testing
{
  MinimalServiceClient::MinimalServiceClient(rclcpp::Node::SharedPtr node, std::string service_name)
      : ServiceClient<mrp_comms_msgs::srv::GetMembersInTeam>(node, false, service_name, rcl_service_get_default_options())
  {
  }

  MinimalServiceClient::~MinimalServiceClient()
  {
  }
}