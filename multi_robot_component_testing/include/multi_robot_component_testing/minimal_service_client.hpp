#ifndef MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_CLIENT_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_CLIENT_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "mrp_common/service_client.hpp"
#include "mrp_comms_msgs/srv/get_members_in_team.hpp"

namespace mrp_component_testing
{
  class MinimalServiceClient : public mrp_common::ServiceClient<mrp_comms_msgs::srv::GetMembersInTeam>
  {
  public:
    MinimalServiceClient(rclcpp::Node::SharedPtr node,
                         std::string service_name = "minimal_service");
    ~MinimalServiceClient();
  };
}

#endif