#ifndef MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_SERVER_HPP_
#define MULTI_ROBOT_COMPONENT_TESTING__MINIMAL_SERVICE_SERVER_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "mrp_common/service_server.hpp"

#include "mrp_comms_msgs/srv/get_members_in_team.hpp"

namespace mrp_component_testing
{
  class MinimalServiceServer : public mrp_common::ServiceServer<mrp_comms_msgs::srv::GetMembersInTeam>
  {
  public:
    MinimalServiceServer(rclcpp::Node::SharedPtr node,
                         std::string service_name = "/robot/get_members_from_team");
    ~MinimalServiceServer();

    void execute(std::shared_ptr<mrp_comms_msgs::srv::GetMembersInTeam::Request> &request,
                 std::shared_ptr<mrp_comms_msgs::srv::GetMembersInTeam::Response> &response);
  };
}

#endif