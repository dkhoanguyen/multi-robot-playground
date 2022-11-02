#include "multi_robot_component_testing/minimal_service_server.hpp"

namespace mrp_component_testing
{
  MinimalServiceServer::MinimalServiceServer(rclcpp::Node::SharedPtr node, std::string service_name)
      : ServiceServer<mrp_comms_msgs::srv::GetMembersInTeam>(node, service_name, nullptr, true, rcl_service_get_default_options())
  {
  }

  MinimalServiceServer::~MinimalServiceServer()
  {
  }

  void MinimalServiceServer::execute(std::shared_ptr<mrp_comms_msgs::srv::GetMembersInTeam::Request> &request,
                                     std::shared_ptr<mrp_comms_msgs::srv::GetMembersInTeam::Response> &response)
  {
    std::cout << "MinimalServiceServer" << std::endl;
    std::cout << "Team ID: " << (int)request->team_id << std::endl;
    response->member_name_list = {"robot0", "robot1", "robot2", "robot3"};
    // std::cout << request->data << std::endl;
    // response->success = true;
    // response->message = "Hello";
    // std::cout << response->message << std::endl;
  }
}