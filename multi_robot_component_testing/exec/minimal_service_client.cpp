// #include "multi_robot_component_testing/minimal_service_client.hpp"
#include "mrp_common/service_client.hpp"
#include "mrp_comms_msgs/srv/get_members_in_team.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("minimal_service_client");

  std::shared_ptr<mrp_common::ServiceClient<mrp_comms_msgs::srv::GetMembersInTeam>> service_client = 
      std::make_shared<mrp_common::ServiceClient<mrp_comms_msgs::srv::GetMembersInTeam>>(
          node,
          true, // Do not spin as an isolated thread
          "/robot0/get_members_from_team",
          rcl_service_get_default_options());

  
  mrp_comms_msgs::srv::GetMembersInTeam::Request::SharedPtr request = std::make_shared<mrp_comms_msgs::srv::GetMembersInTeam::Request>();
  mrp_comms_msgs::srv::GetMembersInTeam::Response::SharedPtr response = std::make_shared<mrp_comms_msgs::srv::GetMembersInTeam::Response>();
  
  service_client->requestAndWaitForResponse(request, response);
  std::cout << response->member_name_list.size() << std::endl;
  rclcpp::shutdown();
}