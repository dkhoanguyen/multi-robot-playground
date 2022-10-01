#include "multi_robot_component_testing/minimal_service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("minimal_service_client");

  std::shared_ptr<mrp_component_testing::MinimalServiceClient> service_client = 
      std::make_shared<mrp_component_testing::MinimalServiceClient>(node);

  
  std_srvs::srv::SetBool::Request::SharedPtr request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  std_srvs::srv::SetBool::Response::SharedPtr response = std::make_shared<std_srvs::srv::SetBool::Response>();
  
  service_client->requestAndWaitForResponse(request, response);
  
  rclcpp::shutdown();
}