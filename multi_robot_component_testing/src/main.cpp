#include "multi_robot_component_testing/minimal_service_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_service_server");

  std::shared_ptr<mrp_component_testing::MinimalServiceServer> service_server = 
      std::make_shared<mrp_component_testing::MinimalServiceServer>(node);

  // mrp_component_testing::MinimalServiceServer service_server(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
}