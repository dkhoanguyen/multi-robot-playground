#include "multi_robot_component_testing/minimal_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("minimal_action_server");
  std::shared_ptr<mrp_component_testing::MinimalActionServer> action_server = 
    std::make_shared<mrp_component_testing::MinimalActionServer>(node,"test_action");
  
  action_server->activate();
  while(rclcpp::ok())
  {
    
  }
  rclcpp::shutdown();
}