#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "multi_robot_component_testing/lifecycle_node.hpp"

int main(int argc, char **argv)
{
  // Some initialization.
  rclcpp::init(argc, argv);
  std::chrono::milliseconds interval(200);
  std::string name = "test_lifecycle_node";
  std::string ns = "test";
  bool auto_start = true;
  bool health_check = true;
  mrp_common::LifecycleNode node(name, ns, auto_start, health_check, interval);
  node.initialiseHeartbeat();
  node.startHeartbeat(); 
  rclcpp::spin(node.get_node_base_interface());
  return 0;
}