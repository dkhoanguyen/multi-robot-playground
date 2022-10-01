#include "multi_robot_component_testing/minimal_action_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("minimal_action_client");
  std::shared_ptr<mrp_component_testing::MinimalActionClient> action_client =
      std::make_shared<mrp_component_testing::MinimalActionClient>(node, "test_action");

  auto goal_msg = turtlesim::action::RotateAbsolute::Goal();
  goal_msg.theta = 10;

  action_client->sendGoal(goal_msg);
  while(rclcpp::ok())
  {
    
  }

  rclcpp::shutdown();
}