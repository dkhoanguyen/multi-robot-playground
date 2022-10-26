#include "controller_server/controller_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<mrp_motion_planner_server::controller_server::ControllerServer> controller_server =
      std::make_shared<mrp_motion_planner_server::controller_server::ControllerServer>();
  controller_server->initialise();
  // For the sake of Thursday demo
  controller_server->loadController("spotturn_controller");
  rclcpp::spin(controller_server->get_node_base_interface());
  return 0;
}