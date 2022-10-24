#include "controller_server/controller_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  mrp_motion_planner_server::controller_server::ControllerServer controller_server;
  controller_server.initialise();

  // For the sake of Thursday demo
  controller_server.loadController("spotturn_controller");
  controller_server.start();
  return 0;
}