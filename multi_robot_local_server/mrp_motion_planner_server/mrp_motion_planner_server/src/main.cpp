#include "mrp_motion_planner_server/motion_planner_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<mrp_motion_planner::MotionPlannerServer> motion_planner_server =
      std::make_shared<mrp_motion_planner::MotionPlannerServer>("spotturn");
  motion_planner_server->initialise();
  // motion_planner_server->loadController("spotturn_controller");
  // rclcpp::spin(controller_server->get_node_base_interface());
  return 0;
}