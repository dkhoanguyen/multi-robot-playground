#include "mrp_motion_planner_server/motion_planner_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::shared_ptr<mrp_motion_planner::MotionPlannerServer> motion_planner_server =
      std::make_shared<mrp_motion_planner::MotionPlannerServer>("pure_pursuit");
  executor->add_node(motion_planner_server->get_node_base_interface());
  executor->spin();
  return 0;
}