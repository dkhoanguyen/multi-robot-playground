#include "mrp_comms_server/comms_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::shared_ptr<mrp_comms::CommsServer> comms_server =
      std::make_shared<mrp_comms::CommsServer>();
  executor->add_node(comms_server->get_node_base_interface());
  executor->spin();
  return 0;
}