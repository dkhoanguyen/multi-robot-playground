#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "mrp_lifecycle_manager/lifecyle_manager.hpp"

#include "std_msgs/msg/string.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<mrp_lifecycle_manager::LifecycleManager> lifecycle_manager =
      std::make_shared<mrp_lifecycle_manager::LifecycleManager>(options,
                                                                std::chrono::milliseconds(2000));
  rclcpp::spin(lifecycle_manager->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}