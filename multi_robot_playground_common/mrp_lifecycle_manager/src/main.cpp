#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "mrp_lifecycle_manager/lifecycle_manager.hpp"

#include "std_msgs/msg/string.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<LifecycleManager> lc_node =
        std::make_shared<LifecycleManager>(rcl_node_get_default_options(),
                                           std::chrono::milliseconds(2000));
    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}