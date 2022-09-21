#include "multi_robot_playground_common/action_client.hpp"

namespace mrp_common
{
  template <typename ActionType>
  ActionClient<ActionType>::ActionClient(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
                                         rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
                                         rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
                                         rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
                                         const std::string &client_name,
                                         const rcl_action_server_options_t &options)
      : node_base_interface_(node_base_interface),
        node_clock_interface_(node_clock_interface),
        node_logging_interface_(node_logging_interface),
        node_waitables_interface_(node_waitables_interface),
        client_name_(client_name)
  {
  }
}