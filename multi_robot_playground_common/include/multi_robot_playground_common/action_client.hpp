#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_CLIENT_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mrp_common
{
  template <typename ActionType>
  class ActionClient
  {
  public:
    explicit ActionClient(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
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

    template <typename NodeType>
    ActionClient(NodeType node,
                 const std::string &client_name,
                 const rcl_service_options_t &options)
        : ActionClient(
              node->get_node_base_interface(),
              node->get_node_clock_interface(),
              node->get_node_logging_interface(),
              node->get_node_waitables_interface(),
              client_name, options)
    {
    }

    virtual ~ActionClient()
    {
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;

    std::string client_name_;
  };
}

#endif