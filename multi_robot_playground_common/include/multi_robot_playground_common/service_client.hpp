#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_CLIENT_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"

namespace mrp_common
{
  template <typename ServiceType>
  class ServiceClient
  {
  public:
    template <typename NodeType>
    explicit ServiceClient(
        NodeType &node,
        const std::string &service_name,
        const rcl_service_options_t &options)
        : ServiceClient(
              node->get_node_base_interface(),
              node->get_node_graph_interface(),
              node->get_node_services_interface(),
              service_name, options)
    {
      callback_group_ = node->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      service_client_ = rclcpp::create_client<ServiceType>(
          node_base_interface_,
          node_graph_interface_,
          node_services_interface_,
          service_name_,
          rmw_qos_profile_default,
          callback_group_);
    }

    explicit ServiceClient(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
        rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
        const std::string &service_name,
        const rcl_service_options_t &options);

    virtual ~ServiceClient();

    typename ServiceType::Response::SharedPtr
    request(typename ServiceType::Request::SharedPtr &request,
            const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface_;
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface_;

    std::string service_name_;

    typename rclcpp::Client<ServiceType>::SharedPtr service_client_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  };
}

#endif