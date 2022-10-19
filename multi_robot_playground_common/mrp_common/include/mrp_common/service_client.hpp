#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_CLIENT_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "logging.hpp"
#include <iostream>

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
        : node_base_interface_(node->get_node_base_interface()),
          node_graph_interface_(node->get_node_graph_interface()),
          node_services_interface_(node->get_node_services_interface()),
          node_logging_interface_(node->get_node_logging_interface()),
          service_name_(service_name)
    {
      callback_group_ = node->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_base_interface_);
      service_client_ = rclcpp::create_client<ServiceType>(
          node_base_interface_,
          node_graph_interface_,
          node_services_interface_,
          service_name_,
          rmw_qos_profile_default,
          callback_group_);
    }

    virtual ~ServiceClient() {}

    bool requestAndWaitForResponse(typename ServiceType::Request::SharedPtr &request,
                 typename ServiceType::Response::SharedPtr &response,
                 const std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
    {
      // Wait for service server to response
      // Question: Should this function be a blocking function ?
      while (!service_client_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          // RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
          std::cout << "client interrupted while waiting for service to appear." << std::endl;
          return 1;
        }
        // RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
        std::cout << "waiting for service to appear..." << std::endl;
      }
      auto future_result = service_client_->async_send_request(request);
      if (executor_->spin_until_future_complete(future_result, timeout) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        throw std::runtime_error(service_name_ + " service client: async_send_request failed");
      }

      response = future_result.get();
      std::cout << "Done" << std::endl;
      return response.get();
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface_;
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;

    std::string service_name_;

    typename rclcpp::Client<ServiceType>::SharedPtr service_client_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  };
}

#endif