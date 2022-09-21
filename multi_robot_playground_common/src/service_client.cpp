#include "multi_robot_playground_common/service_client.hpp"

namespace mrp_common
{
  template <typename ServiceType>
  ServiceClient<ServiceType>::ServiceClient(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
                                            rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
                                            rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
                                            const std::string &service_name,
                                            const rcl_service_options_t &options)
      : node_base_interface_(node_base_interface),
        node_graph_interface_(node_graph_interface),
        node_services_interface_(node_services_interface),
        service_name_(service_name)
  {
    
  }

  template <typename ServiceType>
  ServiceClient<ServiceType>::~ServiceClient()
  {
  }

  template <typename ServiceType>
  typename ServiceType::Response::SharedPtr
  ServiceClient<ServiceType>::request(typename ServiceType::Request::SharedPtr &request,
                                      const std::chrono::nanoseconds timeout)
  {
    // Wait for service server to response
    // Question: Should this function be a blocking function ?
    while (!service_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        // RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return 1;
      }
      // RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto future_result = service_client_->async_send_request(request);
    if (callback_group_executor_.spin_until_future_complete(future_result, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error(service_name_ + " service client: async_send_request failed");
    }

    return future_result.get();
  }
}