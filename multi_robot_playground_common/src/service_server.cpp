#include "multi_robot_playground_common/service_server.hpp"

namespace mrp_common
{
  // template <typename NodeType>
  // ServiceServer::ServiceServer(
  //     NodeType node,
  //     const std::string &service_name,
  //     const rcl_service_options_t &options)
  //     : ServiceServer(
  //           node->get_node_base_interface(),
  //           node->get_node_clock_interface(),
  //           node->get_node_logging_interface(),
  //           node->get_node_waitables_interface(),
  //           service_name, options)
  // {
  // }
  // template <typename ServiceType>
  // ServiceServer<ServiceType>::ServiceServer(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  //                                           rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  //                                           rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  //                                           rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  //                                           const std::string &service_name,
  //                                           const rcl_service_options_t &options)
  //     : node_base_interface_(node_base_interface),
  //       node_clock_interface_(node_clock_interface),
  //       node_logging_interface_(node_logging_interface),
  //       node_waitables_interface_(node_waitables_interface),
  //       service_name_(service_name)
  // {
  //   service_server_ = rclcpp::create_service<ServiceType>(service_name_, &ServiceServer::handleRequest);
  // }

  // template <typename ServiceType>
  // ServiceServer::~ServiceServer()
  // {
  // }

  // template <typename ServiceType>
  // void ServiceServer<ServiceType>::handleRequest(std::shared_ptr<typename ServiceType::Request> request,
  //                                                std::shared_ptr<typename ServiceType::Response> response)
  // {
  //   std::cout << "mrp_common::ServiceServer" << std::endl;
  //   // Fire up a separate thread and handle the request
  //   execution_future_ = std::async(std::launch::async, [this, request, response]()
  //                                  { execute(request, response); });
  // }

  // template <typename ServiceType>
  // void ServiceServer<ServiceType>::execute(std::shared_ptr<typename ServiceType::Request> request,
  //                                          std::shared_ptr<typename ServiceType::Response> response)
  // {
  // }
}
