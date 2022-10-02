#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_SERVER_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "logging.hpp"

namespace mrp_common
{
  template <typename ServiceType>
  class ServiceServer
  {
  public:
    template <typename NodeType>
    explicit ServiceServer(
        NodeType node,
        const std::string &service_name,
        const rcl_service_options_t &options)
        : ServiceServer(
              node->get_node_base_interface(),
              node->get_node_services_interface(),
              node->get_node_logging_interface(),
              service_name, options)
    {
      Log::basicInfo(node_logging_interface_, "Creating Callback Group");
      callback_group_ = node->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_group_executor_.add_node(node);
      Log::basicInfo(node_logging_interface_, "Creating Service");
      using namespace std::placeholders;
      // Create service and resgister callback
      service_server_ = rclcpp::create_service<ServiceType>(
          node_base_interface_,
          node_services_interface_,
          service_name_,
          std::bind(&ServiceServer::handleRequest, this, _1, _2),
          rmw_qos_profile_default,
          callback_group_);
    }

    explicit ServiceServer(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        const std::string &service_name,
        const rcl_service_options_t &options)
        : node_base_interface_(node_base_interface),
          node_services_interface_(node_services_interface),
          node_logging_interface_(node_logging_interface),
          service_name_(service_name) {}

    virtual ~ServiceServer() {}

    void start()
    {
      callback_group_executor_.spin();
    }

    void handleRequest(std::shared_ptr<typename ServiceType::Request> request,
                       std::shared_ptr<typename ServiceType::Response> response)
    {
      std::cout << request->data << std::endl;
      execute(request, response);
    }

    virtual void execute(std::shared_ptr<typename ServiceType::Request> &request,
                         std::shared_ptr<typename ServiceType::Response> &response)
    {
      std::cout << "ServiceServerBase" << std::endl;
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;

    std::string service_name_;
    std::future<void> execution_future_;

    typename rclcpp::Service<ServiceType>::SharedPtr service_server_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
  };
}

#endif