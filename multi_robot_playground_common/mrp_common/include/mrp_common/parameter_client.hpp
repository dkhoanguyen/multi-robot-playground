#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__PARAMETER_CLIENT_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__PARAMETER_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "parameter_interface.hpp"

namespace mrp_common
{
  class ParameterClient : public mrp_common::ParameterInterface
  {
  public:
    explicit ParameterClient();
    virtual ~ParameterClient();

    void setNodeParamInterface(std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_params_interface_);

    // Get single parameter
    bool getParameter(const std::string &param_name, bool &value);
    bool getParameter(const std::string &param_name, int &value);
    bool getParameter(const std::string &param_name, double &value);
    bool getParameter(const std::string &param_name, std::string &value);

    // Get paramter Array


    // Set single parameter
    // TODO: use template to abstract away typing as
    // the ros interface uses template as well
    bool setParameter(const std::string &param_name, const int &value);
    bool setParameter(const std::string &param_name, const double &value);
    bool setParameter(const std::string &param_name, const std::string &value);

  protected:
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_params_interface_;
  };
}

#endif
