#include "mrp_common/parameter_client.hpp"

namespace mrp_common
{
  ParameterClient::ParameterClient()
  {
  }

  ParameterClient::~ParameterClient()
  {
  }

  void ParameterClient::setNodeParamInterface(
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_params_interface)
  {
    node_params_interface_ = node_params_interface;
  }

  bool ParameterClient::getParameter(const std::string &param_name, bool &value)
  {
    try
    {
      value = node_params_interface_->get_parameter(param_name).as_int();
      return true;
    }
    catch (const std::out_of_range &e)
    {
      return false;
    }
  }

  bool ParameterClient::getParameter(const std::string &param_name, int &value)
  {
    try
    {
      value = node_params_interface_->get_parameter(param_name).as_int();
      return true;
    }
    catch (const std::out_of_range &e)
    {
      return false;
    }
  }
  bool ParameterClient::getParameter(const std::string &param_name, double &value)
  {
    try
    {
      value = node_params_interface_->get_parameter(param_name).as_double();
      return true;
    }
    catch (const std::out_of_range &e)
    {
      return false;
    }
  }
  bool ParameterClient::getParameter(const std::string &param_name, std::string &value)
  {
    try
    {
      value = node_params_interface_->get_parameter(param_name).as_string();
      return true;
    }
    catch (const std::out_of_range &e)
    {
      return false;
    }
  }

  bool ParameterClient::setParameter(const std::string &param_name, const int &value)
  {
    try
    {
      rclcpp::ParameterValue param_value(value);
      rclcpp::Parameter param(param_name, param_value);

      std::vector<rclcpp::Parameter> params = {param};
      node_params_interface_->set_parameters(params);
      return true;
    }
    catch (const std::exception &e)
    {
      return false;
    }
  }
  bool ParameterClient::setParameter(const std::string &param_name, const double &value)
  {
    try
    {
      rclcpp::ParameterValue param_value(value);
      rclcpp::Parameter param(param_name, param_value);

      std::vector<rclcpp::Parameter> params = {param};
      node_params_interface_->set_parameters(params);
      return true;
    }
    catch (const std::exception &e)
    {
      return false;
    }
  }
  bool ParameterClient::setParameter(const std::string &param_name, const std::string &value)
  {
    try
    {
      rclcpp::ParameterValue param_value(value);
      rclcpp::Parameter param(param_name, param_value);

      std::vector<rclcpp::Parameter> params = {param};
      node_params_interface_->set_parameters(params);
      return true;
    }
    catch (const std::exception &e)
    {
      return false;
    }
  }

}