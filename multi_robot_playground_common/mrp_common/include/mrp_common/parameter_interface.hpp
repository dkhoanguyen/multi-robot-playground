#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__PARAMETER_INTERFACE_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__PARAMETER_INTERFACE_HPP_

#include <string>

namespace mrp_common
{
  class ParameterInterface
  {
    public:
      virtual ~ParameterInterface(){};

      virtual bool getParameter(const std::string &param_name, bool &value) = 0;
      virtual bool getParameter(const std::string &param_name, int &value) = 0;
      virtual bool getParameter(const std::string &param_name, double &value) = 0;
      virtual bool getParameter(const std::string &param_name, std::string &value) = 0;

      virtual bool setParameter(const std::string &param_name, const int &value) = 0;
      virtual bool setParameter(const std::string &param_name, const double &value) = 0;
      virtual bool setParameter(const std::string &param_name, const std::string &value) = 0;
    
  };
} // namespace mrp_common


#endif