#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"

#include "multi_robot_lifecycle_manager/visibility_control.h"

namespace mrp_lifecycle_manager
{
  class LifecyleManager : public rclcpp::Node
  {
  public:
    LifecyleManager(const rclcpp::NodeOptions & options);
    virtual ~LifecyleManager();

    bool createBondConnection(const std::string & node_name);
  };

} // namespace multi_robot_lifecycle_manager

#endif // MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_
