#include "multi_robot_lifecycle_manager/lifecyle_manager.hpp"

namespace mrp_lifecycle_manager
{

  LifecyleManager::LifecyleManager(const rclcpp::NodeOptions &options)
      : Node("mrp_lifecycle_manager", options)
  {
  }

  LifecyleManager::~LifecyleManager()
  {
  }

} // namespace multi_robot_lifecycle_manager
