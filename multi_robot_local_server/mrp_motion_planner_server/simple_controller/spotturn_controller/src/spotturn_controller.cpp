#include "spotturn_controller/spotturn_controller.hpp"

namespace spotturn_controller
{

  SpotturnController::SpotturnController()
  {
  }

  SpotturnController::~SpotturnController()
  {
  }

  void SpotturnController::initialise()
  {
  }

  void SpotturnController::setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> waypoints)
  {
  }

  void SpotturnController::calculateVelocityCommand(
      geometry_msgs::msg::PoseStamped current_pose,
      geometry_msgs::msg::TwistStamped &vel_cmd)
  {
  }

} // namespace spotturn_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spotturn_controller::SpotturnController, local_server_core::LocalController)