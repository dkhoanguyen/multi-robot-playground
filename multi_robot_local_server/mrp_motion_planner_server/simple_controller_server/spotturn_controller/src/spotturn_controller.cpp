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

  double SpotturnController::euclideanDistance(const geometry_msgs::msg::PoseStamped &current_pose,
                                               const geometry_msgs::msg::PoseStamped &target_pose)
  {
  }

  double SpotturnController::angleToTarget(const geometry_msgs::msg::PoseStamped &current_pose,
                                           const geometry_msgs::msg::PoseStamped &target_pose)
  {
  }

  Eigen::MatrixXd SpotturnController::toLocalPose(const geometry_msgs::msg::PoseStamped &current_pose)
  {
    Eigen::MatrixXd current_pose_mtr(3,3);
    current_pose_mtr << 1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0;
    
  }

} // namespace spotturn_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spotturn_controller::SpotturnController, local_server_core::LocalController)