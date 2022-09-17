#ifndef SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_
#define SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_

#include "spotturn_controller/visibility_control.h"
#include "mrp_local_server_core/local_controller.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace spotturn_controller
{
  class SpotturnController : public local_server_core::LocalController
  {
  public:
    SpotturnController();
    virtual ~SpotturnController();
    void initialise();
    void setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> waypoints);
    void calculateVelocityCommand(
        geometry_msgs::msg::PoseStamped current_pose,
        geometry_msgs::msg::TwistStamped &vel_cmd);
  };
} // namespace spotturn_controller

#endif // SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_
