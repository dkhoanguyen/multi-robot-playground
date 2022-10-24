#ifndef SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_
#define SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_

#include "spotturn_controller/visibility_control.h"
#include "mrp_local_server_core/local_controller.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "mrp_common/utils.hpp"
#include <iostream>

namespace spotturn_controller
{
  class SpotturnController : public local_server_core::LocalController
  {
  public:
    SpotturnController();
    virtual ~SpotturnController();

    void setLinearMax(const double &linear_max);
    void setAngularMax(const double &angular_max);
    void setLinearError(const double &linear_err);
    void setAngularError(const double &angular_err);

    void initialise();
    void setWaypoints(const std::vector<geometry_msgs::msg::Pose> waypoints);
    void calculateVelocityCommand(
        const geometry_msgs::msg::Pose &current_pose,
        geometry_msgs::msg::TwistStamped &vel_cmd);

  protected:
    int current_waypoint_indx_;
    std::vector<geometry_msgs::msg::Pose> waypoints_;

    double max_linear_vel_;
    double max_angular_vel_;

    double linear_error_;
    double angular_error_;

    bool at_position_;

    double calculateLinearVelocity(const geometry_msgs::msg::Pose &current_pose,
                                   const geometry_msgs::msg::Pose &current_waypoint);
    double calculateAngularVelocity(const geometry_msgs::msg::Pose &current_pose,
                                    const geometry_msgs::msg::Pose &current_waypoint);
  };
} // namespace spotturn_controller

#endif // SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_
