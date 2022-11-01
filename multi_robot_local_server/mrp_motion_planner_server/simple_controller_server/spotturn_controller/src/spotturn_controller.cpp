#include "spotturn_controller/spotturn_controller.hpp"

namespace spotturn_controller
{

  SpotturnController::SpotturnController()
  {
    current_waypoint_indx_ = 0;
    setLinearMax(0.22);
    setAngularMax(2.5);
    setLinearError(0.01);
    setAngularError(0.01);
    at_position_ = false;
  }

  SpotturnController::~SpotturnController()
  {
  }

  void SpotturnController::setLinearMax(const double &linear_max)
  {
    max_linear_vel_ = linear_max;
  }
  void SpotturnController::setAngularMax(const double &angular_max)
  {
    max_angular_vel_ = angular_max;
  }
  void SpotturnController::setLinearError(const double &linear_err)
  {
    linear_error_ = linear_err;
  }
  void SpotturnController::setAngularError(const double &angular_err)
  {
    angular_error_ = angular_err;
  }

  void SpotturnController::initialise()
  {
    std::cout << "Initialising spotturn controller" << std::endl;
  }

  void SpotturnController::setWaypoints(const std::vector<geometry_msgs::msg::Pose> waypoints)
  {
    waypoints_ = waypoints;
    current_waypoint_indx_ = 0;
    at_position_ = false;
  }

  void SpotturnController::calculateVelocityCommand(
      const geometry_msgs::msg::Pose &current_pose,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    if(current_waypoint_indx_ == waypoints_.size())
    {
      return;
    }
    geometry_msgs::msg::Pose current_waypoint = waypoints_.at(current_waypoint_indx_);
    vel_cmd.angular.z = calculateAngularVelocity(current_pose, current_waypoint);
    vel_cmd.linear.x = calculateLinearVelocity(current_pose, current_waypoint);

    if (vel_cmd.angular.z == 0 && vel_cmd.linear.x == 0 && at_position_)
    {
      if (current_waypoint_indx_ < waypoints_.size())
      {
        current_waypoint_indx_++;
      }
      at_position_ = false;
    }
  }

  double SpotturnController::calculateLinearVelocity(const geometry_msgs::msg::Pose &current_pose,
                                                     const geometry_msgs::msg::Pose &current_waypoint)
  {
    double distance = mrp_common::GeometryUtils::euclideanDistance(current_pose, current_waypoint);

    double x1 = current_pose.position.x;
    double x2 = current_waypoint.position.x;
    double current_yaw = mrp_common::GeometryUtils::yawFromPose(current_pose);

    double y1 = current_pose.position.y;
    double y2 = current_waypoint.position.y;
    double target_yaw = mrp_common::GeometryUtils::yawFromPose(current_waypoint);

    double theta = atan2((y2 - y1), (x2 - x1)) - current_yaw;
    if (theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }

    if (distance <= linear_error_)
    {
      distance = 0;
      at_position_ = true;
    }

    // Linear
    double linear_vel = distance;
    if (distance > max_linear_vel_)
    {
      linear_vel = max_linear_vel_;
    }

    if (abs(theta) < angular_error_)
    {
      linear_vel = distance;

      if (linear_vel > max_linear_vel_)
      {
        linear_vel = max_linear_vel_;
      }
    }
    else
    {
      linear_vel = 0;
    }
    return linear_vel;
  }
  double SpotturnController::calculateAngularVelocity(const geometry_msgs::msg::Pose &current_pose,
                                                      const geometry_msgs::msg::Pose &current_waypoint)
  {
    double x1 = current_pose.position.x;
    double x2 = current_waypoint.position.x;
    double current_yaw = mrp_common::GeometryUtils::yawFromPose(current_pose);

    double y1 = current_pose.position.y;
    double y2 = current_waypoint.position.y;
    double target_yaw = mrp_common::GeometryUtils::yawFromPose(current_waypoint);

    double theta = atan2((y2 - y1), (x2 - x1)) - current_yaw;

    if (theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }

    if (abs(theta) <= angular_error_)
    {
      theta = 0;
    }

    if (at_position_)
    {
      theta = target_yaw - current_yaw;
      if (abs(theta) <= angular_error_)
      {
        theta = 0;
      }
    }

    double angular_vel = theta;
    if (abs(theta) > max_angular_vel_)
    {
      angular_vel = (theta / abs(theta)) * max_angular_vel_;
    }

    return angular_vel;
  }
} // namespace spotturn_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spotturn_controller::SpotturnController, local_server_core::LocalController)