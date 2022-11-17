#ifndef MRP_RVO__RVO_HPP_
#define MRP_RVO__RVO_HPP_

#include <iostream>
#include <cmath>

#include "mrp_local_server_core/local_motion_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "mrp_common/utils.hpp"

// #include "OsqpEigen/OsqpEigen.h"
#include "osqp/osqp.h"
#include "ifopt/variable_set.h"
// #include "ceres/ceres.h"


namespace mrp_motion_planner
{
  class RVO : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    RVO();
    virtual ~RVO();

    void setLinearMax(const double &linear_max);
    void setAngularMax(const double &angular_max);
    void setLinearError(const double &linear_err);
    void setAngularError(const double &angular_err);

    void initialise();
    void start();
    void stop();
    void resume();
    void pause();

    void setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);
    void calculateVelocityCommand(
        const geometry_msgs::msg::Pose &current_pose,
        const std::vector<nav_msgs::msg::Odometry> &members_odom,
        sensor_msgs::msg::LaserScan &scan,
        geometry_msgs::msg::Twist &vel_cmd);

    double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose);

    bool reachGoal();

  protected:
    int current_waypoint_indx_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;

    double max_linear_vel_;
    double max_angular_vel_;

    double linear_error_;
    double angular_error_;

    bool at_position_;
    bool reach_goal_;

    double calculateLinearVelocity(const geometry_msgs::msg::Pose &current_pose,
                                   const geometry_msgs::msg::Pose &current_waypoint);
    double calculateAngularVelocity(const geometry_msgs::msg::Pose &current_pose,
                                    const geometry_msgs::msg::Pose &current_waypoint);
  };
}

#endif