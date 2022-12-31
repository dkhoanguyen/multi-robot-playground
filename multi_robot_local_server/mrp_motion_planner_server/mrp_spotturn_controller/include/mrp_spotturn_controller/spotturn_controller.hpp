#ifndef MRP_SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_
#define MRP_SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_

#include "mrp_local_server_core/local_motion_planner.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "mrp_common/utils.hpp"
#include <iostream>

namespace mrp_motion_planner
{
  class SpotTurn : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    SpotTurn();
    virtual ~SpotTurn();

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
    void setParameter(const std::unordered_map<std::string, double> &param_map);

    void calculateVelocityCommand(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<nav_msgs::msg::Odometry> &members_odom,
        const sensor_msgs::msg::LaserScan &scan,
        const double &current_time,
        geometry_msgs::msg::Twist &vel_cmd);

    // For feedback 
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
} // namespace spotturn_controller

#endif // SPOTTURN_CONTROLLER__SPOTTURN_CONTROLLER_HPP_
