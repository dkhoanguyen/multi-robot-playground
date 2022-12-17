#ifndef MRP_NMPC_ORCA__MPC_HPP_
#define MRP_NMPC_ORCA__MPC_HPP_

#include <atomic>
#include <memory>

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>

#include "mrp_local_server_core/local_motion_planner.hpp"

namespace mrp_nmpc_orca
{
  class PoseStablisingMPC : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    PoseStablisingMPC();
    ~PoseStablisingMPC();

    void initialise();
    void start();
    void stop();

    void setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);
    void calculateVelocityCommand(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<nav_msgs::msg::Odometry> &members_odom,
        const sensor_msgs::msg::LaserScan &scan,
        geometry_msgs::msg::Twist &vel_cmd);

    // For feedback
    double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose);

    // For accessing
    bool reachGoal();

    void setParameter(const std::unordered_map<std::string, double> &param_map);

  protected:
    std::shared_ptr<mrp_common::ParameterInterface> params_interface_;
    double robot_radius_;
    double observable_range_;
    double delta_tau_;

    int current_waypoint_indx_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    geometry_msgs::msg::Pose temporary_waypoint_;

    double max_linear_vel_;
    double max_angular_vel_;

    double linear_error_;
    double angular_error_;

    std::atomic<bool> at_position_;
    std::atomic<bool> reach_goal_;
    std::atomic<bool> moving_to_temp_;
  };
}

#endif