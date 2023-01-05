#ifndef MRP_PURE_PURSUIT__PURE_PURSUIT_CONTROLLER_HPP_
#define MRP_PURE_PURSUIT__PURE_PURSUIT_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <cmath>
#include <algorithm>

#include <kdl/frames.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "mrp_local_server_core/local_motion_planner.hpp"
#include "mrp_common/parameter_interface.hpp"

#include "mrp_common/utils.hpp"

namespace mrp_pure_pursuit
{
  class PurePursuitController : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    PurePursuitController();
    ~PurePursuitController();

    void initialise();
    void start();
    void stop();

    void setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);
    void calculateVelocityCommand(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<nav_msgs::msg::Odometry> &members_odom,
        const sensor_msgs::msg::LaserScan &scan,
        const double &current_time,
        geometry_msgs::msg::Twist &vel_cmd);

    // For feedback
    double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose);

    // For accessing
    bool reachGoal();

    // Should we abstract away the setting of parameters ?
    // Maybe not ?
    // For accessing ROS parameter server
    void setParameterInterface(std::shared_ptr<mrp_common::ParameterInterface> params_interface);

    void setParameter(const std::unordered_map<std::string, double> &param_map);

  protected:
    std::shared_ptr<mrp_common::ParameterInterface> params_interface_;
    double robot_radius_;
    double observable_range_;
    double delta_tau_;
    bool reevaluate_linear_vel_;

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

    // Vehicle parameters
    double L_;
    // Algorithm variables
    // Position tolerace is measured along the x-axis of the robot!
    double ld_, pos_tol_;
    // Generic control variables
    double v_max_, v_, w_max_;
    // Control variables for Ackermann steering
    // Steering angle is denoted by delta
    double delta_, delta_vel_, acc_, jerk_, delta_max_;
    unsigned idx_;
    bool goal_reached_;
    geometry_msgs::msg::TransformStamped lookahead_;

    //! Compute transform that transforms a pose into the robot frame (base_link)
    KDL::Frame transformToBaseLink(const geometry_msgs::msg::Pose &pose,
                                   const geometry_msgs::msg::Pose &robot_tf);

    double forwardSim(
        const geometry_msgs::msg::Pose &current_pose,
        const double &linear_vel,
        const std::vector<geometry_msgs::msg::PoseStamped> &remaining_path);

  };
}

#endif