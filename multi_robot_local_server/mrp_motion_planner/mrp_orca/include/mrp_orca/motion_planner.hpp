#ifndef MRP_ORCA__MOTION_PLANNER_HPP_
#define MRP_ORCA__MOTION_PLANNER_HPP_

#include <atomic>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

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
#include "mrp_comms_msgs/msg/member_state.hpp"

#include "mrp_orca/orca.hpp"
#include "mrp_orca/solver.hpp"
#include "mrp_orca/geometry.hpp"

namespace mrp_orca
{
  class MotionPlanner : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    MotionPlanner();
    ~MotionPlanner();

    void initialise();
    void start();
    void stop();

    void setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);

    void step(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<mrp_comms_msgs::msg::MemberState> &members_state,
        const sensor_msgs::msg::LaserScan &scan,
        geometry_msgs::msg::Twist &vel_cmd);

    // For feedback
    double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose);

    // For accessing
    bool reachGoal();

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
    std::atomic<bool> allow_reverse_;

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
    int idx_;
    bool goal_reached_;

    void trackLookahead(
        const geometry_msgs::msg::Pose &current_pose,
        const geometry_msgs::msg::TransformStamped &lookahead,
        const bool &evaluate_linear_if_allow_reverse,
        geometry_msgs::msg::Twist &vel_cmd);

    //! Compute transform that transforms a pose into the robot frame (base_link)
    KDL::Frame transformToBaseLink(const geometry_msgs::msg::Pose &pose,
                                   const geometry_msgs::msg::Pose &robot_tf);

    int extractNextWaypoint(
        const geometry_msgs::msg::Pose &current_pose,
        const int &start_indx,
        const std::vector<geometry_msgs::msg::PoseStamped> &path,
        geometry_msgs::msg::TransformStamped &lookahead_tf);

    double forwardSim(
        const geometry_msgs::msg::Pose &current_pose,
        const double &linear_vel,
        const std::vector<geometry_msgs::msg::PoseStamped> &remaining_path);

    Eigen::Vector2d calculateOptimalVelocity(
        const geometry_msgs::msg::Pose &current_pose,
        const geometry_msgs::msg::Pose &current_waypoint);

    Eigen::Vector2d calculateOptimalVelocityGlobal(
        const geometry_msgs::msg::Pose &current_pose,
        const geometry_msgs::msg::Pose &current_waypoint);

    bool isApproachingFinal(
        const std::vector<geometry_msgs::msg::PoseStamped> &path,
        const int &indx)
    {
      return !path.empty() && indx >= path.size();
    };

    Eigen::Vector3d bodyToWorld(const geometry_msgs::msg::Pose &current_pose,
                                const Eigen::Vector3d &local_vel)
    {
      double theta = tf2::getYaw(current_pose.orientation);
      Eigen::Matrix3d R;
      R << cos(theta), -sin(theta), 0,
          sin(theta), cos(theta), 0,
          0, 0, 1;
      Eigen::Vector3d world_vel = R * local_vel;
      return world_vel;
    };
  };
}

#endif