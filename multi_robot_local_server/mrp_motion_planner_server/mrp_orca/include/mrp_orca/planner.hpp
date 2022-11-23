#ifndef MRP_ORCA__PLANNER_HPP_
#define MRP_ORCA__PLANNER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "mrp_local_server_core/local_motion_planner.hpp"

#include "mrp_common/utils.hpp"

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
    void calculateVelocityCommand(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<nav_msgs::msg::Odometry> &members_odom,
        sensor_msgs::msg::LaserScan &scan,
        geometry_msgs::msg::Twist &vel_cmd);

    // For feedback
    double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose);

    // For accessing
    bool reachGoal();

  protected:
    double robot_radius_;
    double observable_range_;
    double delta_tau_;

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
    Eigen::Vector2d calculateRawVelocity(const geometry_msgs::msg::Pose &current_pose,
                                         const geometry_msgs::msg::Pose &current_waypoint);

    Eigen::Vector2d getOptimalVelocity(const geometry_msgs::msg::Pose &current_pose);
    bool checkCollision(const Eigen::Vector2d &optimal_velocity,
                        const mrp_orca::geometry::HalfPlane &orca_plane);
    void pickNewVelocity();
  };
} // namespace mrp_orca

#endif
