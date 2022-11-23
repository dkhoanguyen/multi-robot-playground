#include "mrp_orca/planner.hpp"

namespace mrp_orca
{
  MotionPlanner::MotionPlanner()
      : robot_radius_(0.5),
        observable_range_(2),
        delta_tau_(2),
        current_waypoint_indx_(0),
        max_linear_vel_(0.22),
        max_angular_vel_(2.5),
        linear_error_(0.01),
        angular_error_(0.01),
        at_position_(false),
        reach_goal_(false)
  {
  }

  MotionPlanner::~MotionPlanner()
  {
  }

  void MotionPlanner::initialise()
  {
  }
  void MotionPlanner::start()
  {
  }
  void MotionPlanner::stop()
  {
  }

  void MotionPlanner::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
  }
  void MotionPlanner::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      sensor_msgs::msg::LaserScan &scan,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    std::vector<mrp_orca::geometry::HalfPlane> orca_planes;
    for (const nav_msgs::msg::Odometry member_odom : members_odom)
    {
      // For each member odom, construct the orca plane if the distance between the
      // robots are within some observable range
      if (mrp_common::GeometryUtils::euclideanDistance(
              current_odom.pose.pose, member_odom.pose.pose) <= observable_range_)
      {
        // For now with the lack of robot_info custom message, assume all robots have
        // the same radius and same weight
        orca_planes.push_back(mrp_orca::ORCA::construct(
            current_odom, member_odom, robot_radius_, robot_radius_, delta_tau_, 0.5));
      }
    }

    Eigen::Vector2d robot_vel_vector = mrp_common::GeometryUtils::projectToXY(
        sqrt(std::pow(current_odom.twist.twist.linear.x, 2) + std::pow(current_odom.twist.twist.linear.y, 2)),
        mrp_common::GeometryUtils::yawFromPose(current_odom.pose.pose));

    // Create orca variable object
    std::shared_ptr<mrp_orca::solver::Variables>
        orca_variables_ptr = std::make_shared<mrp_orca::solver::Variables>();
    // Optimise from robot current velocity (which is the optimised velocity toward the target position)
    orca_variables_ptr->SetVariables(robot_vel_vector);

    // Create orca cost function
    std::shared_ptr<mrp_orca::solver::Cost>
        orca_cost_ptr = std::make_shared<mrp_orca::solver::Cost>();
    // Set optimal velocity for cost function (x - xopt)^2 + (y - yopt)^2
    orca_cost_ptr->SetOptimalVelocity(robot_vel_vector);

    // Create orca constraints
    std::shared_ptr<mrp_orca::solver::Constraint>
        orca_constraint_ptr = std::make_shared<mrp_orca::solver::Constraint>(orca_planes.size());
    orca_constraint_ptr->AddConstraints(orca_planes);

    // Create solver and solve for optimal velocity
    Eigen::Vector2d non_collision_velocity = mrp_orca::solver::Solver::solve(
        orca_variables_ptr, orca_constraint_ptr, orca_cost_ptr);
  }

  // For feedback
  double MotionPlanner::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
  }

  // For accessing
  bool MotionPlanner::reachGoal()
  {
  }

  double MotionPlanner::calculateLinearVelocity(const geometry_msgs::msg::Pose &current_pose,
                                                const geometry_msgs::msg::Pose &current_waypoint)
  {
  }
  double MotionPlanner::calculateAngularVelocity(const geometry_msgs::msg::Pose &current_pose,
                                                 const geometry_msgs::msg::Pose &current_waypoint)
  {
  }
} // namespace mrp_orca
