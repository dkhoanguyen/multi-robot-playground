#include "mrp_orca/planner.hpp"
#include <iostream>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mrp_orca
{
  MotionPlanner::MotionPlanner()
      : robot_radius_(0.105),
        observable_range_(1),
        delta_tau_(1.75),
        current_waypoint_indx_(0),
        max_linear_vel_(0.1),
        max_angular_vel_(2.0),
        linear_error_(0.01),
        angular_error_(0.01),
        at_position_(false),
        reach_goal_(false),
        moving_to_temp_(false)
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
    path_ = path;
    current_waypoint_indx_ = 0;
    at_position_ = false;
    reach_goal_ = false;
  }

  void MotionPlanner::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      sensor_msgs::msg::LaserScan &scan,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    if (current_waypoint_indx_ == path_.size())
    {
      return;
    }

    return plan(current_odom, members_odom, scan, vel_cmd);

    // // If we are moving towards temporary waypoint
    // // Skip the calculation of the ORCA planes and commence moving until we reach the target
    // // In the future this should not be like this
    // // ORCA should run at a lower frequency than control signal
    // if (moving_to_temp_)
    // {
    //   // std::cout << "Moving to temporary waypoint " << std::endl;
    //   Eigen::Vector2d vel_to_target = calculateVelocityToTarget(
    //       current_odom.pose.pose,
    //       temporary_waypoint_);
    //   // Maybe let's reconsider this at some point in the future
    //   vel_cmd.angular.z = vel_to_target(0);
    //   vel_cmd.linear.x = vel_to_target(1);

    //   if (vel_cmd.angular.z == 0 && vel_cmd.linear.x == 0 && at_position_)
    //   {
    //     at_position_ = false;
    //     moving_to_temp_ = false;
    //   }
    //   return;
    // }

    // // Else extract the next target location
    // geometry_msgs::msg::Pose current_waypoint = path_.at(current_waypoint_indx_).pose;

    // std::vector<mrp_orca::geometry::HalfPlane> orca_planes;
    // for (const nav_msgs::msg::Odometry member_odom : members_odom)
    // {
    //   mrp_orca::geometry::HalfPlane orca_plane;
    //   // For each member odom, construct the orca plane if the distance between the
    //   // robots are within some observable range
    //   if (mrp_common::GeometryUtils::euclideanDistance(
    //           current_odom.pose.pose,
    //           member_odom.pose.pose) <= observable_range_)
    //   {
    //     // For now with the lack of robot_info custom message, assume all robots have
    //     // the same radius and same weight
    //     Eigen::Vector2d vel_to_target = calculateOptimalVelocity(
    //         current_odom.pose.pose,
    //         current_waypoint);
    //     nav_msgs::msg::Odometry temp_current_odom = current_odom;
    //     // Maybe let's reconsider this at some point in the future
    //     temp_current_odom.twist.twist.linear.x = vel_to_target.norm();

    //     tf2::Quaternion quad;
    //     quad.setRPY(0, 0, std::atan2(current_waypoint.position.y - current_odom.pose.pose.position.y, current_waypoint.position.x - current_odom.pose.pose.position.x));
    //     tf2::convert(quad, temporary_waypoint_.orientation);

    //     if (mrp_orca::ORCA::construct(
    //             orca_plane, temp_current_odom, member_odom,
    //             robot_radius_, robot_radius_, delta_tau_, 1))
    //     {
    //       // We only append the orca_plane if it is valid
    //       orca_planes.push_back(orca_plane);
    //     }
    //   }
    // }

    // // If there is no immediate collision, get the optimal velocity to target
    // if (orca_planes.size() == 0)
    // {
    //   // Get optimal velocity to target
    //   Eigen::Vector2d vel_to_target = calculateVelocityToTarget(
    //       current_odom.pose.pose,
    //       current_waypoint);
    //   // Maybe let's reconsider this at some point in the future
    //   vel_cmd.angular.z = vel_to_target(0);
    //   vel_cmd.linear.x = vel_to_target(1);

    //   if (vel_cmd.angular.z == 0 && vel_cmd.linear.x == 0 && at_position_)
    //   {
    //     if (current_waypoint_indx_ < path_.size())
    //     {
    //       current_waypoint_indx_++;
    //     }
    //     else
    //     {
    //       reach_goal_ = true;
    //     }
    //     at_position_ = false;
    //   }

    //   return;
    // }
    // else
    // {
    //   // std::cout << "Potential collision detected" << std::endl;
    // }

    // // If there is a collision in the near future
    // // Solve the linear programming optimisation to find the optimal velocity vector
    // // that is close to the desired velocity to target
    // Eigen::Vector2d opt_vel_vector = calculateOptimalVelocity(current_odom.pose.pose,
    //                                                           current_waypoint);
    // // Create orca variable object
    // std::shared_ptr<mrp_orca::solver::Variables>
    //     orca_variables_ptr = std::make_shared<mrp_orca::solver::Variables>();
    // // Optimise from robot current velocity (which is the optimised velocity toward the target position)
    // orca_variables_ptr->SetVariables(opt_vel_vector);
    // // Set bounds
    // // Upper bounds
    // Eigen::Vector2d upper_bound(0.1, 0.1);
    // Eigen::Vector2d lower_bound(-0.1, -0.1);
    // orca_variables_ptr->SetBounds(lower_bound, upper_bound);

    // // Create orca cost function
    // std::shared_ptr<mrp_orca::solver::Cost>
    //     orca_cost_ptr = std::make_shared<mrp_orca::solver::Cost>();
    // // Set optimal velocity for cost function (x - xopt)^2 + (y - yopt)^2
    // orca_cost_ptr->SetOptimalVelocity(opt_vel_vector);

    // // Create orca constraints
    // std::shared_ptr<mrp_orca::solver::Constraint>
    //     orca_constraint_ptr = std::make_shared<mrp_orca::solver::Constraint>(orca_planes.size());
    // orca_constraint_ptr->AddConstraints(orca_planes);

    // // Create solver and solve for optimal velocity
    // Eigen::Vector2d non_collision_velocity = mrp_orca::solver::Solver::solve(
    //     orca_variables_ptr, orca_constraint_ptr, orca_cost_ptr);

    // approximateTemporaryWaypoint(current_odom.pose.pose, non_collision_velocity);
    // moving_to_temp_ = true;

    // Eigen::Vector2d vel_to_target = calculateVelocityToTarget(
    //     current_odom.pose.pose,
    //     temporary_waypoint_);
    // // Maybe let's reconsider this at some point in the future
    // vel_cmd.angular.z = vel_to_target(0);
    // vel_cmd.linear.x = vel_to_target(1);
  }

  // For feedback
  double MotionPlanner::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
    geometry_msgs::msg::Pose current_waypoint = path_.at(current_waypoint_indx_).pose;
    return mrp_common::GeometryUtils::euclideanDistance(current_pose, current_waypoint);
  }

  // For accessing
  bool MotionPlanner::reachGoal()
  {
    return reach_goal_;
  }

  double MotionPlanner::calculateLinearVelocity(const geometry_msgs::msg::Pose &current_pose,
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
  double MotionPlanner::calculateAngularVelocity(const geometry_msgs::msg::Pose &current_pose,
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

  Eigen::Vector2d MotionPlanner::calculateVelocityToTarget(const geometry_msgs::msg::Pose &current_pose,
                                                           const geometry_msgs::msg::Pose &current_waypoint)
  {
    return Eigen::Vector2d(
        calculateAngularVelocity(current_pose, current_waypoint),
        calculateLinearVelocity(current_pose, current_waypoint));
  }

  Eigen::Vector2d MotionPlanner::calculateOptimalVelocity(const geometry_msgs::msg::Pose &current_pose,
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
    }

    double linear_vel = distance;
    if (distance > max_linear_vel_)
    {
      linear_vel = max_linear_vel_;
    }

    // Transform from polar coordinate to cartesian coordinate
    Eigen::Vector2d projected_vel = mrp_common::GeometryUtils::projectToXY(linear_vel, theta);
    // Transform to world frame

    geometry_msgs::msg::Pose vel_pos;
    vel_pos.position.x = projected_vel(0);
    vel_pos.position.y = projected_vel(1);

    tf2::Transform t_w_r, t_r_v;
    tf2::fromMsg(current_pose, t_w_r);
    tf2::fromMsg(vel_pos, t_r_v);

    tf2::Transform t_w_vel = t_w_r * t_r_v;
    geometry_msgs::msg::Pose transformed_vel;
    tf2::toMsg(t_w_vel, transformed_vel);

    // std::cout << "Transformed vel x: " <<  transformed_vel.position.x << ", y: " << transformed_vel.position.y << std::endl;

    // std::cout << "Optimal velocity: " << projected_vel.transpose() << std::endl;
    return projected_vel;
  }

  Eigen::Vector2d MotionPlanner::calculateCmdVelFromVelVect(
      const Eigen::Vector2d &vel_vect,
      const geometry_msgs::msg::Pose &current_pose)
  {
    double current_yaw = mrp_common::GeometryUtils::yawFromPose(current_pose);
    double theta = atan2(vel_vect(1), vel_vect(0)) - current_yaw;
    double linear = vel_vect.norm();

    if (theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }

    if (theta == 0)
    {
      if (linear > max_linear_vel_)
      {
        linear = max_linear_vel_;
      }
    }
    else
    {
      linear = 0;
    }

    return Eigen::Vector2d(linear, theta);
  }

  bool MotionPlanner::checkCollision(const Eigen::Vector2d &optimal_velocity,
                                     const mrp_orca::geometry::HalfPlane &orca_plane)
  {
    return true;
  }

  void MotionPlanner::approximateTemporaryWaypoint(
      const geometry_msgs::msg::Pose &current_pose,
      const Eigen::Vector2d &vel_vect)
  {
    // Position
    geometry_msgs::msg::Pose local_temp_pose;
    local_temp_pose.position.x = vel_vect(0);
    local_temp_pose.position.y = vel_vect(1);

    // Orientation
    tf2::Quaternion quad;
    quad.setRPY(0, 0, std::atan2(vel_vect(1), vel_vect(0)));
    tf2::convert(quad, local_temp_pose.orientation);

    temporary_waypoint_ = mrp_common::TransformUtils::toGlobalFrame(
        current_pose, local_temp_pose);
  }

  void MotionPlanner::plan(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      sensor_msgs::msg::LaserScan &scan,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    // If we are moving towards temporary waypoint
    // Skip the calculation of the ORCA planes and commence moving until we reach the target
    // In the future this should not be like this
    // ORCA should run at a lower frequency than control signal
    nav_msgs::msg::Odometry local_odom_A;
    if (moving_to_temp_)
    {
      // std::cout << "Moving to temporary waypoint " << std::endl;
      Eigen::Vector2d vel_to_target = calculateVelocityToTarget(
          current_odom.pose.pose,
          temporary_waypoint_);
      // Maybe let's reconsider this at some point in the future
      vel_cmd.angular.z = vel_to_target(0);
      vel_cmd.linear.x = vel_to_target(1);

      if (vel_cmd.angular.z == 0 && vel_cmd.linear.x == 0 && at_position_)
      {
        at_position_ = false;
        moving_to_temp_ = false;
      }
      return;
    }

    // Else extract the next target location
    geometry_msgs::msg::Pose current_waypoint_local =
        mrp_common::TransformUtils::toLocalFrame(
            current_odom.pose.pose,
            path_.at(current_waypoint_indx_).pose);

    std::vector<mrp_orca::geometry::HalfPlane> orca_planes;
    Eigen::Vector2d opt_vel_vector = calculateOptimalVelocity(
        local_odom_A.pose.pose,
        current_waypoint_local);

    for (const nav_msgs::msg::Odometry member_odom : members_odom)
    {
      mrp_orca::geometry::HalfPlane orca_plane;
      // For each member odom, construct the orca plane if the distance between the
      // robots are within some observable range
      if (mrp_common::GeometryUtils::euclideanDistance(
              current_odom.pose.pose,
              member_odom.pose.pose) <= observable_range_)
      {
        // For now with the lack of robot_info custom message, assume all robots have
        // the same radius and same weight
        nav_msgs::msg::Odometry local_member_odom;
        local_member_odom.pose.pose = mrp_common::TransformUtils::toLocalFrame(
            current_odom.pose.pose,
            member_odom.pose.pose);

        if (mrp_orca::ORCA::localConstruct(
                orca_plane, opt_vel_vector, local_member_odom,
                robot_radius_, robot_radius_, delta_tau_, 1))
        {
          // We only append the orca_plane if it is valid
          orca_planes.push_back(orca_plane);
        }
      }
    }

    // If there is no immediate collision, get the optimal velocity to target
    if (orca_planes.size() == 0)
    {
      geometry_msgs::msg::Pose current_waypoint = path_.at(current_waypoint_indx_).pose;
      // Get optimal velocity to target
      Eigen::Vector2d vel_to_target = calculateVelocityToTarget(
          current_odom.pose.pose,
          current_waypoint);
      // Maybe let's reconsider this at some point in the future
      vel_cmd.angular.z = vel_
    // std::cout << "ORCA point: " << orca_point.transpose() << std::endl;
    // std::cout << "U: " << weighted_u.transpose() << std::endl;to_target(0);
      vel_cmd.linear.x = vel_to_target(1);

      if (vel_cmd.angular.z == 0 && vel_cmd.linear.x == 0 && at_position_)
      {
        if (current_waypoint_indx_ < path_.size())
        {
          current_waypoint_indx_++;
        }
        else
        {
          reach_goal_ = true;
        }
        at_position_ = false;
      }

      return;
    }

    // If there is a collision in the near future
    // Solve the linear programming optimisation to find the optimal velocity vector
    // that is close to the desired velocity to target
    // Create orca variable object
    std::shared_ptr<mrp_orca::solver::Variables>
        orca_variables_ptr = std::make_shared<mrp_orca::solver::Variables>();
    // Optimise from robot current velocity (which is the optimised velocity toward the target position)
    orca_variables_ptr->SetVariables(opt_vel_vector);
    // Set bounds
    // Upper bounds
    Eigen::Vector2d upper_bound(0.1, 0.1);
    Eigen::Vector2d lower_bound(-0.1, -0.1);
    orca_variables_ptr->SetBounds(lower_bound, upper_bound);

    // Create orca cost function
    std::shared_ptr<mrp_orca::solver::Cost>
        orca_cost_ptr = std::make_shared<mrp_orca::solver::Cost>();
    // Set optimal velocity for cost function (x - xopt)^2 + (y - yopt)^2
    orca_cost_ptr->SetOptimalVelocity(opt_vel_vector);

    // Create orca constraints
    std::shared_ptr<mrp_orca::solver::Constraint>
        orca_constraint_ptr = std::make_shared<mrp_orca::solver::Constraint>(orca_planes.size());
    orca_constraint_ptr->AddConstraints(orca_planes);

    // Create solver and solve for optimal velocity
    Eigen::Vector2d non_collision_velocity = mrp_orca::solver::Solver::solve(
        orca_variables_ptr, orca_constraint_ptr, orca_cost_ptr);

    // std::cout << "Non collision: " << non_collision_velocity.transpose() << std::endl;

    approximateTemporaryWaypoint(current_odom.pose.pose, non_collision_velocity);
    moving_to_temp_ = true;

    Eigen::Vector2d vel_to_target = calculateVelocityToTarget(
        local_odom_A.pose.pose,
        temporary_waypoint_);
    // Maybe let's reconsider this at some point in the future
    vel_cmd.angular.z = vel_to_target(0);
    vel_cmd.linear.x = vel_to_target(1);
  }

} // namespace mrp_orca

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_orca::MotionPlanner, mrp_local_server_core::MotionPlannerInterface)
