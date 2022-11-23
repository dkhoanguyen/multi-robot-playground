#include "mrp_rvo/rvo.hpp"
#include "ifopt/problem.h"
#include "ifopt/ipopt_solver.h"
#include "ifopt/test_vars_constr_cost.h"
#include "mrp_rvo/orca.hpp"
#include <chrono>

namespace mrp_motion_planner
{
  RVO::RVO()
  {
    current_waypoint_indx_ = 0;
    setLinearMax(0.22);
    setAngularMax(2.5);
    setLinearError(0.01);
    setAngularError(0.01);
    at_position_ = false;
    reach_goal_ = false;
  }

  RVO::~RVO()
  {
  }

  void RVO::start()
  {
    ifopt::Problem nlp;
    nlp.AddVariableSet  (std::make_shared<mrp_orca::ORCAVariables>());
    nlp.AddConstraintSet(std::make_shared<mrp_orca::ORCAConstraint>());
    nlp.AddCostSet      (std::make_shared<mrp_orca::ORCACost>());
    // nlp.PrintCurrent();

    // 2. choose solver and options
    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("print_level", 0);
    ipopt.SetOption("linear_solver", "mumps");
    ipopt.SetOption("jacobian_approximation", "exact");

    // 3 . solve
    auto t_start = std::chrono::high_resolution_clock::now();
    ipopt.Solve(nlp);
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    std::cout << "Solution: "<< x.transpose() << std::endl;
    std::cout << "Elapsed time (ms): " << elapsed_time_ms << std::endl;
  }

  void RVO::stop()
  {
  }

  void RVO::resume()
  {
  }

  void RVO::pause()
  {
  }

  void RVO::setLinearMax(const double &linear_max)
  {
    max_linear_vel_ = linear_max;
  }
  void RVO::setAngularMax(const double &angular_max)
  {
    max_angular_vel_ = angular_max;
  }
  void RVO::setLinearError(const double &linear_err)
  {
    linear_error_ = linear_err;
  }
  void RVO::setAngularError(const double &angular_err)
  {
    angular_error_ = angular_err;
  }

  void RVO::initialise()
  {
    std::cout << "Initialising spotturn controller" << std::endl;
  }

  void RVO::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    path_ = path;
    current_waypoint_indx_ = 0;
    at_position_ = false;
    reach_goal_ = false;
  }

  void RVO::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      sensor_msgs::msg::LaserScan &scan,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    if (current_waypoint_indx_ == path_.size())
    {
      return;
    }
    geometry_msgs::msg::Pose current_waypoint = path_.at(current_waypoint_indx_).pose;
    vel_cmd.angular.z = calculateAngularVelocity(current_odom.pose.pose, current_waypoint);
    vel_cmd.linear.x = calculateLinearVelocity(current_odom.pose.pose, current_waypoint);

    if (vel_cmd.angular.z == 0 && vel_cmd.linear.x == 0 && at_position_)
    {
      if (++current_waypoint_indx_ == path_.size())
      {
        reach_goal_ = true;
      }
      at_position_ = false;
    }
  }

  double RVO::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
    geometry_msgs::msg::Pose current_waypoint = path_.at(current_waypoint_indx_).pose;
    return mrp_common::GeometryUtils::euclideanDistance(current_pose, current_waypoint);
  }

  bool RVO::reachGoal()
  {
    return reach_goal_;
  }

  double RVO::calculateLinearVelocity(const geometry_msgs::msg::Pose &current_pose,
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
  double RVO::calculateAngularVelocity(const geometry_msgs::msg::Pose &current_pose,
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
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_motion_planner::RVO, mrp_local_server_core::MotionPlannerInterface)