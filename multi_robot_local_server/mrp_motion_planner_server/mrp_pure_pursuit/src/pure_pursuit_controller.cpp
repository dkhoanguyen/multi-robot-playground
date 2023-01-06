#include "mrp_pure_pursuit/pure_pursuit_controller.hpp"

namespace mrp_pure_pursuit
{
  PurePursuitController::PurePursuitController()
      : ld_(0.1), v_max_(0.05), v_(v_max_), w_max_(0.5), pos_tol_(0.005), current_waypoint_indx_(0),
        goal_reached_(true), L_(0.1), allow_reverse_(false),
        robot_radius_(0.14),
        observable_range_(1),
        delta_tau_(3),
        max_linear_vel_(0.1),
        max_angular_vel_(2.0),
        linear_error_(0.01),
        angular_error_(0.05),
        at_position_(false),
        reach_goal_(false),
        moving_to_temp_(false)
  {
    reevaluate_linear_vel_ = true;
    current_waypoint_indx_ = 0;
    std::cout << "Intialisation PurePursuitController" << std::endl;
  }

  PurePursuitController::~PurePursuitController()
  {
  }

  void PurePursuitController::initialise()
  {
  }
  void PurePursuitController::start()
  {
  }
  void PurePursuitController::stop()
  {
  }

  void PurePursuitController::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    current_waypoint_indx_ = 0;
    goal_reached_ = false;
    path_ = path;
    reevaluate_linear_vel_ = true;
  }
  void PurePursuitController::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      const sensor_msgs::msg::LaserScan &scan,
      const double &current_time,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Get lookahead for the next waypoint
    int pre_indx = current_waypoint_indx_;
    geometry_msgs::msg::TransformStamped lookahead;

    // Extract waypoint lookahead and update next waypoint index
    std::cout << "Extracting lookahead" << std::endl;
    current_waypoint_indx_ = extractNextWaypoint(
        current_odom.pose.pose, current_waypoint_indx_, path_,
        lookahead);

    // Condition to support backwards driving
    bool evaluate_linear_vel_if_allow_reverse =
        (current_waypoint_indx_ != pre_indx && current_waypoint_indx_ < path_.size());

    // If the current waypoint idx exceeds the path size
    if (current_waypoint_indx_ >= path_.size())
    {
      trackLookahead(current_odom.pose.pose, lookahead,
                     evaluate_linear_vel_if_allow_reverse, vel_cmd);
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
      return;
    }

    // Convert waypoint to baselink of robot
    nav_msgs::msg::Odometry local_odom_A;
    geometry_msgs::msg::Pose current_waypoint_local =
        mrp_common::TransformUtils::toLocalFrame(
            current_odom.pose.pose,
            path_.at(current_waypoint_indx_).pose);

    // Get the optimal velocity vector towards the waypoint
    std::vector<mrp_pure_pursuit::geometry::HalfPlane> orca_planes;
    Eigen::Vector2d opt_vel_vector = calculateOptimalVelocity(
        local_odom_A.pose.pose,
        current_waypoint_local);

    for (const nav_msgs::msg::Odometry member_odom : members_odom)
    {
      mrp_pure_pursuit::geometry::HalfPlane orca_plane;
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

        if (mrp_pure_pursuit::ORCA::localConstruct(
                orca_plane, opt_vel_vector, local_member_odom,
                robot_radius_, robot_radius_, delta_tau_, 1) == mrp_pure_pursuit::ORCA::Result::COLLISION)
        {
          // We only append the orca_plane if it is valid
          orca_planes.push_back(orca_plane);
        }
      }
    }

    if (orca_planes.size() == 0)
    {
      std::cout << "No immediate collision detected" << std::endl;
      // If there is no immediate collision, move toward the goal simply by tracking the path
      trackLookahead(current_odom.pose.pose, lookahead,
                     evaluate_linear_vel_if_allow_reverse, vel_cmd);

      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

      return;
    }

    std::cout << "Collision detected" << std::endl;
    // If there is a collision in the near future
    // Solve the linear programming optimisation to find the optimal velocity vector
    // that is close to the desired velocity to target
    // Create orca variable object
    std::shared_ptr<mrp_pure_pursuit::solver::Variables>
        orca_variables_ptr = std::make_shared<mrp_pure_pursuit::solver::Variables>();
    // Optimise from robot current velocity (which is the optimised velocity toward the target position)
    orca_variables_ptr->SetVariables(opt_vel_vector);
    // Set bounds
    // Upper bounds
    Eigen::Vector2d upper_bound(1.0, 1.0);
    Eigen::Vector2d lower_bound(-1.0, -1.0);
    orca_variables_ptr->SetBounds(lower_bound, upper_bound);

    // Create orca cost function
    std::shared_ptr<mrp_pure_pursuit::solver::Cost>
        orca_cost_ptr = std::make_shared<mrp_pure_pursuit::solver::Cost>();
    // Set optimal velocity for cost function (x - xopt)^2 + (y - yopt)^2
    orca_cost_ptr->SetOptimalVelocity(opt_vel_vector);

    // Create orca constraints
    std::shared_ptr<mrp_pure_pursuit::solver::Constraint>
        orca_constraint_ptr = std::make_shared<mrp_pure_pursuit::solver::Constraint>(orca_planes.size());
    orca_constraint_ptr->AddConstraints(orca_planes);

    // Create solver and solve for optimal velocity
    Eigen::Vector2d non_collision_velocity = mrp_pure_pursuit::solver::Solver::solve(
        orca_variables_ptr, orca_constraint_ptr, orca_cost_ptr);

    std::cout << non_collision_velocity(0) << std::endl;
    std::cout << non_collision_velocity(1) << std::endl;

    // Track this
    lookahead.transform.translation.x = non_collision_velocity(0);
    lookahead.transform.translation.y = non_collision_velocity(1);
    trackLookahead(current_odom.pose.pose, lookahead, false, vel_cmd);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
  }

  // For feedback
  double PurePursuitController::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
  }

  // For accessing
  bool PurePursuitController::reachGoal()
  {
    return goal_reached_;
  }

  // Should we abstract away the setting of parameters ?
  // Maybe not ?
  // For accessing ROS parameter server
  void PurePursuitController::setParameterInterface(std::shared_ptr<mrp_common::ParameterInterface> params_interface)
  {
  }

  void PurePursuitController::setParameter(const std::unordered_map<std::string, double> &param_map)
  {
  }

  // Protected

  void PurePursuitController::trackLookahead(
      const geometry_msgs::msg::Pose &current_pose,
      const geometry_msgs::msg::TransformStamped &lookahead,
      const bool &evaluate_linear_if_allow_reverse,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.

    // If this is a new waypoint, reevaluate linear velocity
    if (allow_reverse_)
    {
      if (evaluate_linear_if_allow_reverse)
      {
        if (reevaluate_linear_vel_)
        {
          // Get remaining path from the indx onwards
          std::vector<geometry_msgs::msg::PoseStamped> remaining_path(path_.begin() + current_waypoint_indx_, path_.end());
          // Forward sim with positive linear vel
          double pos_dis = forwardSim(current_pose,
                                      fabs(v_max_),
                                      remaining_path);
          double neg_dis = forwardSim(current_pose,
                                      -fabs(v_max_),
                                      remaining_path);
          reevaluate_linear_vel_ = false;
        }
      }
      else
      {
        reevaluate_linear_vel_ = true;
      }
    }
    std::cout << "Idx: " << current_waypoint_indx_ << std::endl;
    if (isApproachingFinal(path_, current_waypoint_indx_))
    {
      // We are approaching the goal,
      // This is the pose of the goal w.r.t. the base_link frame
      std::cout << "We are approaching the goal" << std::endl;
      KDL::Frame F_bl_end = transformToBaseLink(path_.back().pose, current_pose);

      if (fabs(F_bl_end.p.x()) <= pos_tol_)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        path_.clear();
      }
    }

    if (!goal_reached_)
    {
      // We are tracking.
      // Compute linear velocity.
      // Right now,this is not very smart :)
      v_ = copysign(v_max_, v_);

      // Compute the angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      double yt = lookahead.transform.translation.y;
      std::cout << "yt: " << yt << std::endl;
      double ld_2 = ld_ * ld_;
      vel_cmd.angular.z = std::min(2 * v_ / ld_2 * yt, w_max_);

      // Set linear velocity for tracking.
      vel_cmd.linear.x = v_;
    }
    else
    {
      // We are at the goal!
      // Stop the vehicle
      // Stop moving.
      std::cout << "Goal reached" << std::endl;
      vel_cmd.linear.x = 0.0;
      vel_cmd.angular.z = 0.0;
    }
  }

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame PurePursuitController::transformToBaseLink(
      const geometry_msgs::msg::Pose &pose,
      const geometry_msgs::msg::Pose &robot_tf)
  {
    // Pose in global (map) frame
    KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w),
                          KDL::Vector(pose.position.x,
                                      pose.position.y,
                                      pose.position.z));

    // Robot (base_link) in global (map) frame
    KDL::Frame F_map_tf(KDL::Rotation::Quaternion(robot_tf.orientation.x,
                                                  robot_tf.orientation.y,
                                                  robot_tf.orientation.z,
                                                  robot_tf.orientation.w),
                        KDL::Vector(robot_tf.position.x,
                                    robot_tf.position.y,
                                    robot_tf.position.z));
    return F_map_tf.Inverse() * F_map_pose;
  }

  double PurePursuitController::forwardSim(
      const geometry_msgs::msg::Pose &current_pose,
      const double &linear_vel,
      const std::vector<geometry_msgs::msg::PoseStamped> &remaining_path)
  {
    // Conduct a forward simulation to the future to see whether driving forward or backward is better
    // Basically rerun the entire controller in a for loop
    double end_time = 10;
    double control_freq = 0.1;
    double remaining_distance = 0;

    double current_x = current_pose.position.x;
    double current_y = current_pose.position.y;
    double current_yaw = tf2::getYaw(current_pose.orientation);
    double linear_x = linear_vel;
    double angular_z = 0;
    double sim_time = 0;
    double distance = 0;
    bool goal_reach = false;

    int local_indx = 0;
    geometry_msgs::msg::TransformStamped lookahead;
    while (sim_time < end_time)
    {
      // We extract the lookahead point of the next waypoint based on the current
      // pose of the robot
      local_indx = extractNextWaypoint(
          current_pose, local_indx, remaining_path,
          lookahead);

      if (local_indx >= remaining_path.size())
      {
        // We are approaching the goal,
        // which is closer than ld
        // This is the pose of the goal w.r.t. the base_link frame
        KDL::Frame F_bl_end = transformToBaseLink(remaining_path.back().pose, current_pose);

        if (fabs(F_bl_end.p.x()) <= pos_tol_)
        {
          // We have reached the goal
          goal_reach = true;
        }
      }

      if (!goal_reach)
      {
        // Compute linear velocity.
        // Right now,this is not very smart :)
        linear_x = copysign(linear_vel, linear_x);

        // Compute the angular velocity.
        // Lateral error is the y-value of the lookahead point (in base_link frame)
        double yt = lookahead.transform.translation.y;
        double ld_2 = ld_ * ld_;
        angular_z = std::min(2 * linear_x / ld_2 * yt, w_max_);
      }
      else
      {
        // We are at the goal!
        // Stop the vehicle
        lookahead.transform = geometry_msgs::msg::Transform();
        lookahead.transform.rotation.w = 1.0;

        // Stop moving.
        linear_x = 0.0;
        angular_z = 0.0;
      }

      Eigen::Vector3d local_vel(linear_x, 0, angular_z);
      Eigen::Vector3d vel_world = bodyToWorld(current_pose, local_vel);

      // Apply velocities
      current_x = current_x + vel_world(0) * control_freq;
      current_y = current_y + vel_world(1) * control_freq;
      current_yaw = current_yaw + vel_world(2) * control_freq;

      if (local_indx < remaining_path.size())
      {
        double target_x = remaining_path.at(local_indx).pose.position.x;
        double target_y = remaining_path.at(local_indx).pose.position.y;
        distance = sqrt(std::pow(target_x - current_x, 2) + std::pow(target_y - current_y, 2));
      }
      else
      {
        double target_x = remaining_path.back().pose.position.x;
        double target_y = remaining_path.back().pose.position.y;
        distance = sqrt(std::pow(target_x - current_x, 2) + std::pow(target_y - current_y, 2));
      }

      if (distance <= pos_tol_)
      {
        return distance;
      }
      sim_time += control_freq;
    }
    return distance;
  }

  int PurePursuitController::extractNextWaypoint(
      const geometry_msgs::msg::Pose &current_pose,
      const int &start_indx,
      const std::vector<geometry_msgs::msg::PoseStamped> &path,
      geometry_msgs::msg::TransformStamped &lookahead_tf)
  {
    int idx = start_indx;
    for (; idx < path.size(); idx++)
    {
      if (mrp_common::GeometryUtils::euclideanDistance(
              path.at(idx).pose, current_pose) > 0.05)
      {
        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = transformToBaseLink(path.at(idx).pose, current_pose);

        lookahead_tf.transform.translation.x = F_bl_ld.p.x();
        lookahead_tf.transform.translation.y = F_bl_ld.p.y();
        lookahead_tf.transform.translation.z = F_bl_ld.p.z();

        F_bl_ld.M.GetQuaternion(lookahead_tf.transform.rotation.x,
                                lookahead_tf.transform.rotation.y,
                                lookahead_tf.transform.rotation.z,
                                lookahead_tf.transform.rotation.w);
        break;
      }
    }
    return idx;
  }

  Eigen::Vector2d PurePursuitController::calculateOptimalVelocity(
      const geometry_msgs::msg::Pose &current_pose,
      const geometry_msgs::msg::Pose &current_waypoint)
  {
    double distance = mrp_common::GeometryUtils::euclideanDistance(current_pose, current_waypoint);

    double x1 = current_pose.position.x;
    double x2 = current_waypoint.position.x;
    double current_yaw = tf2::getYaw(current_pose.orientation);

    double y1 = current_pose.position.y;
    double y2 = current_waypoint.position.y;
    double target_yaw = tf2::getYaw(current_waypoint.orientation);

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

    return projected_vel;
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_pure_pursuit::PurePursuitController, mrp_local_server_core::MotionPlannerInterface)