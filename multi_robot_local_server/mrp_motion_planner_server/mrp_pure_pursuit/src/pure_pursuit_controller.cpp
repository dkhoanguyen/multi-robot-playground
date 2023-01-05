#include "mrp_pure_pursuit/pure_pursuit_controller.hpp"

namespace mrp_pure_pursuit
{
  PurePursuitController::PurePursuitController()
      : ld_(0.3), v_max_(0.05), v_(v_max_), w_max_(1), pos_tol_(0.005), idx_(0),
        goal_reached_(true), L_(0.1)
  {
    reevaluate_linear_vel_ = true;
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
    idx_ = 0;
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
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_.size(); idx_++)
    {
      if (mrp_common::GeometryUtils::euclideanDistance(path_.at(idx_).pose, current_odom.pose.pose) > 0.05)
      {
        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = transformToBaseLink(path_.at(idx_).pose, current_odom.pose.pose);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);
        if (reevaluate_linear_vel_)
        {
          // Get remaining path from the indx onwards
          std::vector<geometry_msgs::msg::PoseStamped> remaining_path(path_.begin() + idx_, path_.end());
          // Forward sim with positive linear vel
          double pos_dis = forwardSim(current_odom.pose.pose,
                                      fabs(v_max_),
                                      remaining_path);
          double neg_dis = forwardSim(current_odom.pose.pose,
                                      -fabs(v_max_),
                                      remaining_path);
          std::cout << pos_dis << std::endl;
          std::cout << neg_dis << std::endl;
          if (neg_dis < pos_dis)
          {
            std::cout << "Use negative linear vel" << std::endl;
            v_ = -v_max_;
          }
          else
          {
            std::cout << "Use positive linear vel" << std::endl;
            v_ = v_max_;
          }
          reevaluate_linear_vel_ = false;
        }

        break;
      }
      else
      {
        reevaluate_linear_vel_ = true;
      }
    }

    if (!path_.empty() && idx_ >= path_.size())
    {
      // We are approaching the goal,
      // which is closer than ld

      // This is the pose of the goal w.r.t. the base_link frame
      KDL::Frame F_bl_end = transformToBaseLink(path_.back().pose, current_odom.pose.pose);

      if (fabs(F_bl_end.p.x()) <= pos_tol_)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        path_.clear();
      }
    }

    std::cout << "Goal reached: " << goal_reached_ << std::endl;
    if (!goal_reached_)
    {
      // We are tracking.
      // Compute linear velocity.
      // Right now,this is not very smart :)
      v_ = copysign(v_max_, v_);

      // Compute the angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      double yt = lookahead_.transform.translation.y;
      double ld_2 = ld_ * ld_;
      vel_cmd.angular.z = std::min(2 * v_ / ld_2 * yt, w_max_);

      // Set linear velocity for tracking.
      vel_cmd.linear.x = v_;
      std::cout << "Current idx: " << idx_ << std::endl;
    }
    else
    {
      // We are at the goal!

      // Stop the vehicle

      // The lookahead target is at our current pose.
      lookahead_.transform = geometry_msgs::msg::Transform();
      lookahead_.transform.rotation.w = 1.0;

      // Stop moving.
      vel_cmd.linear.x = 0.0;
      vel_cmd.angular.z = 0.0;
    }
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
      for (; local_indx < remaining_path.size(); local_indx++)
      {
        if (mrp_common::GeometryUtils::euclideanDistance(
                remaining_path.at(local_indx).pose, current_pose) > ld_)
        {
          // Transformed lookahead to base_link frame is lateral error
          KDL::Frame F_bl_ld = transformToBaseLink(remaining_path.at(local_indx).pose, current_pose);
          lookahead_.transform.translation.x = F_bl_ld.p.x();
          lookahead_.transform.translation.y = F_bl_ld.p.y();
          lookahead_.transform.translation.z = F_bl_ld.p.z();
          F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                  lookahead_.transform.rotation.y,
                                  lookahead_.transform.rotation.z,
                                  lookahead_.transform.rotation.w);
          break;
        }
      }

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
        double yt = lookahead_.transform.translation.y;
        double ld_2 = ld_ * ld_;
        angular_z = std::min(2 * linear_x / ld_2 * yt, w_max_);
      }
      else
      {
        // We are at the goal!

        // Stop the vehicle

        // The lookahead target is at our current pose.
        lookahead_.transform = geometry_msgs::msg::Transform();
        lookahead_.transform.rotation.w = 1.0;

        // Stop moving.
        linear_x = 0.0;
        angular_z = 0.0;
      }

      // Apply velocities
      current_x = current_x + linear_x * cos(current_yaw) * control_freq;
      current_y = current_y + linear_x * sin(current_yaw) * control_freq;
      current_yaw = current_yaw + angular_z * control_freq;

      std::cout << "Current x: " << current_x << " - y: " << current_y << std::endl;

      double target_x = remaining_path.at(local_indx).pose.position.x;
      double target_y = remaining_path.at(local_indx).pose.position.y;
      distance = sqrt(std::pow(target_x - current_x, 2) + std::pow(target_y - current_y, 2));
      std::cout << "Distance: " << distance << std::endl;
      if (distance <= pos_tol_)
      {
        return distance;
      }
      sim_time += control_freq;
    }
    std::cout << "===" << std::endl;
    return distance;
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_pure_pursuit::PurePursuitController, mrp_local_server_core::MotionPlannerInterface)