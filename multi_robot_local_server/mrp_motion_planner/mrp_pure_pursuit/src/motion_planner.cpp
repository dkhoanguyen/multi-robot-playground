#include "mrp_pure_pursuit/motion_planner.hpp"

namespace mrp_pure_pursuit
{
  MotionPlanner::MotionPlanner()
      : ld_(0.1), v_max_(0.05), v_(v_max_), w_max_(0.5),
        pos_tol_(0.005), current_waypoint_indx_(0),
        goal_reached_(true), L_(0.1), allow_reverse_(false),
        reevaluate_linear_vel_(true)
  {
    std::cout << "Intialisation mrp_pure_pursuit::MotionPlanner" << std::endl;
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
    current_waypoint_indx_ = 0;
    goal_reached_ = false;
    path_ = path;
    reevaluate_linear_vel_ = true;
  }

  void MotionPlanner::step(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<mrp_comms_msgs::msg::MemberState> &members_state,
      const sensor_msgs::msg::LaserScan &scan,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    // Get lookahead for the next waypoint
    int pre_indx = current_waypoint_indx_;
    geometry_msgs::msg::TransformStamped lookahead;

    // Extract waypoint lookahead and update next waypoint index
    current_waypoint_indx_ = extractNextWaypoint(
        current_odom.pose.pose, current_waypoint_indx_, path_,
        lookahead);

    bool evaluate_linear_vel_if_allow_reverse =
        (current_waypoint_indx_ != pre_indx && current_waypoint_indx_ < path_.size());

    trackLookahead(current_odom.pose.pose, lookahead,
                   evaluate_linear_vel_if_allow_reverse, vel_cmd);
  }

  // For feedback
  double MotionPlanner::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
    if (goal_reached_)
    {
      return 0;
    }

    if (path_.size() == 0)
    {
      return -1;
    }

    Eigen::Vector2d current_pos(current_pose.position.x, current_pose.position.y);
    Eigen::Vector2d goal(path_.back().pose.position.x, path_.back().pose.position.y);
    return (goal - current_pos).norm();
  }

  // For accessing
  bool MotionPlanner::reachGoal()
  {
    return goal_reached_;
  }

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame MotionPlanner::transformToBaseLink(
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

  double MotionPlanner::forwardSim(
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

  int MotionPlanner::extractNextWaypoint(
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

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_pure_pursuit::MotionPlanner, mrp_local_server_core::MotionPlannerInterface)