#include "mrp_pure_pursuit/pure_pursuit_controller.hpp"

namespace mrp_pure_pursuit
{
  PurePursuitController::PurePursuitController()
      : ld_(0.1), v_max_(0.05), v_(v_max_), w_max_(1.2), pos_tol_(0.01), idx_(0),
        goal_reached_(true), L_(0.1)
  {
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
  }
  void PurePursuitController::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      const sensor_msgs::msg::LaserScan &scan,
      const double &current_time,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_.size(); idx_++)
    {
      if (mrp_common::GeometryUtils::euclideanDistance(path_.at(idx_).pose, current_odom.pose.pose) > ld_)
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
        break;
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
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_pure_pursuit::PurePursuitController, mrp_local_server_core::MotionPlannerInterface)