#include "mrp_nmpc_orca/nmpc_path_tracker.hpp"
#include <iostream>

namespace mrp_nmpc_orca
{
  NMPCPathTracker::NMPCPathTracker()
  {
    // Control parameters
    control_sampling_time_ = 0.1;
    reference_speed_ = 0.15;

    // Cgmres parameters
    cgmres_param_.Tf_ = 1.0;
    cgmres_param_.alpha_ = 0.5;
    cgmres_param_.N_ = 10;
    cgmres_param_.finite_distance_increment_ = 0.0002;
    cgmres_param_.zeta_ = 10;
    cgmres_param_.kmax_ = 10;

    // Initialise CGMRES Solver
    nmpc_solver_ptr_ = std::make_unique<cgmres::ContinuationGMRES>(
        cgmres_param_.Tf_, cgmres_param_.alpha_, cgmres_param_.N_, cgmres_param_.finite_distance_increment_,
        cgmres_param_.zeta_, cgmres_param_.kmax_);
    const double solution_initial_guess[MPC_INPUT::DIM] = {0.01, 0.01};
    nmpc_solver_ptr_->setParametersForInitialization(solution_initial_guess, 1e-06, 50);

    // Intialise course manager
    course_manager_ptr_ = std::make_unique<pathtrack_tools::CourseManager>();

    // Set functions used in prediction horizon
    path_curvature_ = [this](const double &x_f)
    {
      return this->course_manager_ptr_->get_curvature(x_f);
    }; //!< @brief return curvature from pose x_f in frenet coordinate
    trajectory_speed_ = [this](const double &x_f)
    {
      return this->course_manager_ptr_->get_speed(x_f);
    }; //!< @brief return reference speed from pose x_f in frenet coordinate
    drivable_width_ = [this](const double &x_f)
    {
      return this->course_manager_ptr_->get_drivable_width(x_f);
    }; // not used now

    current_time_ = 0;

    ego_pose_global_.x = 0.0;
    ego_pose_global_.y = 0.0;
    ego_pose_global_.z = 0.0;
    ego_pose_global_.roll = 0.0;
    ego_pose_global_.pitch = 0.0;
    ego_pose_global_.yaw = 0.0;
    robot_twist_.x = 0.0;
    robot_twist_.y = 0.0;
    robot_twist_.yaw = 0.0;

    mpc_simulator_ptr_ = std::make_unique<pathtrack_tools::MPCSimulator>(control_sampling_time_);

    initial_solution_calculate_ = false;

    std::cout << "NMPC Initialised " << std::endl;
  }

  NMPCPathTracker::~NMPCPathTracker()
  {
  }

  void NMPCPathTracker::initialise()
  {
  }

  void NMPCPathTracker::start()
  {
  }
  void NMPCPathTracker::stop()
  {
  }

  void NMPCPathTracker::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    if (path.size() == 0)
    {
      course_manager_ptr_->clear_path();
    }
    path_ = path;
    current_waypoint_indx_ = 0;
    at_position_ = false;
    reach_goal_ = false;
    course_manager_ptr_->set_course_from_nav_msgs(path_, reference_speed_);
    current_time_ = 0;
  }

  void NMPCPathTracker::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      const sensor_msgs::msg::LaserScan &scan,
      const double &current_time,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // stop_watch_.lap();
    // State and Control input
    std::vector<double> current_state_vec_frenet(MPC_STATE_SPACE::DIM); // current state at Frenet coordinate
    double control_input_vec[MPC_INPUT::DIM];                           // calculated control input (tire_angle, accel)

    // coordinate convert
    const FrenetCoordinate ego_pose_frenet =
        frenet_serret_converter_.global2frenet(course_manager_ptr_->get_mpc_course(), ego_pose_global_);

    // Set state vector
    current_state_vec_frenet.at(MPC_STATE_SPACE::X_F) = ego_pose_frenet.x_f;
    current_state_vec_frenet.at(MPC_STATE_SPACE::Y_F) = ego_pose_frenet.y_f;
    current_state_vec_frenet.at(MPC_STATE_SPACE::YAW_F) = ego_pose_frenet.yaw_f;
    current_state_vec_frenet.at(MPC_STATE_SPACE::TWIST_X) = robot_twist_.x;

    std::array<std::vector<double>, MPC_INPUT::DIM> control_input_series;

    if (!initial_solution_calculate_)
    {
      // The initial solution is calculated using Newton-GMRES method
      nmpc_solver_ptr_->initializeSolution(current_time_, current_state_vec_frenet, path_curvature_, trajectory_speed_,
                                           drivable_width_);
      nmpc_solver_ptr_->getControlInput(control_input_vec);
      initial_solution_calculate_ = true;
    }
    else
    {
      std::cout << "Current time: " << current_time_ << std::endl;
      // Update control_input_vec by C/GMRES method
      const bool is_mpc_solved =
          nmpc_solver_ptr_->controlUpdate(current_time_, current_state_vec_frenet, control_sampling_time_, path_curvature_,
                                          trajectory_speed_, drivable_width_, control_input_vec, &control_input_series);
      if (!is_mpc_solved)
      {
        std::cerr << "Break Down C/GMRES Method" << std::endl;
        exit(-1);
      }
    }

    const auto [updated_ego_pose_global, updated_robot_twist] =
        mpc_simulator_ptr_->update_ego_state(current_time_, ego_pose_global_, robot_twist_, control_input_vec, control_sampling_time_);
    
    ego_pose_global_ = updated_ego_pose_global;
    robot_twist_ = updated_robot_twist;

    std::cout << "linear: " << robot_twist_.x << std::endl;
    std::cout << "angular: " << robot_twist_.yaw << std::endl;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    std::cout << "=============================" << std::endl;
    vel_cmd.linear.x = robot_twist_.x;
    vel_cmd.angular.z = robot_twist_.yaw;
    current_time_ += control_sampling_time_;
  }

  void NMPCPathTracker::setParameter(const std::unordered_map<std::string, double> &param_map)
  {
  }

  // For feedback
  double NMPCPathTracker::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
  }

  // For accessing
  bool NMPCPathTracker::reachGoal()
  {
    return false;
  }

  // Should we abstract away the setting of parameters ?
  // Maybe not ?
  // For accessing ROS parameter server
  void NMPCPathTracker::setParameterInterface(std::shared_ptr<mrp_common::ParameterInterface> params_interface)
  {
  }
} // namespace mrp

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_nmpc_orca::NMPCPathTracker, mrp_local_server_core::MotionPlannerInterface)