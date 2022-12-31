#include "mrp_nmpc_orca/nmpc_path_tracker.hpp"
#include <iostream>

namespace mrp_nmpc_orca
{
  NMPCPathTracker::NMPCPathTracker()
  {
    // Control parameters
    control_sampling_time_ = 0.1;
    reference_speed_ = 0.1;

    // Path smoothing parameters
    curvature_smoothing_num_ = 10;
    max_curvature_change_rate_ = 1.0;
    speed_reduction_rate_ = 0.1;
    deceleration_rate_for_stop_ = 0.3;

    // Cgmres parameters
    cgmres_param_.Tf_ = 1.0;
    cgmres_param_.alpha_ = 0.5;
    cgmres_param_.N_ = 10;
    cgmres_param_.finite_distance_increment_ = 0.0002;
    cgmres_param_.zeta_ = 62.5;
    cgmres_param_.kmax_ = 5;

    // nmpc parameters
    mpc_param_.q_.at(MPC_STATE_SPACE::X_F) = 0.0;
    mpc_param_.q_.at(MPC_STATE_SPACE::Y_F) = 0.1;
    mpc_param_.q_.at(MPC_STATE_SPACE::YAW_F) = 0.1;
    mpc_param_.q_.at(MPC_STATE_SPACE::TWIST_X) = 0.1;

    mpc_param_.q_terminal_.at(MPC_STATE_SPACE::X_F) = 0.0;
    mpc_param_.q_terminal_.at(MPC_STATE_SPACE::Y_F) = 0.1;
    mpc_param_.q_terminal_.at(MPC_STATE_SPACE::YAW_F) = 0.1;
    mpc_param_.q_terminal_.at(MPC_STATE_SPACE::TWIST_X) = 0.1;

    mpc_param_.r_.at(MPC_INPUT::ANGULAR_VEL_YAW) = 0.01;
    mpc_param_.r_.at(MPC_INPUT::ACCEL) = 0.01;

    mpc_param_.a_max_ = 0.2;
    mpc_param_.a_min_ = -0.2;

    // Initialise CGMRES Solver
    nmpc_solver_ptr_ = std::make_unique<cgmres::ContinuationGMRES>(
        cgmres_param_.Tf_, cgmres_param_.alpha_, cgmres_param_.N_, cgmres_param_.finite_distance_increment_,
        cgmres_param_.zeta_, cgmres_param_.kmax_);
    const double solution_initial_guess[MPC_INPUT::DIM] = {0.01, 0.01};
    nmpc_solver_ptr_->setParametersForInitialization(solution_initial_guess, 1e-06, 50);
    nmpc_solver_ptr_->continuation_problem_.ocp_.model_.set_parameters(mpc_param_.q_, mpc_param_.q_terminal_,
                                                                       mpc_param_.r_, mpc_param_.barrier_coefficient_,
                                                                       mpc_param_.a_max_, mpc_param_.a_min_);

    // Intialise course manager
    course_manager_ptr_ = std::make_unique<pathtrack_tools::CourseManager>(
        curvature_smoothing_num_, max_curvature_change_rate_, speed_reduction_rate_, deceleration_rate_for_stop_);

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
  }

  void NMPCPathTracker::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      const sensor_msgs::msg::LaserScan &scan,
      const double &current_time,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    /*update robot pose global*/
    robot_status_.robot_pose_global_.x = current_odom.pose.pose.position.x;
    robot_status_.robot_pose_global_.y = current_odom.pose.pose.position.y;
    robot_status_.robot_pose_global_.z = current_odom.pose.pose.position.z; // not used now
    robot_status_.robot_pose_global_.roll = 0.0;                            // not used now
    robot_status_.robot_pose_global_.pitch = 0.0;                           // not used now
    robot_status_.robot_pose_global_.yaw = tf2::getYaw(current_odom.pose.pose.orientation);

    /*update robot twist*/
    robot_status_.robot_twist_.x = current_odom.twist.twist.linear.x;
    robot_status_.robot_twist_.y = current_odom.twist.twist.linear.y;
    robot_status_.robot_twist_.z = current_odom.twist.twist.linear.z;      // not used now
    robot_status_.robot_twist_.roll = current_odom.twist.twist.angular.x;  // not used now
    robot_status_.robot_twist_.pitch = current_odom.twist.twist.angular.y; // note used now
    robot_status_.robot_twist_.yaw = current_odom.twist.twist.angular.z;

    const int path_size = course_manager_ptr_->get_path_size();

    std::array<double, MPC_INPUT::DIM> control_input{0.0, 0.0};             // updated variables by mpc
    std::array<std::vector<double>, MPC_INPUT::DIM> control_input_series{}; // for visualization
    double F_norm = 0.0;                                                    // F_norm
    bool is_mpc_solved = calculate_mpc(&control_input, &control_input_series, &F_norm, current_time);

    // const double calculation_time = stop_watch_.lap();
    if (!is_mpc_solved)
    {
      Twist stop_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      // publish_twist(prev_twist_cmd_);
      // publish_twist(stop_twist);
      reset_cgmres({0.0, 0.0});
    }

    /*Publish twist cmd*/
    vel_cmd.linear.x = robot_status_.robot_twist_.x + control_input[MPC_INPUT::ACCEL] * control_sampling_time_;
    vel_cmd.angular.z = control_input[MPC_INPUT::ANGULAR_VEL_YAW];
  }

  bool NMPCPathTracker::calculate_mpc(std::array<double, MPC_INPUT::DIM> *control_input,
                                      std::array<std::vector<double>, MPC_INPUT::DIM> *control_input_series,
                                      double *F_norm, const double &current_time)
  {
    /*coordinate convert from euclid to frenet-serret*/
    robot_status_.robot_pose_frenet_ =
        frenet_serret_converter_.global2frenet(course_manager_ptr_->get_mpc_course(), robot_status_.robot_pose_global_);

    /*Set state vector*/
    std::vector<double> state_vec(MPC_STATE_SPACE::DIM);
    state_vec[MPC_STATE_SPACE::X_F] = robot_status_.robot_pose_frenet_.x_f;
    state_vec[MPC_STATE_SPACE::Y_F] = robot_status_.robot_pose_frenet_.y_f;
    state_vec[MPC_STATE_SPACE::YAW_F] = robot_status_.robot_pose_frenet_.yaw_f;
    state_vec[MPC_STATE_SPACE::TWIST_X] = robot_status_.robot_twist_.x;

    /*Control input*/
    double control_input_vec[MPC_INPUT::DIM] = {0.0, 0.0};

    bool is_mpc_solved = true;

    /*Solve NMPC by C/GMRES method*/
    if (!initial_solution_calculate_)
    {
      // The initial solution is calculated using Newton-GMRES method
      nmpc_solver_ptr_->initializeSolution(current_time, state_vec, path_curvature_, trajectory_speed_, drivable_width_);
      nmpc_solver_ptr_->getControlInput(control_input_vec);
      initial_solution_calculate_ = true;
    }
    else
    {
      // Update control_input_vec by C / GMRES method
      is_mpc_solved =
          nmpc_solver_ptr_->controlUpdate(current_time, state_vec, control_sampling_time_, path_curvature_,
                                          trajectory_speed_, drivable_width_, control_input_vec, control_input_series);

      // Update control input
      control_input->at(MPC_INPUT::ANGULAR_VEL_YAW) = control_input_vec[MPC_INPUT::ANGULAR_VEL_YAW];
      control_input->at(MPC_INPUT::ACCEL) = control_input_vec[MPC_INPUT::ACCEL];

      // Update F_norm
      *F_norm =
          nmpc_solver_ptr_->getErrorNorm(current_time, state_vec, path_curvature_, trajectory_speed_, drivable_width_);
    }

    /*NAN Guard*/
    if (std::isnan(control_input->at(MPC_INPUT::ANGULAR_VEL_YAW)) || std::isnan(control_input->at(MPC_INPUT::ACCEL)))
    {
      is_mpc_solved = false;
    }

    return is_mpc_solved;
  }

  void NMPCPathTracker::reset_cgmres(const std::array<double, MPC_INPUT::DIM> &solution_initial_guess)
  {
    nmpc_solver_ptr_.reset();
    nmpc_solver_ptr_ = std::make_unique<cgmres::ContinuationGMRES>(
        cgmres_param_.Tf_, cgmres_param_.alpha_, cgmres_param_.N_, cgmres_param_.finite_distance_increment_,
        cgmres_param_.zeta_, cgmres_param_.kmax_);

    const double initial_guess[MPC_INPUT::DIM] = {solution_initial_guess[MPC_INPUT::ANGULAR_VEL_YAW],
                                                  solution_initial_guess[MPC_INPUT::ACCEL]};
    nmpc_solver_ptr_->setParametersForInitialization(initial_guess, 1e-06, 50);
    nmpc_solver_ptr_->continuation_problem_.ocp_.model_.set_parameters(mpc_param_.q_, mpc_param_.q_terminal_,
                                                                       mpc_param_.r_, mpc_param_.barrier_coefficient_,
                                                                       mpc_param_.a_max_, mpc_param_.a_min_);

    initial_solution_calculate_ = false;
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