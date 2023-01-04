#ifndef MRP_NMPC_ORCA__NMPC_PATH_TRACKER_HPP_
#define MRP_NMPC_ORCA__NMPC_PATH_TRACKER_HPP_

#include <atomic>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "mrp_local_server_core/local_motion_planner.hpp"
#include "mrp_common/parameter_interface.hpp"

#include "mrp_common/utils.hpp"

#include "mrp_nmpc_orca/course_manager.hpp"
#include "mrp_nmpc_orca/state_space_order.hpp"
#include "mrp_nmpc_orca/Twist.hpp"
#include "mrp_nmpc_orca/Pose.hpp"
#include "mrp_nmpc_orca/FrenetCoordinate.hpp"
#include "mrp_nmpc_orca/frenet_serret_converter.hpp"
#include "mrp_nmpc_orca/mpc_simulator.hpp"
#include "mrp_nmpc_orca/StopWatch.hpp"

#include "cgmres_solver/continuation_gmres.hpp"

namespace mrp_nmpc_orca
{
  class NMPCPathTracker : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    NMPCPathTracker();
    ~NMPCPathTracker();

    void initialise();
    void start();
    void stop();

    void setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);
    void calculateVelocityCommand(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<nav_msgs::msg::Odometry> &members_odom,
        const sensor_msgs::msg::LaserScan &scan,
        const double &current_time,
        geometry_msgs::msg::Twist &vel_cmd);

    // For feedback
    double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose);

    // For accessing
    bool reachGoal();

    // Should we abstract away the setting of parameters ?
    // Maybe not ?
    // For accessing ROS parameter server
    void setParameterInterface(std::shared_ptr<mrp_common::ParameterInterface> params_interface);

    void setParameter(const std::unordered_map<std::string, double> &param_map);

  protected:
    /*cgmres solver parameters*/
    struct CGMRESParam
    {
      double Tf_;                        //!< @brief Length of predictive horizon in C/GMRES [s]
      double alpha_;                     //!< @brief The length horizon at time t is given by T_f * (1-exp(-alpha*t))
      int N_;                            //!< @brief The number of discrete of the predictive horizon
      double finite_distance_increment_; //!< @brief Step length of the finite difference in C/GMRES method
      double zeta_;                      //!< @brief A parameter for stabilization of the C/GMRES method.
      int kmax_;                         //!< @brief dimension of the Krylov subspace and maximum iteration number
    };
    CGMRESParam cgmres_param_;

    // Planner
    std::shared_ptr<mrp_common::ParameterInterface> params_interface_;
    double robot_radius_;
    double observable_range_;
    double delta_tau_;

    int current_waypoint_indx_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    geometry_msgs::msg::Pose temporary_waypoint_;

    double max_linear_vel_;
    double max_angular_vel_;

    double linear_error_;
    double angular_error_;

    std::atomic<bool> at_position_;
    std::atomic<bool> reach_goal_;
    std::atomic<bool> moving_to_temp_;

    // Control system
    double control_sampling_time_;
    double reference_speed_;
    double current_time_;

    Pose ego_pose_global_;
    Twist robot_twist_;
    StopWatch stop_watch_;

    /*function used in the predictive horizon of MPC*/
    std::function<double(double)> path_curvature_; //!< @brief return curvature from pose x_f in frenet coordinate
    std::function<double(double)>
        trajectory_speed_;                         //!< @brief return reference speed from pose x_f in frenet coordinate
    std::function<double(double)> drivable_width_; // not used now

    /*Flags*/
    bool is_robot_state_ok_ = false;          //!< @brief Check getting robot observed info
    bool initial_solution_calculate_ = false; //!< @brief Initialize C/GMRES method using Newton-method
    bool is_finish_goal_ = false;             //!< @brief Check reached the goal
    bool is_permit_control_ = false;          //!< @brief Check control permission from higher level planner

    // Library
    std::unique_ptr<cgmres::ContinuationGMRES> nmpc_solver_ptr_; //!< @brief nonlinear mpc solver pointer
    std::unique_ptr<pathtrack_tools::CourseManager>
        course_manager_ptr_; //!< @brief Manage reference path, reference speed and drivable area in MPC
    pathtrack_tools::FrenetSerretConverter
        frenet_serret_converter_;                                      //!< @brief Converter between global coordinate and frenet-serret coordinate
    std::unique_ptr<pathtrack_tools::MPCSimulator> mpc_simulator_ptr_; //!< @brief Reproduct MPC predictive state
  };
}

#endif