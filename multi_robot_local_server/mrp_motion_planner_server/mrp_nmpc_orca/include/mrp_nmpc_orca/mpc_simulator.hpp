#ifndef MPC_SIMULATOR_H
#define MPC_SIMULATOR_H

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>
#include "mrp_nmpc_orca/FrenetCoordinate.hpp"
#include "mrp_nmpc_orca/Pose.hpp"
#include "mrp_nmpc_orca/Twist.hpp"
#include "mrp_nmpc_orca/state_space_order.hpp"

// TODO : 制御入力の上下限，steerの不感帯，観測値に乗るノイズをつける
namespace pathtrack_tools {
///
/// @class MPCSimulator
/// @brief
///

class MPCSimulator {
 public:
  /**
   * @brief Default constructor
   *
   */
  MPCSimulator(const double &sampling_time);

  /**
   * @brief Destroy the VehicleSimulator object
   *
   */
  ~MPCSimulator();

  /**
   * @brief Update the ego car state with applied control input by numerical integration using the fourth-order
   * Runge-Kutta method.
   *
   * @param current_time
   * @param current_ego_pose_global
   * @param current_ego_twist
   * @param control_input_vec
   * @param sampling_time
   * @return std::tuple<Pose, Twist>
   */
  std::pair<Pose, Twist> update_ego_state(const double current_time, const Pose &current_ego_pose_global,
                                          const Twist &ego_twist, const double *control_input_vec,
                                          const double sampling_time);

  /**
   * @brief Reproduct predicted state in MPC from calculated control input series
   *
   * @param current_ego_pose_global
   * @param current_ego_twist
   * @param control_input_series
   * @param sampling_time
   * @return std::array<std::vector<double>, MPC_STATE_SPACE::DIM>
   */
  std::array<std::vector<double>, MPC_STATE_SPACE::DIM> reproduct_predivted_state(
      const Pose &current_ego_pose_global, const Twist &current_robot_twist,
      const std::array<std::vector<double>, MPC_INPUT::DIM> &control_input_series, const double &sampling_time) const;

 private:
  std::queue<double> accel_input_queue_;  // buffer for accel input to describe time delay
  std::queue<double> angle_input_queue_;  // buffer for tire angle input to describe time delay
  const double accel_delay_time_ = 0.01;  // time delay for accel input [s]
  const double angle_delay_time_ = 0.01;  // time delay for tire angle input [s]

  std::array<double, MPC_STATE_SPACE::DIM> ego_pose_to_state(const Pose &ego_pose, const Twist &twist) const;
  std::pair<Pose, Twist> ego_state_to_pose(const std::array<double, MPC_STATE_SPACE::DIM> &ego_state) const;

  /**
   * @brief Update the state by numerically integrating the state equation using the fourth-order Runge-Kutta method.
   *
   * @tparam StateArray
   * @tparam InputArray
   * @param current_time
   * @param current_state_array
   * @param control_input_array
   * @param sampling_time [s]
   * @param state_func
   * @return StateArray
   */
  template <typename StateArray, typename InputArray, typename StateFunc>
  StateArray runge_kutta_gill(const double current_time, const StateArray &current_state_array,
                              const InputArray &control_input_array, const double &sampling_time,
                              const StateFunc state_func) const {
    StateArray k1_vec, k2_vec, k3_vec, k4_vec, tmp_vec, updated_state_vec;

    k1_vec = state_func(current_time, current_state_array, control_input_array);
    for (size_t i = 0; i < current_state_array.size(); i++) {
      tmp_vec[i] = current_state_array[i] + 0.5 * sampling_time * k1_vec[i];
    }

    k2_vec = state_func(current_time + 0.5 * sampling_time, tmp_vec, control_input_array);
    for (size_t i = 0; i < current_state_array.size(); i++) {
      tmp_vec[i] = current_state_array[i] + sampling_time * 0.5 * (std::sqrt(2) - 1) * k1_vec[i] +
                   sampling_time * (1 - (1 / std::sqrt(2))) * k2_vec[i];
    }

    k3_vec = state_func(current_time + 0.5 * sampling_time, tmp_vec, control_input_array);
    for (size_t i = 0; i < current_state_array.size(); i++) {
      tmp_vec[i] = current_state_array[i] - sampling_time * 0.5 * std::sqrt(2) * k2_vec[i] +
                   sampling_time * (1 + (1 / std::sqrt(2))) * k3_vec[i];
    }

    k4_vec = state_func(current_time + sampling_time, tmp_vec, control_input_array);
    for (size_t i = 0; i < current_state_array.size(); i++) {
      updated_state_vec[i] =
          current_state_array[i] + (sampling_time / 6) * (k1_vec[i] + (2 - std::sqrt(2)) * k2_vec[i] +
                                                          (2 + std::sqrt(2)) * k3_vec[i] + k4_vec[i]);
    }

    return updated_state_vec;
  }

  /**
   * @brief Update the state by numerically integrating the state equation using eular method
   *
   * @tparam StateArray
   * @tparam InputArray
   * @tparam StateFunc
   * @param current_time
   * @param current_state_array
   * @param control_input_array
   * @param sampling_time
   * @param state_func
   * @return StateArray
   */
  template <typename StateArray, typename InputArray, typename StateFunc>
  StateArray eular(const double current_time, const StateArray &current_state_array,
                   const InputArray &control_input_array, const double &sampling_time,
                   const StateFunc state_func) const {
    StateArray updated_state_vec, dx_vec;

    dx_vec = state_func(current_time, current_state_array, control_input_array);

    for (size_t i = 0; i < current_state_array.size(); i++) {
      updated_state_vec[i] = current_state_array[i] + sampling_time * dx_vec[i];
    }
    return updated_state_vec;
  }

  /**
   * @brief Pack the buffer with elements for the delay_time_.
   *
   * @param sampling_time
   */
  void initialize_input_queue(const double &sampling_time);

  double delay_accel_input(const double &raw_accel);
  double delay_angle_input(const double &raw_angle);

  /**
   * @brief State function of 2 defferential wheel model without input delay
   *
   * @param [in] current_time
   * @param [in] x = [x_g, y_g, yaw_g]
   * @param [in] u = [twist_yaw, twist_x]
   * @return std::array<double, MPC_STATE_SPACE::DIM>
   */
  std::array<double, MPC_STATE_SPACE::DIM> two_dw_without_delay(const double &current_time,
                                                                const std::array<double, MPC_STATE_SPACE::DIM> &x,
                                                                const std::array<double, MPC_INPUT::DIM> &u) const;

  /**
   * @brief State function of 2 defferential wheel model with input delay
   *
   * @param [in] current_time
   * @param [in] x = [x_g, y_g, yaw_g]
   * @param [in] u = [twist_yaw, twist_x]
   * @return std::array<double, MPC_STATE_SPACE::DIM>
   */
  std::array<double, MPC_STATE_SPACE::DIM> two_dw_with_delay(const double &current_time,
                                                             const std::array<double, MPC_STATE_SPACE::DIM> &x,
                                                             const std::array<double, MPC_INPUT::DIM> &u);
};

}  // namespace pathtrack_tools

#endif
