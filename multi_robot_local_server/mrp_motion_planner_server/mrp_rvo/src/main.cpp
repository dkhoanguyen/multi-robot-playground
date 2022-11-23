#include <cmath>
#include <chrono>

#include "mrp_rvo/rvo.hpp"
#include "mrp_rvo/orca.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2/LinearMath/Quaternion.h"

int main(int argc, char **argv)
{
  mrp_motion_planner::RVO rvo;
  rvo.start();

  nav_msgs::msg::Odometry odom_A;
  odom_A.pose.pose.position.x = 0;
  odom_A.pose.pose.position.y = 0;
  odom_A.pose.pose.position.z = 0;

  tf2::Quaternion quad;
  quad.setRPY(0, 0, std::atan2(0.46, 0.92));
  tf2::convert(quad, odom_A.pose.pose.orientation);
  odom_A.twist.twist.linear.x = sqrt(std::pow(0.92, 2) + std::pow(0.46, 2));

  nav_msgs::msg::Odometry odom_B;
  odom_B.pose.pose.position.x = 4;
  odom_B.pose.pose.position.y = 1;
  odom_B.pose.pose.position.z = 0;
  quad.setRPY(0, 0, std::atan2(0.23, -1.21));
  tf2::convert(quad, odom_B.pose.pose.orientation);
  odom_B.twist.twist.linear.x = sqrt(std::pow(-1.22, 2) + std::pow(0.23, 2));

  double radius_A = 0.5;
  double radius_B = 0.5;
  double delta_tau = 1.75;
  double weight = 0.5;

  mrp_orca::ORCA orca;
  auto t_start = std::chrono::high_resolution_clock::now();
  mrp_orca::common::HalfPlane orca_p = orca.construct(odom_A, odom_B, radius_A, radius_B, delta_tau, weight);
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  std::cout << "Elapsed time (ms): " << elapsed_time_ms << std::endl;
  return 0;
}