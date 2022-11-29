#include <gtest/gtest.h>
#include "mrp_orca/geometry.hpp"
#include "mrp_orca/orca.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "mrp_orca/planner.hpp"

TEST(ORCAGeometry, LineConstructionTest)
{
  // First line
  // x = 0
  Eigen::Vector2d vector1(1, 0);
  Eigen::Vector2d point1(0, 0);
  mrp_orca::geometry::Line line1(vector1, point1);

  EXPECT_EQ(line1.a(), 1);
  EXPECT_EQ(line1.b(), 0);
  EXPECT_EQ(line1.c(), 0);

  // Second line
  // 2x + 3y = 5
  Eigen::Vector2d vector2(2, 3);
  Eigen::Vector2d point2(1, 1);
  mrp_orca::geometry::Line line2(vector2, point2);

  EXPECT_EQ(line2.a(), 2);
  EXPECT_EQ(line2.b(), 3);
  EXPECT_EQ(line2.c(), 5);
}

// TEST(ORCANoCollision, NoCollisionAtAll)
// {
//   nav_msgs::msg::Odometry odom_A;
//   // Position A vector [0,0]
//   odom_A.pose.pose.position.x = 0;
//   odom_A.pose.pose.position.y = 0;
//   odom_A.pose.pose.position.z = 0;

//   // Velocity A vector [0.92, -0.46]
//   Eigen::Vector2d vel_vect_A(0.92, -0.46);
//   tf2::Quaternion quad;
//   quad.setRPY(0, 0, std::atan2(vel_vect_A(1), vel_vect_A(0)));
//   tf2::convert(quad, odom_A.pose.pose.orientation);
//   odom_A.twist.twist.linear.x = vel_vect_A.norm();

//   nav_msgs::msg::Odometry odom_B;
//   // Position B vector [4,1]
//   odom_B.pose.pose.position.x = 4;
//   odom_B.pose.pose.position.y = 1;
//   odom_B.pose.pose.position.z = 0;

//   // Velocity B vector [-1.22,0.23]
//   Eigen::Vector2d vel_vect_B(-1.22, 0.23);
//   quad.setRPY(0, 0, std::atan2(vel_vect_B(1), vel_vect_B(0)));
//   tf2::convert(quad, odom_B.pose.pose.orientation);
//   odom_B.twist.twist.linear.x = vel_vect_B.norm();

//   double radius_A = 0.5;
//   double radius_B = 0.5;
//   double delta_tau = 2;
//   double weight = 0.5;

//   mrp_orca::geometry::HalfPlane orca_p;
//   bool result = mrp_orca::ORCA::construct(orca_p, odom_A, odom_B, radius_A, radius_B, delta_tau, weight);
//   EXPECT_EQ(result, false);
// }

// TEST(ORCANoCollision, NoImmediateCollision)
// {
//   nav_msgs::msg::Odometry odom_A;
//   // Position A vector [0,0]
//   odom_A.pose.pose.position.x = 0;
//   odom_A.pose.pose.position.y = 0;
//   odom_A.pose.pose.position.z = 0;

//   // Velocity A vector [0.92, -0.46]
//   Eigen::Vector2d vel_vect_A(0.92 * 0.5, -0.46 * 0.5);
//   tf2::Quaternion quad;
//   quad.setRPY(0, 0, std::atan2(vel_vect_A(1), vel_vect_A(0)));
//   tf2::convert(quad, odom_A.pose.pose.orientation);
//   odom_A.twist.twist.linear.x = vel_vect_A.norm();

//   nav_msgs::msg::Odometry odom_B;
//   // Position B vector [4,1]
//   odom_B.pose.pose.position.x = 4;
//   odom_B.pose.pose.position.y = 1;
//   odom_B.pose.pose.position.z = 0;

//   // Velocity B vector [-1.22,0.23]
//   Eigen::Vector2d vel_vect_B(-1.22 * 0.5, 0.23 * 0.5);
//   quad.setRPY(0, 0, std::atan2(vel_vect_B(1), vel_vect_B(0)));
//   tf2::convert(quad, odom_B.pose.pose.orientation);
//   odom_B.twist.twist.linear.x = vel_vect_B.norm();

//   double radius_A = 0.5;
//   double radius_B = 0.5;
//   double delta_tau = 2;
//   double weight = 0.5;

//   mrp_orca::geometry::HalfPlane orca_p;
//   bool result = mrp_orca::ORCA::construct(orca_p, odom_A, odom_B, radius_A, radius_B, delta_tau, weight);
//   EXPECT_EQ(result, false);
// }

// TEST(ORCACollision, Collision)
// {
//   nav_msgs::msg::Odometry odom_A;
//   // Position A vector [0,0]
//   odom_A.pose.pose.position.x = 0;
//   odom_A.pose.pose.position.y = 0;
//   odom_A.pose.pose.position.z = 0;

//   // Velocity A vector [0.92, -0.46]
//   Eigen::Vector2d vel_vect_A(0.92, 0.46);
//   tf2::Quaternion quad;
//   quad.setRPY(0, 0, std::atan2(vel_vect_A(1), vel_vect_A(0)));
//   tf2::convert(quad, odom_A.pose.pose.orientation);
//   odom_A.twist.twist.linear.x = vel_vect_A.norm();

//   nav_msgs::msg::Odometry odom_B;
//   // Position B vector [4,1]
//   odom_B.pose.pose.position.x = 4;
//   odom_B.pose.pose.position.y = 1;
//   odom_B.pose.pose.position.z = 0;

//   // Velocity B vector [-1.22,0.23]
//   Eigen::Vector2d vel_vect_B(-1.22, 0.23);
//   quad.setRPY(0, 0, std::atan2(vel_vect_B(1), vel_vect_B(0)));
//   tf2::convert(quad, odom_B.pose.pose.orientation);
//   odom_B.twist.twist.linear.x = vel_vect_B.norm();

//   double radius_A = 0.5;
//   double radius_B = 0.5;
//   double delta_tau = 2;
//   double weight = 0.5;

//   mrp_orca::geometry::HalfPlane orca_p;
//   bool result = mrp_orca::ORCA::construct(orca_p, odom_A, odom_B, radius_A, radius_B, delta_tau, weight);
//   EXPECT_EQ(result, true);
// }

TEST(ORCACollision, CorrectORCAPlane)
{
  nav_msgs::msg::Odometry odom_A;
  // Position A vector [0,0]
  odom_A.pose.pose.position.x = 0;
  odom_A.pose.pose.position.y = 0;
  odom_A.pose.pose.position.z = 0;

  // Velocity A vector [0.92, -0.46]
  Eigen::Vector2d vel_vect_A(0.099, 0);
  tf2::Quaternion quad;
  quad.setRPY(0, 0, std::atan2(vel_vect_A(1), vel_vect_A(0)));
  tf2::convert(quad, odom_A.pose.pose.orientation);
  odom_A.twist.twist.linear.x = vel_vect_A.norm();

  nav_msgs::msg::Odometry odom_B;
  // Position B vector [4,1]
  odom_B.pose.pose.position.x = 2 - 1.903;
  odom_B.pose.pose.position.y = 0;
  odom_B.pose.pose.position.z = 0;

  // Velocity B vector [-1.22,0.23]
  Eigen::Vector2d vel_vect_B(0, 0);
  quad.setRPY(0, 0, std::atan2(vel_vect_B(1), vel_vect_B(0)));
  tf2::convert(quad, odom_B.pose.pose.orientation);
  odom_B.twist.twist.linear.x = vel_vect_B.norm();

  double radius_A = 0.05;
  double radius_B = 0.05;
  double delta_tau = 2.25;
  double weight = 1;

  mrp_orca::geometry::HalfPlane orca_p;
  bool result = mrp_orca::ORCA::construct(orca_p, odom_A, odom_B, radius_A, radius_B, delta_tau, weight);
  // EXPECT_EQ(result, true);
  //   EXPECT_DOUBLE_EQ(orca_p.line().normal()(0), 0);
  //   EXPECT_DOUBLE_EQ(orca_p.line().normal()(1), -0.115);
  //   EXPECT_DOUBLE_EQ(orca_p.line().point()(0), 0.92);
  //   EXPECT_DOUBLE_EQ(orca_p.line().point()(1), 0.3450);
}

TEST(ORCACollision, PlanVelocity)
{
  nav_msgs::msg::Odometry odom_A;
  // Position A vector [0,0]
  odom_A.pose.pose.position.x = 0;
  odom_A.pose.pose.position.y = 0;
  odom_A.pose.pose.position.z = 0;

  // Velocity A vector [0.92, -0.46]
  Eigen::Vector2d vel_vect_A(0.099, 0);
  tf2::Quaternion quad;
  quad.setRPY(0, 0, std::atan2(vel_vect_A(1), vel_vect_A(0)));
  tf2::convert(quad, odom_A.pose.pose.orientation);
  odom_A.twist.twist.linear.x = vel_vect_A.norm();

  nav_msgs::msg::Odometry odom_B;
  // Position B vector [4,1]
  odom_B.pose.pose.position.x = 2 - 1.7503;
  odom_B.pose.pose.position.y = 0;
  odom_B.pose.pose.position.z = 0;

  std::vector<nav_msgs::msg::Odometry> member_odoms = {odom_B};

  // Velocity B vector [-1.22,0.23]
  Eigen::Vector2d vel_vect_B(0, 0);
  quad.setRPY(0, 0, std::atan2(vel_vect_B(1), vel_vect_B(0)));
  tf2::convert(quad, odom_B.pose.pose.orientation);
  odom_B.twist.twist.linear.x = vel_vect_B.norm();

  double radius_A = 0.05;
  double radius_B = 0.05;
  double delta_tau = 2.25;
  double weight = 1;

  geometry_msgs::msg::Twist cmd_vel;
  sensor_msgs::msg::LaserScan scan;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  geometry_msgs::msg::PoseStamped waypoint;

  waypoint.pose.position.x = 2;

  path.push_back(waypoint);

  mrp_orca::MotionPlanner planner;
  planner.setPath(path);

  planner.calculateVelocityCommand(
      odom_A, member_odoms, scan, cmd_vel);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}