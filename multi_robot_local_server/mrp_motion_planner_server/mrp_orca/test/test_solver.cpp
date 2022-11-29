#include <gtest/gtest.h>
#include "mrp_orca/geometry.hpp"
#include "mrp_orca/orca.hpp"
#include "mrp_orca/planner.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class ExposedPlanner : public mrp_orca::MotionPlanner
{
public:
  ExposedPlanner(){};
  ~ExposedPlanner(){};

  // Expose all protected functions
  double exposeCalculateLinearVelocity(
      const geometry_msgs::msg::Pose &current_pose,
      const geometry_msgs::msg::Pose &current_waypoint)
  {
    return calculateLinearVelocity(current_pose, current_waypoint);
  }

  double exposeCalculateAngularVelocity(
      const geometry_msgs::msg::Pose &current_pose,
      const geometry_msgs::msg::Pose &current_waypoint)
  {
    return calculateAngularVelocity(current_pose, current_waypoint);
  }

  Eigen::Vector2d exposeCalculateVelocityToTarget(
      const geometry_msgs::msg::Pose &current_pose,
      const geometry_msgs::msg::Pose &current_waypoint)
  {
    return calculateVelocityToTarget(current_pose, current_waypoint);
  }

  Eigen::Vector2d exposeCalculateOptimalVelocity(
      const geometry_msgs::msg::Pose &current_pose,
      const geometry_msgs::msg::Pose &current_waypoint)
  {
    return calculateOptimalVelocity(current_pose, current_waypoint);
  }

  Eigen::Vector2d exposeCalculateCmdVelFromVelVect(
      const Eigen::Vector2d &vel_vect,
      const geometry_msgs::msg::Pose &current_pose)
  {
    return calculateCmdVelFromVelVect(vel_vect, current_pose);
  }
};

class PlannerTest : public ::testing::Test
{
public:
  PlannerTest(){};
  ~PlannerTest(){};

protected:
  void SetUp() override
  {
  }

  ExposedPlanner planner;
};

TEST_F(PlannerTest, calcLinearVelocity)
{
  geometry_msgs::msg::Pose pose, waypoint;

  pose.position.x = 0;
  pose.position.y = 0;

  waypoint.position.x = 3;
  waypoint.position.y = 4;
  // EXPECT_EQ(planner.exposeCalculateLinearVelocity(pose, waypoint), 5);
}

TEST_F(PlannerTest, calcAngularVelocity)
{
  
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}