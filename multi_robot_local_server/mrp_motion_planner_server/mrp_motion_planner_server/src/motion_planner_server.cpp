#include "mrp_motion_planner_server/motion_planner_server.hpp"

namespace mrp_motion_planner
{
  MotionPlannerServer::MotionPlannerServer(const std::string &default_planner_name)
      : mrp_common::LifecycleNode::LifecycleNode(
            "motion_planner_server",
            "robot", true, true, std::chrono::milliseconds(1000))
  {
    loader_ptr_ = std::make_shared<pluginlib::ClassLoader<mrp_local_server_core::MotionPlannerInterface>>(
        "mrp_local_server_core", "mrp_local_server_core::MotionPlannerInterface");
    planner_name_ = default_planner_name;
  }

  MotionPlannerServer::~MotionPlannerServer()
  {
  }

  void MotionPlannerServer::initialise()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Extracting parameters from parameter server");

    declare_parameter<std::vector<std::string>>("planner_name", std::vector<std::string>());
    declare_parameter<std::vector<std::string>>("planner_mapping", std::vector<std::string>());

    std::vector<std::string> planner_names = get_parameter("planner_name").as_string_array();
    std::vector<std::string> planner_mapping = get_parameter("planner_mapping").as_string_array();
    robot_name_ = get_namespace();
    robot_cmd_vel_topic_name_ = robot_name_ + "/cmd_vel";
    robot_odom_topic_name_ = robot_name_ + "/odom";
    
    // Extract planner names and definition from param server
    for (unsigned int idx; idx < planner_names.size(); idx++)
    {
      planner_name_map_[planner_names.at(idx)] = planner_mapping.at(idx);
    }

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating motion planner action server for " + robot_name_);
    follow_path_action_server_ = std::make_shared<mrp_common::ActionServer<nav2_msgs::action::FollowPath>>(
      shared_from_this(),
      "follow_path",
      std::bind(&MotionPlannerServer::followPath,this),
      nullptr,
      std::chrono::milliseconds(1000),
      false,
      rcl_action_server_get_default_options());
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Motion planner action server for " + robot_name_ + " created successfully!");
  }

  void MotionPlannerServer::start()
  {
    mrp_common::Log::basicInfo(
      get_node_logging_interface(),
      "Registering publisher for topic " + robot_cmd_vel_topic_name_;
    )
  }

  void MotionPlannerServer::followPath()
  {

  }
}