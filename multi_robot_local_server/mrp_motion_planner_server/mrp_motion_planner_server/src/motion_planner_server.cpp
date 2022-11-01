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

    declare_parameter<std::vector<std::string>>("planner_name_list", std::vector<std::string>());
    declare_parameter<std::vector<std::string>>("planner_name_obj_mapping", std::vector<std::string>());

    std::vector<std::string> planner_names = get_parameter("planner_name_list").as_string_array();
    std::vector<std::string> planner_mapping = get_parameter("planner_name_obj_mapping").as_string_array();
    robot_name_ = get_namespace();
    robot_cmd_vel_topic_name_ = robot_name_ + "/cmd_vel";
    robot_odom_topic_name_ = robot_name_ + "/odom";

    // Extract planner names and definition from param server
    for (unsigned int idx; idx < planner_names.size(); idx++)
    {
      // Only add to map if the key does not exist
      if (planner_name_map_.find(planner_names.at(idx)) == planner_name_map_.end())
      {
        planner_name_map_[planner_names.at(idx)] = planner_mapping.at(idx);
      }
    }

    // Create cmd_vel subscriber
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Registering publisher for topic " + robot_cmd_vel_topic_name_);
    robot_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(robot_cmd_vel_topic_name_, 10);

    // Create get_all_teams service client
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating service client for getting all available teams");
    try
    {
      get_team_client_ = std::make_shared<mrp_common::ServiceClient<mrp_comms_msgs::srv::GetAllTeams>>(
          shared_from_this(),
          false, // Do not spin as an isolated thread
          robot_name_ + "get_all_teams",
          rcl_service_get_default_options());
    }
    catch (const std::exception &e)
    {
      mrp_common::Log::basicError(
          get_node_logging_interface(),
          "Unable to create get team service client for " + robot_name_ + " due to err " + std::string(e.what()));
    }

    // Create follow_path action server
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating motion planner action server for " + robot_name_);
    try
    {
      follow_path_action_server_ = std::make_shared<mrp_common::ActionServer<nav2_msgs::action::FollowPath>>(
          shared_from_this(),
          "follow_path",
          nullptr,
          nullptr,
          std::chrono::milliseconds(1000),
          false,
          rcl_action_server_get_default_options());
      mrp_common::Log::basicInfo(
          get_node_logging_interface(),
          "Motion planner action server for " + robot_name_ + " created successfully!");
    }
    catch (const std::exception &e) // Maybe it's better not to catch all exception here
    {
      mrp_common::Log::basicError(
          get_node_logging_interface(),
          "Unable to create motion planner action server for " + robot_name_ + " due to err " + std::string(e.what()));
    }
  }

  void MotionPlannerServer::start()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Activating cmd_vel publisher");
    robot_cmd_vel_pub_->on_activate();

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Subscribing to robot odom for this robot");
    robot_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        robot_odom_topic_name_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          std::unique_lock<std::recursive_mutex> lck(robot_odom_->mtx);
          robot_odom_->current_odom = *msg;
          robot_odom_->ready = true;
        });

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Subscribing to odom of other robots beside this one");

    for (const std::string robot_name : member_robots_names_)
    {
      member_robots_odom_data_map_[robot_name] = std::make_shared<RobotOdom>();
      member_robots_odom_sub_map_[robot_name] = create_subscription<nav_msgs::msg::Odometry>(
          robot_name + "/odom", 10,
          [this, robot_name](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            std::unique_lock<std::recursive_mutex> lck(member_robots_odom_data_map_[robot_name]->mtx);
            member_robots_odom_data_map_[robot_name]->current_odom = *msg;
            member_robots_odom_data_map_[robot_name]->ready = true;
          });
    }
  }

  void MotionPlannerServer::setOtherRobotNames(const std::vector<std::string> &robot_names)
  {
    member_robots_names_ = robot_names;
  }

  void MotionPlannerServer::getAllTeams(std::vector<int> &team_id_list)
  {
    mrp_comms_msgs::srv::GetAllTeams::Request::SharedPtr request =
        std::make_shared<mrp_comms_msgs::srv::GetAllTeams::Request>();
    mrp_comms_msgs::srv::GetAllTeams::Response::SharedPtr response =
        std::make_shared<mrp_comms_msgs::srv::GetAllTeams::Response>();

    // This should block
    try
    {
      get_team_client_->requestAndWaitForResponse(request, response);
      team_id_list = std::vector<int>(response->id_list.begin(), response->id_list.end());
    }
    catch (std::exception &e)
    {
      mrp_common::Log::basicFatal(
          get_node_logging_interface(),
          "Unable to request list of teams due to error " + std::string(e.what()));
    }
  }

  void MotionPlannerServer::getAllMembersInTeam(const int &team_id, std::vector<std::string> &member_names)
  {
    mrp_comms_msgs::srv::GetMembersInTeam::Request::SharedPtr request =
        std::make_shared<mrp_comms_msgs::srv::GetMembersInTeam::Request>();
    mrp_comms_msgs::srv::GetMembersInTeam::Response::SharedPtr response =
        std::make_shared<mrp_comms_msgs::srv::GetMembersInTeam::Response>();
    request->team_id = team_id;

    try
    {
      get_all_robots_in_team_client_->requestAndWaitForResponse(request, response);
      member_names = response->member_name_list;
    }
    catch (const std::exception &e)
    {
      mrp_common::Log::basicFatal(
          get_node_logging_interface(),
          "Unable to request list of member in team " + std::to_string(team_id) + " due to error " + std::string(e.what()));
    }
  }

  void MotionPlannerServer::followPath()
  {
    if (!robot_odom_->ready)
    {
      return;
    }

    std::shared_ptr<const nav2_msgs::action::FollowPath::Goal> current_goal =
        follow_path_action_server_->getCurrentGoal();
    std::vector<geometry_msgs::msg::PoseStamped> path = current_goal->path.poses;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MotionPlannerServer::on_configure(const rclcpp_lifecycle::State &state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MotionPlannerServer::on_activate(const rclcpp_lifecycle::State &state)
  {
    start();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
}