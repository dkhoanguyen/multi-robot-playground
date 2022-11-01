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
      // Only add to map if the key does not exist
      if (planner_name_map_.find(planner_names.at(idx)) == planner_name_map_.end())
      {
        planner_name_map_[planner_names.at(idx)] = planner_mapping.at(idx);
      }
    }

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
          "Unable to create motion planner action server for " + robot_name_);
    }

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Registering publisher for topic " + robot_cmd_vel_topic_name_);
    robot_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(robot_cmd_vel_topic_name_, 10);
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
    std::string local_string = robot_odom_topic_name_;
    robot_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        robot_odom_topic_name_, 10,
        [this, local_string](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          std::unique_lock<std::recursive_mutex> lck(robot_odom_mtx_);
          std::cout << "Message coming from " + robot_odom_topic_name_ << std::endl;
          robot_current_odom_ = *msg;
          robot_odom_ready_ = true;
        });

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Subscribing to odom of other robots beside this one");

    for (const std::string robot_name : other_robots_names_)
    {
      other_robots_odom_data_map_[robot_name] = std::make_shared<RobotOdom>();
      other_robots_odom_sub_map_[robot_name] = create_subscription<nav_msgs::msg::Odometry>(
          robot_name + "/odom", 10,
          [this, robot_name](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::unique_lock<std::recursive_mutex> lck(other_robots_odom_data_map_[robot_name]->mtx);
            other_robots_odom_data_map_[robot_name]->current_odom = *msg;
            other_robots_odom_data_map_[robot_name]->ready = true;
          });
    }
  }

  void MotionPlannerServer::followPath()
  {
    if (!robot_odom_ready_)
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