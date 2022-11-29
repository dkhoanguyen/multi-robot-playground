#include "mrp_motion_planner_server/motion_planner_server.hpp"

namespace mrp_motion_planner
{
  const std::string MotionPlannerServer::FALLBACK_PLANNER = "orca";
  MotionPlannerServer::MotionPlannerServer(const std::string &planner_name)
      : mrp_common::LifecycleNode::LifecycleNode(
            "motion_planner_server",
            "robot", true, true, std::chrono::milliseconds(1000))
  {
    loader_ptr_ = std::make_shared<pluginlib::ClassLoader<mrp_local_server_core::MotionPlannerInterface>>(
        "mrp_local_server_core", "mrp_local_server_core::MotionPlannerInterface");
    planner_name_ = planner_name;

    planner_ptr_ = loader_ptr_->createSharedInstance("mrp_orca::MotionPlanner");

    // Get all available plugins for planner
    declare_parameter<std::vector<std::string>>("planner_name_list", std::vector<std::string>());
    declare_parameter<std::vector<std::string>>("planner_name_plugin_mapping", std::vector<std::string>());

    // Motion planner params
    declare_parameter<double>("planner_rate", 100.0); // Planner rate
    declare_parameter<std::string>("planner_plugin", "orca");

    robot_odom_ = std::make_shared<RobotOdom>();
    robot_odom_->ready = false;

    robot_scan_ = std::make_shared<RobotLaserScan>();
    robot_scan_->ready = false;
  }

  MotionPlannerServer::~MotionPlannerServer()
  {
  }

  // ============================================ //
  //                                              //
  // ============================================ //
  // === INIT === //
  bool MotionPlannerServer::initialise()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Extracting parameters from parameter server");

    std::vector<std::string> planner_names = get_parameter("planner_name_list").as_string_array();
    std::vector<std::string> planner_mapping = get_parameter("planner_name_plugin_mapping").as_string_array();
    planner_rate_ = std::chrono::milliseconds((int)get_parameter("planner_rate").as_double());

    robot_name_ = get_namespace();
    robot_cmd_vel_topic_name_ = robot_name_ + "/cmd_vel";
    robot_odom_topic_name_ = robot_name_ + "/odom";
    robot_scan_topic_name_ = robot_name_ + "/scan";

    // Extract planner names and definition from param server
    for (unsigned int idx = 0; idx < planner_names.size(); idx++)
    {
      // Only add to map if the key does not exist
      if (planner_name_map_.find(planner_names.at(idx)) == planner_name_map_.end())
      {
        mrp_common::Log::basicDebug(
            get_node_logging_interface(),
            "Adding " + planner_mapping.at(idx) + " to planner map");
        planner_name_map_[planner_names.at(idx)] = planner_mapping.at(idx);
      }
    }

    // Create cmd_vel subscriber
    createCmdVelPublisher();

    // Create get_all_teams service client
    createGetTeamClient();

    // Create get_all_member service client
    createGetMemberInTeamClient();

    // Create follow_path action server
    createFollowPathActionServer();

    // Initialise heartbeat
    initialiseHeartbeat();

    // Should we load planner here ?
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Motion planner server initialises successfully");
    return true;
  }

  // === START === //
  bool MotionPlannerServer::start()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Activating cmd_vel publisher");
    robot_cmd_vel_pub_->on_activate();

    createOdomSubscriber();

    createLaserScanSubscriber();

    // Request list of all member robots in the current team
    registerMemberRobots();

    // Activating follow_path action server
    follow_path_action_server_->activate();

    // Start beating
    startHeartbeat();

    return true;
  }

  // === STOP === //
  bool MotionPlannerServer::stop()
  {
    follow_path_action_server_->deactivate();
    publishZeroVelocity();
    robot_cmd_vel_pub_->on_deactivate();

    // Stop heartbeat
    stopHeartbeat();
    return true;
  }

  // ============================================ //
  //           LIFECYCLE RELATED                  //
  // ============================================ //

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MotionPlannerServer::on_configure(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Initialising motion planner server");
    if (!initialise())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    std::cout << "Yooyoy" << std::endl;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MotionPlannerServer::on_activate(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Activating motion planner server");
    if (!start())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MotionPlannerServer::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Deactivating motion planner server");
    if (!stop())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MotionPlannerServer::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Cleaning up motion planner server");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MotionPlannerServer::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Shutting down motion planner server");

    // Clean up all pointers
    loader_ptr_.reset();
    planner_ptr_.reset();
    robot_odom_.reset();
    robot_cmd_vel_pub_.reset();
    robot_odom_sub_.reset();

    for (auto element : member_robots_odom_sub_map_)
    {
      element.second.reset();
    }
    for (auto element : member_robots_odom_data_map_)
    {
      element.second.reset();
    }

    // Clean up all services and actions
    get_team_client_.reset();
    get_all_robots_in_team_client_.reset();
    follow_path_action_server_.reset();

    heartbeat_ptr_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // ============================================ //
  //                                              //
  // ============================================ //

  bool MotionPlannerServer::loadPlanner(const std::string &planner_name)
  {
    planner_name_ = planner_name;
    try
    {
      mrp_common::Log::basicInfo(
          get_node_logging_interface(),
          "Creating " + planner_name_);
      planner_ptr_ = loader_ptr_->createSharedInstance(planner_name_map_[planner_name_]);
      mrp_common::Log::basicInfo(
          get_node_logging_interface(),
          "Initialing " + planner_name_);
      planner_ptr_->initialise();
      mrp_common::Log::basicInfo(
          get_node_logging_interface(),
          "Successfully initialised " + planner_name_);
    }
    catch (pluginlib::PluginlibException &ex)
    {
      return false;
    }
    return true;
  }

  void MotionPlannerServer::setMemberRobotNames(const std::vector<std::string> &robot_names)
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
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Begin computing control command");
    try
    {
      // Load planner first
      if (!loadPlanner(planner_name_))
      {
        mrp_common::Log::basicError(
            get_node_logging_interface(),
            "Unable to load planner " + planner_name_ +
                ". Fall back to defaul planner " + MotionPlannerServer::FALLBACK_PLANNER);
        loadPlanner(MotionPlannerServer::FALLBACK_PLANNER);
      }

      mrp_common::Log::basicInfo(
          get_node_logging_interface(),
          "Setting path");
      // Set path
      planner_ptr_->setPath(follow_path_action_server_->getCurrentGoal()->path.poses);

      rclcpp::WallRate loop_rate(planner_rate_);
      while (rclcpp::ok())
      {
        // Follow path should be a blocking function until we finish the path
        // Can't start until we have odom data of ourselves
        if (!robot_odom_->ready)
        {
          mrp_common::Log::basicWarn(
              get_node_logging_interface(),
              "Robot odom is not ready");
          loop_rate.sleep();
          continue;
        }

        // We can't start until we receive all odom data from other team members
        if (!all_members_odom_ready_)
        {
          bool reset_loop = false;
          for (const std::string &robot_name : member_robots_names_)
          {
            if (!member_robots_odom_data_map_[robot_name]->ready)
            {
              mrp_common::Log::basicWarn(
                  get_node_logging_interface(),
                  "Member robot odom is not ready");
              loop_rate.sleep();
              all_members_odom_ready_ = false;
              reset_loop = true;
              continue;
            }
          }
          if (reset_loop)
          {
            loop_rate.sleep();
            continue;
          }
          all_members_odom_ready_ = true;
        }

        if (!robot_scan_->ready)
        {
          mrp_common::Log::basicWarn(
              get_node_logging_interface(),
              "Robot scan is not ready");
          loop_rate.sleep();
          continue;
        }

        // If action server is inactive -> terminate
        if (follow_path_action_server_ == nullptr || !follow_path_action_server_->isServerActive())
        {
          mrp_common::Log::basicError(
              get_node_logging_interface(),
              "Follow path action server is inactive. Stopping now");
          publishZeroVelocity();
          return;
        }

        // If cancel request is received
        if (follow_path_action_server_->isCancelRequested())
        {
          mrp_common::Log::basicInfo(
              get_node_logging_interface(),
              "Goal was canceled. Stopping now");
          follow_path_action_server_->terminateAll();
          publishZeroVelocity();
          return;
        }

        // Check to see if there is any new path request coming

        //=== Start computing and publishing control command ===//
        computeAndPublishVelocity();

        // Publish feedback
        publishFeedback();

        if (reachEndOfPath())
        {
          mrp_common::Log::basicInfo(
              get_node_logging_interface(),
              "Reach goal");
          publishZeroVelocity();
          break;
        }

        if (!loop_rate.sleep())
        {
          mrp_common::Log::basicWarn(
              get_node_logging_interface(),
              "Motion planner exceeded the specified rate");
        }
      }
    }
    catch (const std::exception &e)
    {
      // std::cout << e.what() << std;
    }

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Controller succeeded. Sending back result");

    std::shared_ptr<nav2_msgs::action::FollowPath::Result> result =
        std::make_shared<nav2_msgs::action::FollowPath::Result>();
    result->result = std_msgs::msg::Empty();
    follow_path_action_server_->succeededCurrent(result);
  }

  void MotionPlannerServer::updatePath()
  {
  }

  void MotionPlannerServer::computeAndPublishVelocity()
  {
    nav_msgs::msg::Odometry robot_current_odom;
    {
      std::unique_lock<std::recursive_mutex> lck(robot_odom_->mtx);
      robot_current_odom = robot_odom_->current_odom;
    }
    // Get all odom from other members
    std::vector<nav_msgs::msg::Odometry> member_odom;
    for (const std::string &robot_name : member_robots_names_)
    {
      std::unique_lock<std::recursive_mutex> lck(member_robots_odom_data_map_[robot_name]->mtx);
      member_odom.push_back((member_robots_odom_data_map_[robot_name]->current_odom));
    }

    // Get current scan
    sensor_msgs::msg::LaserScan current_scan;
    {
      std::unique_lock<std::recursive_mutex> lck(robot_scan_->mtx);
      current_scan = robot_scan_->current_scan;
    }

    geometry_msgs::msg::Twist control_velocity;
    planner_ptr_->calculateVelocityCommand(
        robot_current_odom,
        member_odom,
        current_scan,
        control_velocity);

    // Publish control command
    robot_cmd_vel_pub_->publish(control_velocity);
  }

  bool MotionPlannerServer::reachEndOfPath()
  {
    return planner_ptr_->reachGoal();
  }

  void MotionPlannerServer::publishVelocity(const geometry_msgs::msg::Twist &control_velocity)
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(control_velocity);
    if (robot_cmd_vel_pub_ != nullptr && robot_cmd_vel_pub_->is_activated() &&
        robot_cmd_vel_pub_->get_subscription_count() > 0)
    {
      robot_cmd_vel_pub_->publish(std::move(cmd_vel));
    }
  }

  void MotionPlannerServer::publishZeroVelocity()
  {
    geometry_msgs::msg::Twist control_velocity;
    control_velocity.linear.x = 0;
    control_velocity.linear.y = 0;
    control_velocity.linear.z = 0;

    control_velocity.angular.x = 0;
    control_velocity.angular.y = 0;
    control_velocity.angular.z = 0;
    robot_cmd_vel_pub_->publish(control_velocity);
  }

  void MotionPlannerServer::createCmdVelPublisher()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Registering publisher for topic " + robot_cmd_vel_topic_name_);
    robot_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(robot_cmd_vel_topic_name_, 10);
  }

  void MotionPlannerServer::createOdomSubscriber()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Subscribing to robot odom for topic " + robot_odom_topic_name_);
    robot_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        robot_odom_topic_name_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          std::unique_lock<std::recursive_mutex> lck(robot_odom_->mtx);
          robot_odom_->current_odom = *msg;
          robot_odom_->ready = true;
        });
  }

  void MotionPlannerServer::createLaserScanSubscriber()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Subscribing to laser scan for topic " + robot_scan_topic_name_);
    robot_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        robot_scan_topic_name_, 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
          std::unique_lock<std::recursive_mutex> lck(robot_scan_->mtx);
          robot_scan_->current_scan = *msg;
          robot_scan_->ready = true;
        });
  }

  void MotionPlannerServer::createGetTeamClient()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating service client for getting all available teams");
    try
    {
      get_team_client_ = std::make_shared<mrp_common::ServiceClient<mrp_comms_msgs::srv::GetAllTeams>>(
          shared_from_this(),
          false, // Do not spin as an isolated thread
          robot_name_ + "/get_all_teams",
          rcl_service_get_default_options());
    }
    catch (const std::exception &e)
    {
      mrp_common::Log::basicError(
          get_node_logging_interface(),
          "Unable to create get team service client for " + robot_name_ + " due to err " + std::string(e.what()));
    }
  }

  void MotionPlannerServer::createGetMemberInTeamClient()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating service client for getting all members in team");
    try
    {
      get_all_robots_in_team_client_ = std::make_shared<mrp_common::ServiceClient<mrp_comms_msgs::srv::GetMembersInTeam>>(
          shared_from_this(),
          false, // Do not spin as an isolated thread
          robot_name_ + "/get_members_from_team",
          rcl_service_get_default_options());
    }
    catch (const std::exception &e)
    {
      mrp_common::Log::basicError(
          get_node_logging_interface(),
          "Unable to create get team service client for " + robot_name_ + " due to err " + std::string(e.what()));
    }
  }

  void MotionPlannerServer::createFollowPathActionServer()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating motion planner action server for " + robot_name_);
    try
    {
      follow_path_action_server_ = std::make_shared<mrp_common::ActionServer<nav2_msgs::action::FollowPath>>(
          shared_from_this(),
          "follow_path",
          std::bind(&MotionPlannerServer::followPath, this),
          nullptr,
          std::chrono::milliseconds(100), // Execution frequency
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

  void MotionPlannerServer::registerMemberRobots()
  {
    int team_id = 0; // This should later be requested
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Requesting list of robot member in team " + std::to_string(team_id));
    getAllMembersInTeam(team_id, member_robots_names_);

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Subscribing to odom of other robots beside this one");

    for (const std::string robot_name : member_robots_names_)
    {
      // We ignore this robot
      if (robot_name == robot_name_)
      {
        continue;
      }
      mrp_common::Log::basicInfo(
          get_node_logging_interface(),
          "Registering " + robot_name);
      member_robots_odom_data_map_[robot_name] = std::make_shared<RobotOdom>();

      mrp_common::Log::basicDebug(
          get_node_logging_interface(),
          "Subscribing to " + robot_name + " odom");
      member_robots_odom_sub_map_[robot_name] = create_subscription<nav_msgs::msg::Odometry>(
          "/" + robot_name + "/odom", 10,
          [this, robot_name](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            std::unique_lock<std::recursive_mutex> lck(member_robots_odom_data_map_[robot_name]->mtx);
            member_robots_odom_data_map_[robot_name]->current_odom = *msg;
            member_robots_odom_data_map_[robot_name]->ready = true;
          });
    }
  }

  void MotionPlannerServer::publishFeedback()
  {
    nav_msgs::msg::Odometry robot_current_odom;
    {
      std::unique_lock<std::recursive_mutex> lck(robot_odom_->mtx);
      robot_current_odom = robot_odom_->current_odom;
    }
    // Create feedback
    std::shared_ptr<nav2_msgs::action::FollowPath::Feedback> feedback =
        std::make_shared<nav2_msgs::action::FollowPath::Feedback>();
    feedback->distance_to_goal = planner_ptr_->getDistanceToGoal(robot_current_odom.pose.pose);

    // Publish feedback
    follow_path_action_server_->publishFeedback(feedback);
  }
}