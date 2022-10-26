#include "controller_server/controller_server.hpp"

namespace mrp_motion_planner_server
{
  using namespace std::chrono_literals;
  namespace controller_server
  {
    ControllerServer::ControllerServer()
        : mrp_common::LifecycleNode::LifecycleNode(
              "controller_server",
              "robot1", true, true, std::chrono::milliseconds(1000))
    {
      controller_loader_ptr_ = std::make_shared<pluginlib::ClassLoader<local_server_core::LocalController>>("mrp_local_server_core", "local_server_core::LocalController");
      controller_name_ = "spotturn_controller";
    }

    ControllerServer::~ControllerServer()
    {
    }

    void ControllerServer::initialise()
    {
      declare_parameter<std::vector<std::string>>("controller_name", std::vector<std::string>());
      declare_parameter<std::vector<std::string>>("controller_mapping", std::vector<std::string>());
      declare_parameter<std::string>("robot_name", std::string());

      std::vector<std::string> controller_names = get_parameter("controller_name").as_string_array();
      std::vector<std::string> controller_mapping = get_parameter("controller_mapping").as_string_array();
      robot_name_ = get_parameter("robot_name").as_string();
      cmd_vel_topic_ = "/" + robot_name_ + "/cmd_vel";
      odom_topic_ = "/" + robot_name_ + "/odom";

      for (unsigned int idx; idx < controller_names.size(); idx++)
      {
        controller_name_map_[controller_names.at(idx)] = controller_mapping.at(idx);
      }

      std::cout << "Creating service server" << std::endl;
      waypoints_server_ = std::make_shared<mrp_common::ServiceServer<mrp_motion_planner_msgs::srv::Waypoints>>(
          shared_from_this(),
          "waypoints",
          std::bind(&ControllerServer::waypointSrvCallback, this, std::placeholders::_1, std::placeholders::_2),
          false,
          rcl_service_get_default_options());
      std::cout << "Server created" << std::endl;
    }

    void ControllerServer::start()
    {
      // Initialise service for trajectory control
      // Create publisher
      std::cout << cmd_vel_topic_ << std::endl;
      cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
      std::cout << cmd_vel_pub_->is_activated() << std::endl;
      cmd_vel_pub_->on_activate();
      std::cout << cmd_vel_pub_->is_activated() << std::endl;

      // Create odom subscriber
      std::cout << odom_topic_ << std::endl;
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_, 10,
          std::bind(&ControllerServer::odomCallback, this, std::placeholders::_1));

      // Create timer - 10Hz controller
      cmd_vel_pub_timer_ = create_wall_timer(
          100ms, std::bind(&ControllerServer::followWaypoints, this));
    }

    void ControllerServer::followWaypoints()
    {
      if (!odom_ready_)
      {
        return;
      }
      if (waypoints_.size() == 0)
      {
        return;
      }
      std::cout << "Following waypoints: " << std::endl;
      nav_msgs::msg::Odometry current_odom;
      {
        std::unique_lock<std::recursive_mutex> lck(odom_mtx_);
        current_odom = current_odom_;
      }
      geometry_msgs::msg::Twist control_velocity;
      controller_ptr_->calculateVelocityCommand(current_odom.pose.pose, control_velocity);
      std::cout << "Linear: " << control_velocity.linear.x << std::endl;
      std::cout << "Angular: " << control_velocity.angular.z << std::endl;
      cmd_vel_pub_->publish(control_velocity);
    }

    void ControllerServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      std::unique_lock<std::recursive_mutex> lck(odom_mtx_);
      current_odom_ = *msg;
      odom_ready_ = true;
    }

    void ControllerServer::waypointSrvCallback(std::shared_ptr<mrp_motion_planner_msgs::srv::Waypoints::Request> &request,
                                               std::shared_ptr<mrp_motion_planner_msgs::srv::Waypoints::Response> &response)
    {
      std::unique_lock<std::recursive_mutex> lck(waypoint_mtx_);
      waypoints_ = request->pose_array.poses;
      controller_ptr_->setWaypoints(waypoints_);
      response->success = true;
      std::cout << "Waypoints received" << std::endl;
    }

    bool ControllerServer::loadController(const std::string &controller_name)
    {
      controller_name_ = controller_name;

      // pluginlib::ClassLoader<local_server_core::LocalController> controller_loader("mrp_local_server_core", "local_server_core::LocalController");
      try
      {
        controller_ptr_ = controller_loader_ptr_->createSharedInstance(controller_name_map_[controller_name_]);
        controller_ptr_->initialise();
      }
      catch (pluginlib::PluginlibException &ex)
      {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        return false;
      }
      return true;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerServer::on_configure(const rclcpp_lifecycle::State &state)
    {
      // initialise();
      // // Load controller based on name
      // loadController("spotturn_controller");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerServer::on_activate(const rclcpp_lifecycle::State &state)
    {
      start();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
  }
}