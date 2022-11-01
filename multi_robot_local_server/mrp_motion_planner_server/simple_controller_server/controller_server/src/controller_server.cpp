#include "controller_server/controller_server.hpp"

namespace mrp_motion_planner_server
{
  using namespace std::chrono_literals;
  namespace controller_server
  {
    ControllerServer::ControllerServer()
        : mrp_common::LifecycleNode::LifecycleNode(
              "controller_server",
              "robot", true, true, std::chrono::milliseconds(1000))
    {
      controller_loader_ptr_ = std::make_shared<pluginlib::ClassLoader<local_server_core::LocalController>>("mrp_local_server_core", "local_server_core::LocalController");
      controller_name_ = "spotturn_controller";
    }

    ControllerServer::~ControllerServer()
    {
    }

    bool ControllerServer::atFinalTarget()
    {
    }

    void ControllerServer::initialise()
    {
      declare_parameter<std::vector<std::string>>("controller_name", std::vector<std::string>());
      declare_parameter<std::vector<std::string>>("controller_mapping", std::vector<std::string>());

      std::vector<std::string> controller_names = get_parameter("controller_name").as_string_array();
      std::vector<std::string> controller_mapping = get_parameter("controller_mapping").as_string_array();
      robot_name_ = get_namespace();
      cmd_vel_topic_ = robot_name_ + "/cmd_vel";
      odom_topic_ = robot_name_ + "/odom";

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
      std::string local_string = odom_topic_;
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_, 10,
          [this, local_string](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            std::cout << "Message coming from " + local_string << std::endl;
            std::unique_lock<std::recursive_mutex> lck(odom_mtx_);
            current_odom_ = *msg;
            odom_ready_ = true;
          });

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

      nav_msgs::msg::Odometry current_odom;
      {
        std::unique_lock<std::recursive_mutex> lck(odom_mtx_);
        current_odom = current_odom_;
      }
      geometry_msgs::msg::Twist control_velocity;
      controller_ptr_->calculateVelocityCommand(current_odom.pose.pose, control_velocity);
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
    }

    bool ControllerServer::loadController(const std::string &controller_name)
    {
      controller_name_ = controller_name;
      try
      {
        controller_ptr_ = controller_loader_ptr_->createSharedInstance(controller_name_map_[controller_name_]);
        controller_ptr_->initialise();
      }
      catch (pluginlib::PluginlibException &ex)
      {
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