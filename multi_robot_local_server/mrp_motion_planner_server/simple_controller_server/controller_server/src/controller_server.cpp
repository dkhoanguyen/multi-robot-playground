#include "controller_server/controller_server.hpp"

namespace mrp_motion_planner_server
{
  namespace controller_server
  {
    ControllerServer::ControllerServer()
        : mrp_common::LifecycleNode::LifecycleNode(
              "controller_server",
              "turtlebot3", true, true, std::chrono::milliseconds(1000))
    {
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

      for (unsigned int idx; idx < controller_names.size(); idx++)
      {
        controller_name_map_[controller_names.at(idx)] = controller_mapping.at(idx);
      }
    }

    void ControllerServer::start()
    {
      // Initialise service for trajectory control
    }

    bool ControllerServer::loadController(const std::string &controller_name)
    {
      controller_name_ = controller_name;

      pluginlib::ClassLoader<local_server_core::LocalController> controller_loader("mrp_local_server_core", "local_server_core::LocalController");
      try
      {
        controller_ptr_ = controller_loader.createSharedInstance(controller_name_map_[controller_name_]);
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
      // Load controller based on name
      if (!loadController(controller_name_))
      {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    ControllerServer::on_activate(const rclcpp_lifecycle::State &state)
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
  }
}