#include "mrp_comms_server/comms_server.hpp"

namespace mrp_comms
{
  CommsServer::CommsServer()
      : mrp_common::LifecycleNode(
            "robot_comms_server",
            "robot", true, true, std::chrono::milliseconds(1000))
  {
  }

  CommsServer::~CommsServer()
  {
  }

  bool CommsServer::initialise()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Configuring robot_comms_server...");
    mrp_common::Log::basicInfo(
        get_node_logging_interface(), "Extracting parameters from parameter server");

    robot_name_ = get_namespace();

    // Create get_members_from_team
    createGetMembersInTeamServer();

    // Create get_all_team
    createGetAllTeamsServer();

    // Initialise heartbeat
    initialiseHeartbeat();

    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Robot comms server initialises successfully");

    return true;
  }
  bool CommsServer::start()
  {
    // Start beating
    startHeartbeat();
    return true;
  }
  bool CommsServer::stop()
  {
    // Stop heartbeat
    stopHeartbeat();
    return true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  CommsServer::on_configure(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Initialising robot comms server");
    if (!initialise())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  CommsServer::on_activate(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Activating robot comms server");
    if (!start())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  CommsServer::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Deactivating robot comms server");
    if (!stop())
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  CommsServer::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Cleaning up robot comms server");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  CommsServer::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Shutting down robot comms server");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void CommsServer::createGetMembersInTeamServer()
  {
    mrp_common::Log::basicInfo(
        get_node_logging_interface(),
        "Creating service server for providing all members in a given team");

    try
    {
      get_members_server_ =
          std::make_shared<mrp_common::ServiceServer<mrp_comms_msgs::srv::GetMembersInTeam>>(
              shared_from_this(),
              robot_name_ + "/get_members_from_team",
              std::bind(&CommsServer::handleGetMembersInTeamServerRequest, this,
                        std::placeholders::_1, std::placeholders::_2),
              false,
              rcl_service_get_default_options());
    }
    catch (const std::exception &e)
    {
    }
  }

  void CommsServer::createGetAllTeamsServer()
  {
  }

  std::vector<std::string> CommsServer::getMembersInTeam(const int &team_id)
  {
    std::vector<std::string> members_list;
    members_list.push_back("robot");
    // members_list.push_back("robot0");
    // members_list.push_back("robot1");
    // members_list.push_back("robot2");
    return members_list;
  }

  void CommsServer::handleGetMembersInTeamServerRequest(
      mrp_comms_msgs::srv::GetMembersInTeam::Request::SharedPtr request,
      mrp_comms_msgs::srv::GetMembersInTeam::Response::SharedPtr response)
  {
    response->member_name_list = getMembersInTeam(request->team_id);
  }
}