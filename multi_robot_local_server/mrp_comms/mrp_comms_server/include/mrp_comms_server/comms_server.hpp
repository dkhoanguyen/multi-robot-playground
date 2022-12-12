#ifndef MRP_COMMS_SERVER__COMMS_SERVER_HPP_
#define MRP_COMMS_SERVER__COMMS_SERVER_HPP_

#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common/service_server.hpp"

#include "mrp_comms_msgs/srv/get_all_teams.hpp"
#include "mrp_comms_msgs/srv/get_members_in_team.hpp"

namespace mrp_comms
{
  class CommsServer : public mrp_common::LifecycleNode
  {
  public:
    CommsServer();
    virtual ~CommsServer();

    bool initialise();
    bool start();
    bool stop();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state) override;

  protected:
    std::string robot_name_;

    // Team related`
    std::unordered_map<int, std::vector<std::string>> team_id_map_;

    std::shared_ptr<mrp_common::ServiceServer<mrp_comms_msgs::srv::GetMembersInTeam>> get_members_server_;
    void createGetMembersInTeamServer();

    // std::shared_ptr<mrp_common::ServiceServer<mrp_comms_msgs::srv::GetAllTeams>> get_members_server_;
    void createGetAllTeamsServer();

    std::vector<std::string> getMembersInTeam(const int &team_id);

    void handleGetMembersInTeamServerRequest(
        mrp_comms_msgs::srv::GetMembersInTeam::Request::SharedPtr request,
        mrp_comms_msgs::srv::GetMembersInTeam::Response::SharedPtr response);
  };
}

#endif