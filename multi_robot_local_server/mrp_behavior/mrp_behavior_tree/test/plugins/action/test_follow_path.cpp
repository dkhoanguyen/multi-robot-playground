#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "mrp_behavior_tree/plugins/action/follow_path_action.hpp"
#include "mrp_msgs/action/follow_path.hpp"

class FollowPathActionServer : public TestActionServer<mrp_msgs::action::FollowPath>
{
public:
  FollowPathActionServer()
      : TestActionServer("follow_path")
  {
  }

protected:
  void execute(
      const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<mrp_msgs::action::FollowPath>>)
      override
  {
  }
};

class FollowPathActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("follow_path_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
        "node",
        node_);
    config_->blackboard->set<std::chrono::milliseconds>(
        "server_timeout",
        std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("path_updated", false);
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<int>("number_recoveries", 0);

    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
      return std::make_unique<mrp_behavior_tree::FollowPathAction>(
          name, "follow_path", config);
    };

    factory_->registerBuilder<mrp_behavior_tree::FollowPathAction>("FollowPath", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    config_->blackboard->set("number_recoveries", 0);
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<FollowPathActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration *config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr FollowPathActionTestFixture::node_ = nullptr;
std::shared_ptr<FollowPathActionServer> FollowPathActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration *FollowPathActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> FollowPathActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> FollowPathActionTestFixture::tree_ = nullptr;

TEST_F(FollowPathActionTestFixture, test_ports)
{
  std::string xml_txt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <FollowPath />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<double>("speed"), 0.15);
  // EXPECT_EQ(tree_->rootNode()->getInput<std::string>("motion_planner"), "mrp_orca");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  FollowPathActionTestFixture::action_server_ = std::make_shared<FollowPathActionServer>();
  std::thread server_thread([]()
                            { rclcpp::spin(FollowPathActionTestFixture::action_server_); });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
