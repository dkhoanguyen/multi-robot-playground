#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "test_msgs/action/fibonacci.hpp"
#include "std_msgs/msg/empty.hpp"

#include "mrp_common/action_client.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class SimpleActionClientNode : public rclcpp::Node
{
public:
  SimpleActionClientNode() : rclcpp::Node("simple_action_client")
  {
  }

  ~SimpleActionClientNode() {}

  void init()
  {
    action_client_ = std::make_shared<mrp_common::ActionClient<test_msgs::action::Fibonacci>>(
        shared_from_this(),
        "fibonacci",
        std::bind(&SimpleActionClientNode::feedbackCallback, this, _1),
        std::bind(&SimpleActionClientNode::resultCallback, this, _1),
        false);

    // action_client_->waitForServer();
    std::cout << "Client init done" << std::endl;
  }

  bool resultReady()
  {
    return result_ready_;
  }
  std::shared_ptr<mrp_common::ActionClient<test_msgs::action::Fibonacci>> action_client_;

protected:
  bool result_ready_{false};

  void feedbackCallback(std::shared_ptr<const test_msgs::action::Fibonacci::Feedback> feedback)
  {
  }

  void resultCallback(rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::WrappedResult result)
  {
    result_ready_ = true;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      std::cout << "SUCCEEDED" << std::endl;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      std::cout << "ABORTED" << std::endl;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      std::cout << "CANCELED" << std::endl;
      break;
    default:
      break;
    }
  }
};

class SimpleActionServerNode : public rclcpp::Node
{
public:
  SimpleActionServerNode() : rclcpp::Node("simple_action_server")
  {
  }

  ~SimpleActionServerNode() {}

  void init()
  {
    action_server_ = rclcpp_action::create_server<test_msgs::action::Fibonacci>(
        shared_from_this(),
        "fibonacci",
        std::bind(&SimpleActionServerNode::handleGoal, this, _1, _2),
        std::bind(&SimpleActionServerNode::handleCancel, this, _1),
        std::bind(&SimpleActionServerNode::handleAccepted, this, _1));
  }

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const test_msgs::action::Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SimpleActionServerNode::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<test_msgs::action::Fibonacci::Feedback>();
    auto &sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<test_msgs::action::Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  rclcpp_action::Server<test_msgs::action::Fibonacci>::SharedPtr action_server_;
};

class RclCppFixture
{
public:
  RclCppFixture()
  {
  }

  void Setup()
  {
    std::cout << "Running RclCppFixture" << std::endl;
    server_thread_ =
        std::make_shared<std::thread>(std::bind(&RclCppFixture::serverThreadFunc, this));
  }

  ~RclCppFixture()
  {
    server_thread_->join();
  }

  void serverThreadFunc()
  {
    auto node = std::make_shared<SimpleActionServerNode>();
    node->init();
    rclcpp::spin(node->get_node_base_interface());
    node.reset();
  }

  std::shared_ptr<std::thread> server_thread_;
};

class ActionClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<SimpleActionClientNode>();
    node_->init();
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<SimpleActionClientNode> node_;
};

TEST_F(ActionClientTest, test_action_request)
{
  auto goal = test_msgs::action::Fibonacci::Goal();
  goal.order = 12;

  EXPECT_TRUE(node_->action_client_->sendGoal(goal));
  EXPECT_TRUE(node_->action_client_->waitForResult());

  // Obtain final result
  auto result = node_->action_client_->getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

  // Sum all of the values in the requested fibonacci series
  int sum = 0;
  for (auto number : result.result->sequence)
  {
    sum += number;
  }

  EXPECT_EQ(sum, 376);
  SUCCEED();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RclCppFixture g_rclcppfixture;
  g_rclcppfixture.Setup();
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  rclcpp::Rate(1).sleep();
  return result;
}