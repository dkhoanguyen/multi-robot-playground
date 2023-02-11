#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "test_msgs/action/fibonacci.hpp"
#include "std_msgs/msg/empty.hpp"

#include "mrp_common/action_server.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SimpleActionServerNode : public rclcpp::Node
{
public:
  SimpleActionServerNode() : rclcpp::Node("simple_action_server")
  {
  }

  ~SimpleActionServerNode() {}

  void init()
  {
    action_server_ = std::make_shared<mrp_common::ActionServer<test_msgs::action::Fibonacci>>(
        shared_from_this(),
        "fibonacci",
        std::bind(&SimpleActionServerNode::execute, this),
        nullptr,
        std::chrono::milliseconds(100),
        false,
        rcl_action_server_get_default_options());
    action_server_->activate();

    deactivate_subs_ = create_subscription<std_msgs::msg::Empty>(
        "deactivate_server",
        1,
        [this](std_msgs::msg::Empty::UniquePtr /*msg*/)
        {
          RCLCPP_INFO(this->get_logger(), "Deactivating");
          action_server_->deactivate();
        });

    activate_subs_ = create_subscription<std_msgs::msg::Empty>(
        "activate_server",
        1,
        [this](std_msgs::msg::Empty::UniquePtr /*msg*/)
        {
          RCLCPP_INFO(this->get_logger(), "Activating");
          action_server_->activate();
        });

    omit_preempt_subs_ = create_subscription<std_msgs::msg::Empty>(
        "omit_preemption",
        1,
        [this](std_msgs::msg::Empty::UniquePtr /*msg*/)
        {
          RCLCPP_INFO(this->get_logger(), "Ignoring preemptions");
          do_premptions_ = false;
        });
  }

  void term()
  {
    // // when nothing's running make sure everything's dead.
    // const std::shared_ptr<const Fibonacci::Goal> a = action_server_->acceptPendingGoal();
    // const std::shared_ptr<const Fibonacci::Goal> b = action_server_->getCurrentGoal();
    // assert(a == b);
    // assert(action_server_->isCancelRequested() == false);
    // auto feedback = std::make_shared<Fibonacci::Feedback>();
    // action_server_->publishFeedback(feedback);
    // action_server_.reset();
  }

  void execute()
  {
    rclcpp::Rate loop_rate(100);

  preempted:
    // Initialize the goal, feedback, and result
    auto goal = action_server_->getCurrentGoal();
    auto feedback = std::make_shared<test_msgs::action::Fibonacci::Feedback>();
    auto result = std::make_shared<test_msgs::action::Fibonacci::Result>();

    // Fibonacci-specific initialization
    auto &sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
      // Should be check periodically if this action has been canceled
      // or if the server has been deactivated.
      if (action_server_->isCancelRequested() || !action_server_->isServerActive())
      {
        result->sequence = sequence;
        return;
      }

      // Check if we've gotten an new goal, pre-empting the current one
      if (do_premptions_ && action_server_->isPreemptRequested())
      {
        action_server_->acceptPendingGoal();
        goto preempted;
      }

      // Update the sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);

      // Publish feedback
      action_server_->publishFeedback(feedback);
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->sequence = sequence;
      action_server_->succeededCurrent(result);
    }
  }

protected:
  std::shared_ptr<mrp_common::ActionServer<test_msgs::action::Fibonacci>> action_server_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr deactivate_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr activate_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr omit_preempt_subs_;

  bool do_premptions_{true};
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
    node->term();
    node.reset();
  }

  std::shared_ptr<std::thread> server_thread_;
};

class SimpleActionTestNode : public rclcpp::Node
{
public:
  SimpleActionTestNode()
      : rclcpp::Node("action_test_node")
  {
  }

  ~SimpleActionTestNode()
  {
  }

  void init()
  {
    action_client_ = rclcpp_action::create_client<test_msgs::action::Fibonacci>(shared_from_this(), "fibonacci");
    action_client_->wait_for_action_server();

    deactivate_pub_ = this->create_publisher<std_msgs::msg::Empty>("deactivate_server", 1);
    activate_pub_ = this->create_publisher<std_msgs::msg::Empty>("activate_server", 1);
    omit_prempt_pub_ = this->create_publisher<std_msgs::msg::Empty>("omit_preemption", 1);
  }

  void term()
  {
    action_client_.reset();
  }

  void deactivateServer()
  {
    deactivate_pub_->publish(std_msgs::msg::Empty());
  }

  void activateServer()
  {
    activate_pub_->publish(std_msgs::msg::Empty());
  }

  void omitServerPreemptions()
  {
    omit_prempt_pub_->publish(std_msgs::msg::Empty());
  }

  rclcpp_action::Client<test_msgs::action::Fibonacci>::SharedPtr action_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr deactivate_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr activate_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr omit_prempt_pub_;
};

class ActionServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<SimpleActionTestNode>();
    node_->init();
  }

  void TearDown() override
  {
    std::cout << " Teardown" << std::endl;
    node_->term();
    std::cout << " Teardown..." << std::endl;
    node_.reset();
    std::cout << " Teardown complete" << std::endl;
  }

  std::shared_ptr<SimpleActionTestNode> node_;
};

TEST_F(ActionServerTest, test_handle_action_request)
{
  // The goal for this invocation
  auto goal = test_msgs::action::Fibonacci::Goal();
  goal.order = 12;

  // Send the goal
  auto future_goal_handle = node_->action_client_->async_send_goal(goal);
  EXPECT_EQ(
      rclcpp::spin_until_future_complete(
          node_,
          future_goal_handle),
      rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = future_goal_handle.get();

  // Wait for the result
  auto future_result = node_->action_client_->async_get_result(goal_handle);
  EXPECT_EQ(
      rclcpp::spin_until_future_complete(node_, future_result),
      rclcpp::FutureReturnCode::SUCCESS);

  // The final result
  rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::WrappedResult result = future_result.get();
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

TEST_F(ActionServerTest, test_simple_action_with_feedback)
{
  int feedback_sum = 0;

  // A callback to accumulate the intermediate values
  auto feedback_callback = [&feedback_sum](
                               rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::SharedPtr /*goal_handle*/,
                               const std::shared_ptr<const test_msgs::action::Fibonacci::Feedback> feedback)
  {
    feedback_sum += feedback->sequence.back();
  };

  // The goal for this invocation
  auto goal = test_msgs::action::Fibonacci::Goal();
  goal.order = 10;

  auto send_goal_options = rclcpp_action::Client<test_msgs::action::Fibonacci>::SendGoalOptions();
  send_goal_options.feedback_callback = feedback_callback;

  // Send the goal
  auto future_goal_handle = node_->action_client_->async_send_goal(goal, send_goal_options);
  EXPECT_EQ(
      rclcpp::spin_until_future_complete(
          node_,
          future_goal_handle),
      rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = future_goal_handle.get();

  // Wait for the result
  auto future_result = node_->action_client_->async_get_result(goal_handle);
  EXPECT_EQ(
      rclcpp::spin_until_future_complete(
          node_,
          future_result),
      rclcpp::FutureReturnCode::SUCCESS);

  // The final result
  rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::WrappedResult result = future_result.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);

  // Sum all of the values in the requested fibonacci series
  int sum = 0;
  for (auto number : result.result->sequence)
  {
    sum += number;
  }

  EXPECT_EQ(sum, 143);
  EXPECT_GE(feedback_sum, 0); // We should have received *some* feedback
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