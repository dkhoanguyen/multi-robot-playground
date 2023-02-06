#ifndef MRP_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define MRP_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace mrp_behavior_tree
{
  enum class BtStatus
  {
    SUCCEEDED,
    FAILED,
    CANCELED
  };

  class BehaviorTreeEngine
  {
    BehaviorTreeEngine(const std::vector<std::string> &plugin_libraries);
    virtual ~BehaviorTreeEngine();

    BtStatus run(
        BT::Tree *tree,
        std::function<void()> onLoop,
        std::function<bool()> cancelRequested,
        std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

    BT::Tree createTreeFromText(
        const std::string &xml_string,
        BT::Blackboard::Ptr blackboard);

    BT::Tree createTreeFromFile(
        const std::string &file_path,
        BT::Blackboard::Ptr blackboard);

    void haltAllActions(BT::TreeNode *root_node);

  protected:
    BT::BehaviorTreeFactory factory_;
  };
}

#endif