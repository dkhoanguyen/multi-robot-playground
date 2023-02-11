#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__UTILS_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <Eigen/Dense>

namespace mrp_common
{
  class TransformUtils
  {
  public:
    static geometry_msgs::msg::Pose toLocalFrame(
        const geometry_msgs::msg::Pose base_frame,
        const geometry_msgs::msg::Pose child_frame);

    static geometry_msgs::msg::Pose toGlobalFrame(
        const geometry_msgs::msg::Pose base_frame,
        const geometry_msgs::msg::Pose child_frame);
  };

  class GeometryUtils
  {
  public:
    static double euclideanDistance(const geometry_msgs::msg::Pose &first,
                                    const geometry_msgs::msg::Pose &second);
    static double yawFromPose(const geometry_msgs::msg::Pose &pose);

    static Eigen::Vector2d projectToXY(const double &length, const double &theta);

    static bool vectorIsInside(const Eigen::Vector2d &target_vector,
                               const Eigen::Vector2d &lower_bound,
                               const Eigen::Vector2d &upper_bound);
  };

  class ROSUtils
  {
  public:
    static std::string sanitiseNodeName(const std::string &input_node_name);
    static std::string generateInternalNodeName(const std::string &prefix = "");
    static rclcpp::Node::SharedPtr generateInternalNode(const std::string &prefix = "");
    static std::string timeToString(size_t len);
  };
}

#endif