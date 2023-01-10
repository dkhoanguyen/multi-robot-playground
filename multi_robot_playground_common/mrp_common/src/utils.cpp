#include "mrp_common/utils.hpp"
#include <iostream>

namespace mrp_common
{
  geometry_msgs::msg::Pose TransformUtils::toLocalFrame(
      const geometry_msgs::msg::Pose base_frame,
      const geometry_msgs::msg::Pose child_frame)
  {
    tf2::Transform t_base, t_child, t_out;
    geometry_msgs::msg::Pose out_pose;
    tf2::fromMsg(base_frame, t_base);
    tf2::fromMsg(child_frame, t_child);
    t_out = t_base.inverseTimes(t_child);
    tf2::toMsg(t_out, out_pose);
    return out_pose;
  }

  geometry_msgs::msg::Pose TransformUtils::toGlobalFrame(
      const geometry_msgs::msg::Pose base_frame,
      const geometry_msgs::msg::Pose child_frame)
  {
    tf2::Transform t_base, t_child, t_out;
    geometry_msgs::msg::Pose out_pose;
    tf2::fromMsg(base_frame, t_base);
    tf2::fromMsg(child_frame, t_child);
    t_out = t_base * t_child;
    tf2::toMsg(t_out, out_pose);
    return out_pose;
  }

  double GeometryUtils::euclideanDistance(const geometry_msgs::msg::Pose &first,
                                          const geometry_msgs::msg::Pose &second)
  {
    double x1 = first.position.x;
    double x2 = second.position.x;

    double y1 = first.position.y;
    double y2 = second.position.y;

    double z1 = first.position.z;
    double z2 = second.position.z;

    std::cout << "x1: " << x1 << " y1: " << y1 << std::endl;
    std::cout << "x2: " << x2 << " y2: " << y2 << std::endl;
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
  }

  double GeometryUtils::yawFromPose(const geometry_msgs::msg::Pose &pose)
  {
    tf2::Quaternion quad(pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w);
    tf2::Matrix3x3 m(quad);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  Eigen::Vector2d GeometryUtils::projectToXY(const double &length, const double &theta)
  {
    return Eigen::Vector2d{
        length * cos(theta),
        length * sin(theta)};
  }

  bool GeometryUtils::vectorIsInside(const Eigen::Vector2d &target_vector,
                                     const Eigen::Vector2d &lower_bound,
                                     const Eigen::Vector2d &upper_bound)
  {
    Eigen::Vector3d middle(
        target_vector(0),
        target_vector(1),
        0);

    Eigen::Vector3d lower(
        lower_bound(0),
        lower_bound(1),
        0);

    Eigen::Vector3d upper(
        upper_bound(0),
        upper_bound(1),
        0);

    Eigen::Vector3d c_lm = lower.cross(middle);
    Eigen::Vector3d c_lu = lower.cross(upper);

    Eigen::Vector3d c_um = upper.cross(middle);
    Eigen::Vector3d c_ul = upper.cross(lower);

    return (c_lm.dot(c_lu) > 0 && c_um.dot(c_ul) > 0);
  }
}