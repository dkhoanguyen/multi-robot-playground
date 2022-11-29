#include "mrp_common/utils.hpp"
#include <iostream>

namespace mrp_common
{
  tf2::Transform TransformUtils::toLocalFrame(const tf2::Transform base_frame,
                                              const tf2::Transform child_frame)
  {
    return base_frame.inverseTimes(child_frame);
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
      0
    );

    Eigen::Vector3d lower(
      lower_bound(0),
      lower_bound(1),
      0
    );

    Eigen::Vector3d upper(
      upper_bound(0),
      upper_bound(1),
      0
    );

    Eigen::Vector3d c_lm = lower.cross(middle);
    Eigen::Vector3d c_mu = middle.cross(upper);

    return c_lm.dot(c_mu) > 0;
  }
}