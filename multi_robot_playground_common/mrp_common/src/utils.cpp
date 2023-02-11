#include "mrp_common/utils.hpp"

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

  std::string ROSUtils::sanitiseNodeName(const std::string &input_node_name)
  {
    std::string node_name(input_node_name);
    // read this as `replace` characters in `node_name` `if` not alphanumeric.
    // replace with '_'
    replace_if(
        begin(node_name), end(node_name),
        [](auto c)
        { return !isalnum(c); },
        '_');
    return node_name;
  }

  std::string ROSUtils::generateInternalNodeName(const std::string &prefix)
  {
    return ROSUtils::sanitiseNodeName(prefix) + "_" + ROSUtils::timeToString(8);
  }

  rclcpp::Node::SharedPtr ROSUtils::generateInternalNode(const std::string &prefix)
  {
    auto options =
        rclcpp::NodeOptions()
            .start_parameter_services(false)
            .start_parameter_event_publisher(false)
            .arguments({"--ros-args", "-r", "__node:=" + ROSUtils::generateInternalNodeName(prefix), "--"});
    return rclcpp::Node::make_shared("_", options);
  }

  std::string ROSUtils::timeToString(size_t len)
  {
    std::string output(len, '0'); // prefill the string with zeros
    auto timepoint = std::chrono::high_resolution_clock::now();
    auto timecount = timepoint.time_since_epoch().count();
    auto timestring = std::to_string(timecount);
    if (timestring.length() >= len)
    {
      // if `timestring` is shorter, put it at the end of `output`
      output.replace(
          0, len,
          timestring,
          timestring.length() - len, len);
    }
    else
    {
      // if `output` is shorter, just copy in the end of `timestring`
      output.replace(
          len - timestring.length(), timestring.length(),
          timestring,
          0, timestring.length());
    }
    return output;
  }
}