#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double move_towards(double current, double target, double max_delta)
{
  if (max_delta <= 0.0) {
    return target;
  }

  const auto delta = target - current;
  if (std::abs(delta) <= max_delta) {
    return target;
  }

  return current + std::copysign(max_delta, delta);
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw_rad)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw_rad);
  return tf2::toMsg(quaternion);
}

std_msgs::msg::ColorRGBA make_color(float red, float green, float blue, float alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

visualization_msgs::msg::Marker make_cube_marker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  int32_t id,
  const std::string & ns,
  double x_m,
  double y_m,
  double z_m,
  double yaw_rad,
  double scale_x_m,
  double scale_y_m,
  double scale_z_m,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x_m;
  marker.pose.position.y = y_m;
  marker.pose.position.z = z_m;
  marker.pose.orientation = yaw_to_quaternion(yaw_rad);
  marker.scale.x = scale_x_m;
  marker.scale.y = scale_y_m;
  marker.scale.z = scale_z_m;
  marker.color = color;
  return marker;
}

visualization_msgs::msg::Marker make_arrow_marker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  int32_t id,
  const std::string & ns,
  double x_m,
  double y_m,
  double z_m,
  double yaw_rad,
  double length_m,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x_m;
  marker.pose.position.y = y_m;
  marker.pose.position.z = z_m;
  marker.pose.orientation = yaw_to_quaternion(yaw_rad);
  marker.scale.x = length_m;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  marker.color = color;
  return marker;
}

}  // namespace

class AckermannSimulatorNode : public rclcpp::Node
{
public:
  AckermannSimulatorNode()
  : Node("ackermann_simulator_node"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    path_topic_ = declare_parameter<std::string>("path_topic", "/simulation/path");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/simulation/vehicle_markers");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    max_speed_mps_ = declare_parameter<double>("max_speed_mps", 2.0);
    max_acceleration_mps2_ = declare_parameter<double>("max_acceleration_mps2", 2.0);
    max_deceleration_mps2_ = declare_parameter<double>("max_deceleration_mps2", 3.0);
    max_steering_rate_radps_ = declare_parameter<double>("max_steering_rate_degps", 180.0) * kPi / 180.0;
    wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.9);
    track_width_m_ = declare_parameter<double>("track_width_m", 0.65);
    rear_axle_to_front_m_ = declare_parameter<double>("rear_axle_to_front_m", 0.65);
    rear_axle_to_back_m_ = declare_parameter<double>("rear_axle_to_back_m", 0.25);
    rear_axle_to_base_link_x_m_ = declare_parameter<double>("rear_axle_to_base_link_x_m", 0.0);
    vehicle_height_m_ = declare_parameter<double>("vehicle_height_m", 0.22);
    wheel_diameter_m_ = declare_parameter<double>("wheel_diameter_m", 0.22);
    wheel_thickness_m_ = declare_parameter<double>("wheel_thickness_m", 0.08);
    max_steering_angle_rad_ =
      declare_parameter<double>("max_steering_angle_deg", 30.0) * kPi / 180.0;
    publish_path_ = declare_parameter<bool>("publish_path", true);
    publish_visualization_ = declare_parameter<bool>("publish_visualization", true);
    max_path_points_ = static_cast<std::size_t>(declare_parameter<int>("max_path_points", 500));

    rear_axle_x_m_ = declare_parameter<double>("initial_x_m", 0.0);
    rear_axle_y_m_ = declare_parameter<double>("initial_y_m", 0.0);
    yaw_rad_ = declare_parameter<double>("initial_yaw_deg", 0.0) * kPi / 180.0;

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be greater than 0");
    }
    if (cmd_timeout_sec_ < 0.0) {
      throw std::runtime_error("cmd_timeout_sec must be greater than or equal to 0");
    }
    if (wheelbase_m_ <= 0.0) {
      throw std::runtime_error("wheelbase_m must be greater than 0");
    }
    if (track_width_m_ <= 0.0) {
      throw std::runtime_error("track_width_m must be greater than 0");
    }
    if (rear_axle_to_front_m_ <= 0.0 || rear_axle_to_back_m_ < 0.0) {
      throw std::runtime_error("rear axle geometry parameters are invalid");
    }
    if (vehicle_height_m_ <= 0.0 || wheel_diameter_m_ <= 0.0 || wheel_thickness_m_ <= 0.0) {
      throw std::runtime_error("vehicle visualization sizes must be greater than 0");
    }
    if (max_path_points_ == 0U) {
      max_path_points_ = 1U;
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    if (publish_path_) {
      path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
      path_msg_.header.frame_id = map_frame_id_;
    }
    if (publish_visualization_) {
      marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10,
      std::bind(&AckermannSimulatorNode::cmd_callback, this, std::placeholders::_1));

    last_update_time_ = now();
    last_cmd_time_ = last_update_time_;

    const auto period =
      std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AckermannSimulatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Ackermann simulator ready: cmd_vel='%s', odom='%s', tf=%s->%s",
      cmd_vel_topic_.c_str(),
      odom_topic_.c_str(),
      map_frame_id_.c_str(),
      base_frame_id_.c_str());
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();
    has_cmd_ = true;

    target_speed_mps_ = std::clamp(msg->linear.x, -max_speed_mps_, max_speed_mps_);

    if (std::abs(target_speed_mps_) < 1e-4 || std::abs(msg->angular.z) < 1e-6) {
      target_steering_angle_rad_ = 0.0;
      return;
    }

    const auto requested_steering =
      std::atan((wheelbase_m_ * msg->angular.z) / target_speed_mps_);
    target_steering_angle_rad_ =
      std::clamp(requested_steering, -max_steering_angle_rad_, max_steering_angle_rad_);
  }

  void on_timer()
  {
    const auto current_time = now();
    const auto dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    last_update_time_ = current_time;

    const auto cmd_age_sec = (current_time - last_cmd_time_).seconds();
    const bool command_is_fresh = has_cmd_ && cmd_age_sec <= cmd_timeout_sec_;
    const auto commanded_speed = command_is_fresh ? target_speed_mps_ : 0.0;
    const auto commanded_steering = command_is_fresh ? target_steering_angle_rad_ : 0.0;

    const auto accel_limit =
      std::abs(commanded_speed) >= std::abs(speed_mps_) ?
      max_acceleration_mps2_ :
      max_deceleration_mps2_;
    speed_mps_ = move_towards(speed_mps_, commanded_speed, accel_limit * dt);
    steering_angle_rad_ = move_towards(
      steering_angle_rad_,
      commanded_steering,
      max_steering_rate_radps_ * dt);

    const auto yaw_rate_radps = speed_mps_ * std::tan(steering_angle_rad_) / wheelbase_m_;
    rear_axle_x_m_ += speed_mps_ * std::cos(yaw_rad_) * dt;
    rear_axle_y_m_ += speed_mps_ * std::sin(yaw_rad_) * dt;
    yaw_rad_ = normalize_angle(yaw_rad_ + yaw_rate_radps * dt);

    publish_odometry(current_time, yaw_rate_radps);
    publish_transform(current_time);
    publish_path(current_time);
    publish_markers(current_time);
  }

  void publish_odometry(const rclcpp::Time & stamp, double yaw_rate_radps)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = base_link_x_m();
    odom.pose.pose.position.y = base_link_y_m();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yaw_to_quaternion(yaw_rad_);
    odom.twist.twist.linear.x = speed_mps_;
    odom.twist.twist.angular.z = yaw_rate_radps;
    odom_publisher_->publish(odom);
  }

  void publish_transform(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = base_link_x_m();
    transform.transform.translation.y = base_link_y_m();
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = yaw_to_quaternion(yaw_rad_);
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_path(const rclcpp::Time & stamp)
  {
    if (!publish_path_ || !path_publisher_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_id_;
    pose.pose.position.x = base_link_x_m();
    pose.pose.position.y = base_link_y_m();
    pose.pose.position.z = 0.0;
    pose.pose.orientation = yaw_to_quaternion(yaw_rad_);

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(std::move(pose));
    if (path_msg_.poses.size() > max_path_points_) {
      const auto erase_count = path_msg_.poses.size() - max_path_points_;
      path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + erase_count);
    }

    path_publisher_->publish(path_msg_);
  }

  void publish_markers(const rclcpp::Time & stamp)
  {
    if (!publish_visualization_ || !marker_publisher_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    const auto body_center_x_m =
      ((rear_axle_to_front_m_ - rear_axle_to_back_m_) * 0.5) - rear_axle_to_base_link_x_m_;
    const auto body_length_m = rear_axle_to_front_m_ + rear_axle_to_back_m_;
    const auto wheel_z_m = wheel_diameter_m_ * 0.5;
    const auto body_z_m = wheel_diameter_m_ + (vehicle_height_m_ * 0.5);
    const auto front_wheel_x_m = wheelbase_m_ - rear_axle_to_base_link_x_m_;
    const auto rear_wheel_x_m = -rear_axle_to_base_link_x_m_;
    const auto half_track_m = track_width_m_ * 0.5;
    markers.markers.push_back(make_cube_marker(
      base_frame_id_,
      stamp,
      0,
      "vehicle",
      body_center_x_m,
      0.0,
      body_z_m,
      0.0,
      body_length_m,
      track_width_m_,
      vehicle_height_m_,
      make_color(0.12F, 0.53F, 0.85F, 0.95F)));

    markers.markers.push_back(make_cube_marker(
      base_frame_id_,
      stamp,
      1,
      "vehicle",
      rear_wheel_x_m,
      half_track_m,
      wheel_z_m,
      0.0,
      wheel_diameter_m_,
      wheel_thickness_m_,
      wheel_diameter_m_,
      make_color(0.18F, 0.18F, 0.18F, 1.0F)));

    markers.markers.push_back(make_cube_marker(
      base_frame_id_,
      stamp,
      2,
      "vehicle",
      rear_wheel_x_m,
      -half_track_m,
      wheel_z_m,
      0.0,
      wheel_diameter_m_,
      wheel_thickness_m_,
      wheel_diameter_m_,
      make_color(0.18F, 0.18F, 0.18F, 1.0F)));

    markers.markers.push_back(make_cube_marker(
      base_frame_id_,
      stamp,
      3,
      "vehicle",
      front_wheel_x_m,
      0.0,
      wheel_z_m,
      steering_angle_rad_,
      wheel_diameter_m_,
      wheel_thickness_m_,
      wheel_diameter_m_,
      make_color(0.18F, 0.18F, 0.18F, 1.0F)));

    markers.markers.push_back(make_arrow_marker(
      base_frame_id_,
      stamp,
      4,
      "vehicle",
      body_center_x_m,
      0.0,
      body_z_m + (vehicle_height_m_ * 0.75),
      0.0,
      std::max(body_length_m * 0.7, 0.2),
      make_color(0.95F, 0.25F, 0.15F, 0.95F)));

    marker_publisher_->publish(markers);
  }

  double base_link_x_m() const
  {
    return rear_axle_x_m_ + (rear_axle_to_base_link_x_m_ * std::cos(yaw_rad_));
  }

  double base_link_y_m() const
  {
    return rear_axle_y_m_ + (rear_axle_to_base_link_x_m_ * std::sin(yaw_rad_));
  }

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string map_frame_id_;
  std::string base_frame_id_;

  double publish_rate_hz_{50.0};
  double cmd_timeout_sec_{0.5};
  double max_speed_mps_{2.0};
  double max_acceleration_mps2_{2.0};
  double max_deceleration_mps2_{3.0};
  double max_steering_rate_radps_{kPi};
  double wheelbase_m_{0.9};
  double track_width_m_{0.65};
  double rear_axle_to_front_m_{0.65};
  double rear_axle_to_back_m_{0.25};
  double rear_axle_to_base_link_x_m_{0.0};
  double vehicle_height_m_{0.22};
  double wheel_diameter_m_{0.22};
  double wheel_thickness_m_{0.08};
  double max_steering_angle_rad_{30.0 * kPi / 180.0};
  bool publish_path_{true};
  bool publish_visualization_{true};
  std::size_t max_path_points_{500U};

  double rear_axle_x_m_{0.0};
  double rear_axle_y_m_{0.0};
  double yaw_rad_{0.0};
  double speed_mps_{0.0};
  double steering_angle_rad_{0.0};
  double target_speed_mps_{0.0};
  double target_steering_angle_rad_{0.0};
  bool has_cmd_{false};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path path_msg_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
