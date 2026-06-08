#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "yuyi_controller/path_types.hpp"
#include "yuyi_controller/path_utils.hpp"
#include "yaml-cpp/yaml.h"

namespace yuyi_controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kAnsiReset = "\033[0m";
constexpr const char * kAnsiCyan = "\033[1;36m";
constexpr const char * kAnsiGreen = "\033[1;32m";
constexpr const char * kAnsiYellow = "\033[1;33m";
constexpr const char * kAnsiRed = "\033[1;31m";

struct Pose2D
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

double yaw_from_quaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z));
  return std::atan2(siny_cosp, cosy_cosp);
}

double distance_xy(const Pose2D & pose, const PathPoint & point)
{
  return std::hypot(point.x_m - pose.x_m, point.y_m - pose.y_m);
}

std::filesystem::path workspace_root_from_share(const std::filesystem::path & share_dir)
{
  auto root = share_dir;
  for (int i = 0; i < 4; ++i) {
    if (!root.has_parent_path()) {
      return share_dir;
    }
    root = root.parent_path();
  }
  return root;
}

std::filesystem::path resolve_path_file(
  const std::filesystem::path & configured_path,
  const std::filesystem::path & share_dir)
{
  if (configured_path.empty()) {
    return {};
  }
  if (configured_path.is_absolute()) {
    return configured_path;
  }

  const auto cwd_candidate = std::filesystem::current_path() / configured_path;
  if (std::filesystem::exists(cwd_candidate)) {
    return cwd_candidate;
  }

  const auto workspace_root = workspace_root_from_share(share_dir);
  const auto workspace_candidate = workspace_root / configured_path;
  if (std::filesystem::exists(workspace_candidate)) {
    return workspace_candidate;
  }

  const auto share_candidate = share_dir / configured_path;
  if (std::filesystem::exists(share_candidate)) {
    return share_candidate;
  }

  return cwd_candidate;
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> parts;
  std::string current;
  for (const auto ch : line) {
    if (ch == ',') {
      parts.push_back(current);
      current.clear();
    } else {
      current.push_back(ch);
    }
  }
  parts.push_back(current);
  return parts;
}

std::vector<PathPoint> load_path_from_csv(const std::filesystem::path & path_file)
{
  std::ifstream stream(path_file);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open csv path file: " + path_file.string());
  }

  std::vector<PathPoint> points;
  std::string line;
  bool is_first_line = true;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    if (is_first_line) {
      is_first_line = false;
      continue;
    }

    const auto fields = split_csv_line(line);
    if (fields.size() < 6U) {
      continue;
    }

    PathPoint point;
    point.index = static_cast<std::size_t>(std::stoul(fields[0]));
    point.s_m = std::stod(fields[1]);
    point.x_m = std::stod(fields[2]);
    point.y_m = std::stod(fields[3]);
    point.yaw_rad = std::stod(fields[4]);
    point.curvature = std::stod(fields[5]);
    points.push_back(point);
  }

  return points;
}

std::vector<PathPoint> load_path_from_yaml(const std::filesystem::path & path_file)
{
  const auto yaml = YAML::LoadFile(path_file.string());
  const auto spline_points = yaml["spline_points"];
  if (!spline_points || !spline_points.IsSequence()) {
    throw std::runtime_error("yaml path file does not contain spline_points sequence");
  }

  std::vector<PathPoint> points;
  points.reserve(spline_points.size());

  for (const auto & item : spline_points) {
    PathPoint point;
    point.index = item["index"] ? item["index"].as<std::size_t>() : points.size();
    point.s_m = item["s_m"].as<double>();
    point.x_m = item["x_m"].as<double>();
    point.y_m = item["y_m"].as<double>();
    point.yaw_rad = item["yaw_rad"] ? item["yaw_rad"].as<double>() : 0.0;
    point.curvature = item["curvature"] ? item["curvature"].as<double>() : 0.0;
    points.push_back(point);
  }

  return points;
}

std::vector<PathPoint> load_path_points(const std::filesystem::path & path_file)
{
  const auto extension = path_file.extension().string();
  if (extension == ".yaml" || extension == ".yml") {
    return load_path_from_yaml(path_file);
  }
  if (extension == ".csv") {
    return load_path_from_csv(path_file);
  }
  throw std::runtime_error("unsupported path file extension: " + extension);
}

}  // namespace

class YuyiControllerNode : public rclcpp::Node
{
public:
  YuyiControllerNode()
  : Node("yuyi_controller_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    path_topic_ = declare_parameter<std::string>("path_topic", "/yuyi_controller/reference_path");
    target_marker_topic_ = declare_parameter<std::string>(
      "target_marker_topic", "/yuyi_controller/lookahead_target");
    controller_rate_hz_ = declare_parameter<double>("controller_rate_hz", 20.0);
    lookahead_time_sec_ = declare_parameter<double>("lookahead_time_sec", 1.5);
    min_lookahead_distance_m_ = declare_parameter<double>("min_lookahead_distance_m", 0.6);
    max_lookahead_distance_m_ = declare_parameter<double>("max_lookahead_distance_m", 2.0);
    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.2);
    max_speed_mps_ = declare_parameter<double>("max_speed_mps", 0.8);
    max_acceleration_mps2_ = declare_parameter<double>("max_acceleration_mps2", 0.5);
    max_deceleration_mps2_ = declare_parameter<double>("max_deceleration_mps2", 0.8);
    max_angular_speed_radps_ = declare_parameter<double>("max_angular_speed_radps", 1.5);
    loop_path_ = declare_parameter<bool>("loop_path", false);
    stop_at_goal_ = declare_parameter<bool>("stop_at_goal", true);

    const auto package_share_dir = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("yuyi_controller"));
    const auto configured_path_file =
      std::filesystem::path(declare_parameter<std::string>("path_file", ""));
    path_file_ = resolve_path_file(configured_path_file, package_share_dir);

    if (path_file_.empty()) {
      throw std::runtime_error("path_file parameter must not be empty");
    }
    if (controller_rate_hz_ <= 0.0) {
      throw std::runtime_error("controller_rate_hz must be greater than 0");
    }
    if (min_lookahead_distance_m_ <= 0.0 || max_lookahead_distance_m_ < min_lookahead_distance_m_) {
      throw std::runtime_error("lookahead distance parameters are invalid");
    }
    if (max_speed_mps_ < 0.0 || max_acceleration_mps2_ <= 0.0 || max_deceleration_mps2_ <= 0.0) {
      throw std::runtime_error("speed or acceleration parameters are invalid");
    }

    path_points_ = load_path_points(path_file_);
    if (path_points_.size() < 2U) {
      throw std::runtime_error("loaded path must contain at least two points");
    }

    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).transient_local());
    target_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      target_marker_topic_, 10);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(10),
      std::bind(&YuyiControllerNode::odom_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / controller_rate_hz_)),
      std::bind(&YuyiControllerNode::control_step, this));

    publish_reference_path();

    RCLCPP_INFO(
      get_logger(),
      "%sLoaded %zu path points from %s; odom='%s', cmd_vel='%s', loop_path=%s%s",
      kAnsiCyan,
      path_points_.size(),
      path_file_.string().c_str(),
      odom_topic_.c_str(),
      cmd_vel_topic_.c_str(),
      loop_path_ ? "true" : "false",
      kAnsiReset);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.x_m = msg->pose.pose.position.x;
    current_pose_.y_m = msg->pose.pose.position.y;
    current_pose_.yaw_rad = yaw_from_quaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    has_odom_ = true;
  }

  void control_step()
  {
    const auto now = get_clock()->now();
    const auto dt = last_control_time_.nanoseconds() == 0 ?
      (1.0 / controller_rate_hz_) :
      std::max(1e-3, (now - last_control_time_).seconds());
    last_control_time_ = now;

    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%swaiting for odom%s", kAnsiYellow, kAnsiReset);
      publish_stop();
      return;
    }

    nearest_index_ = find_nearest_index(current_pose_, nearest_index_);
    const auto remaining_distance_m = path_utils::remaining_distance_m(
      path_points_, nearest_index_, loop_path_);

    if (path_utils::should_stop_at_goal(
        remaining_distance_m, goal_tolerance_m_, stop_at_goal_, loop_path_))
    {
      commanded_speed_mps_ = move_towards(commanded_speed_mps_, 0.0, max_deceleration_mps2_ * dt);
      publish_cmd(commanded_speed_mps_, 0.0);
      publish_target_marker(path_points_.back(), true);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "%sgoal reached%s", kAnsiGreen, kAnsiReset);
      return;
    }

    const auto lookahead_distance_m = compute_lookahead_distance();
    const auto target_index = find_lookahead_index(nearest_index_, lookahead_distance_m);
    const auto & target_point = path_points_[target_index];

    const auto dx = target_point.x_m - current_pose_.x_m;
    const auto dy = target_point.y_m - current_pose_.y_m;
    const auto local_x = std::cos(current_pose_.yaw_rad) * dx + std::sin(current_pose_.yaw_rad) * dy;
    const auto local_y = -std::sin(current_pose_.yaw_rad) * dx + std::cos(current_pose_.yaw_rad) * dy;
    const auto effective_lookahead_m = std::max(lookahead_distance_m, std::hypot(local_x, local_y));

    const auto curvature = std::abs(effective_lookahead_m) <= 1e-6 ?
      0.0 :
      (2.0 * local_y) / (effective_lookahead_m * effective_lookahead_m);
    const auto target_speed_mps = compute_target_speed(remaining_distance_m);
    commanded_speed_mps_ = move_towards(
      commanded_speed_mps_,
      target_speed_mps,
      (target_speed_mps >= commanded_speed_mps_ ? max_acceleration_mps2_ : max_deceleration_mps2_) * dt);
    const auto angular_z = std::clamp(
      commanded_speed_mps_ * curvature,
      -max_angular_speed_radps_,
      max_angular_speed_radps_);

    publish_cmd(commanded_speed_mps_, angular_z);
    publish_target_marker(target_point, false);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "%snearest=%zu target=%zu rem=%.2f lookahead=%.2f cmd_v=%.2f cmd_w=%.2f loop=%s%s",
      kAnsiCyan,
      nearest_index_,
      target_index,
      remaining_distance_m,
      effective_lookahead_m,
      commanded_speed_mps_,
      angular_z,
      loop_path_ ? "true" : "false",
      kAnsiReset);
  }

  std::size_t find_nearest_index(const Pose2D & pose, std::size_t hint_index) const
  {
    (void)hint_index;
    std::size_t best_index = 0U;
    double best_distance = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < path_points_.size(); ++i) {
      const auto dist = distance_xy(pose, path_points_[i]);
      if (dist < best_distance) {
        best_distance = dist;
        best_index = i;
      }
    }

    return best_index;
  }

  std::size_t find_lookahead_index(std::size_t nearest_index, double lookahead_distance_m) const
  {
    return path_utils::find_lookahead_index(
      path_points_, nearest_index, lookahead_distance_m, loop_path_);
  }

  double compute_lookahead_distance() const
  {
    const auto dynamic_distance = std::abs(commanded_speed_mps_) * lookahead_time_sec_;
    return std::clamp(dynamic_distance, min_lookahead_distance_m_, max_lookahead_distance_m_);
  }

  double compute_target_speed(double remaining_distance_m) const
  {
    return path_utils::target_speed_mps(
      remaining_distance_m, max_speed_mps_, max_deceleration_mps2_, loop_path_);
  }

  double move_towards(double current, double target, double max_delta) const
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

  void publish_cmd(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_publisher_->publish(cmd);
  }

  void publish_stop()
  {
    commanded_speed_mps_ = 0.0;
    publish_cmd(0.0, 0.0);
  }

  void publish_reference_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    path.poses.reserve(path_points_.size());

    for (const auto & point : path_points_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x_m;
      pose.pose.position.y = point.y_m;
      pose.pose.orientation.z = std::sin(point.yaw_rad * 0.5);
      pose.pose.orientation.w = std::cos(point.yaw_rad * 0.5);
      path.poses.push_back(pose);
    }

    path_publisher_->publish(path);
  }

  void publish_target_marker(const PathPoint & target, bool goal_reached)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "yuyi_controller";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = target.x_m;
    marker.pose.position.y = target.y_m;
    marker.pose.position.z = 0.15;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0F;
    if (goal_reached) {
      marker.color.r = 0.2F;
      marker.color.g = 0.8F;
      marker.color.b = 0.2F;
    } else {
      marker.color.r = 0.95F;
      marker.color.g = 0.2F;
      marker.color.b = 0.1F;
    }
    target_marker_publisher_->publish(marker);
  }

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string path_topic_;
  std::string target_marker_topic_;
  std::filesystem::path path_file_;
  double controller_rate_hz_{20.0};
  double lookahead_time_sec_{1.5};
  double min_lookahead_distance_m_{0.6};
  double max_lookahead_distance_m_{2.0};
  double goal_tolerance_m_{0.2};
  double max_speed_mps_{0.8};
  double max_acceleration_mps2_{0.5};
  double max_deceleration_mps2_{0.8};
  double max_angular_speed_radps_{1.5};
  bool loop_path_{false};
  bool stop_at_goal_{true};

  std::vector<PathPoint> path_points_;
  Pose2D current_pose_;
  bool has_odom_{false};
  std::size_t nearest_index_{0U};
  double commanded_speed_mps_{0.0};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace yuyi_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<yuyi_controller::YuyiControllerNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("yuyi_controller_node"),
      "%sfatal error: %s%s",
      "\033[1;31m",
      ex.what(),
      "\033[0m");
  }
  rclcpp::shutdown();
  return 0;
}
