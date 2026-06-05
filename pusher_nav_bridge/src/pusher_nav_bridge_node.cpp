#include "pusher_nav_bridge/protocol.hpp"
#include "pusher_nav_bridge/route_processing.hpp"
#include "pusher_nav_bridge/side_distance_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace pusher_nav_bridge
{

namespace
{

FollowSide parse_follow_side(const std::string & value)
{
  return value == "right" ? FollowSide::Right : FollowSide::Left;
}

TravelDirection parse_travel_direction(const std::string & value)
{
  return value == "reverse" ? TravelDirection::Reverse : TravelDirection::Forward;
}

PosesReferenceType parse_reference_type(const std::string & value)
{
  if (value == "left_guardrail") {
    return PosesReferenceType::LeftGuardrail;
  }
  if (value == "right_guardrail") {
    return PosesReferenceType::RightGuardrail;
  }
  return PosesReferenceType::VehicleCenter;
}

bool valid_range(float range, float range_min, float range_max)
{
  return std::isfinite(range) && range >= range_min && range <= range_max;
}

std::vector<Point2D> scan_to_base_points(const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<Point2D> points;
  points.reserve(scan.ranges.size());

  if (scan.angle_increment == 0.0F) {
    return points;
  }

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!valid_range(range, scan.range_min, scan.range_max)) {
      continue;
    }
    const auto angle = static_cast<double>(scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(scan.angle_increment));
    const auto range_m = static_cast<double>(range);
    points.push_back(Point2D{range_m * std::cos(angle), range_m * std::sin(angle)});
  }

  return points;
}

uint16_t nonnegative_u16_param(rclcpp::Node & node, const std::string & name, int default_value)
{
  const auto value = static_cast<int>(node.declare_parameter<int>(name, default_value));
  return static_cast<uint16_t>(std::clamp(value, 0, 65535));
}

}  // namespace

class PusherNavBridgeNode : public rclcpp::Node
{
public:
  PusherNavBridgeNode()
  : Node("pusher_nav_bridge_node")
  {
    const auto poses_file = declare_parameter<std::string>("poses_file", "maps/poses.txt");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    follow_side_ = parse_follow_side(declare_parameter<std::string>("follow_side", "left"));

    RouteConfig route_config;
    route_config.target_left_distance_m = declare_parameter<double>("target_left_distance_m", 1.1);
    route_config.target_right_distance_m = declare_parameter<double>("target_right_distance_m", 1.1);
    route_config.min_point_gap_m = declare_parameter<double>("min_point_gap_m", 0.05);
    route_config.resample_step_m = declare_parameter<double>("resample_step_m", 0.5);
    route_config.heading_window_m = declare_parameter<double>("heading_window_m", 0.5);
    route_config.rdp_tolerance_m = declare_parameter<double>("rdp_tolerance_m", 0.15);
    route_config.min_segment_length_m = declare_parameter<double>("min_segment_length_m", 0.5);
    route_config.travel_direction =
      parse_travel_direction(declare_parameter<std::string>("travel_direction", "forward"));
    route_config.poses_reference_type =
      parse_reference_type(declare_parameter<std::string>("poses_reference_type", "vehicle_center"));

    side_config_.target_left_distance_m = route_config.target_left_distance_m;
    side_config_.target_right_distance_m = route_config.target_right_distance_m;
    side_config_.x_min_m = declare_parameter<double>("side_roi_x_min_m", -2.0);
    side_config_.x_max_m = declare_parameter<double>("side_roi_x_max_m", 3.0);
    side_config_.y_min_m = declare_parameter<double>("side_roi_y_min_m", 0.2);
    side_config_.y_max_m = declare_parameter<double>("side_roi_y_max_m", 5.0);
    side_config_.side_distance_gate_m = declare_parameter<double>("side_distance_gate_m", 1.0);
    side_config_.ransac_inlier_threshold_m =
      declare_parameter<double>("ransac_inlier_threshold_m", 0.08);
    side_config_.min_inliers =
      static_cast<std::size_t>(declare_parameter<int>("min_side_inliers", 8));
    side_config_.line_heading_gate_deg = declare_parameter<double>("line_heading_gate_deg", 45.0);

    travel_speed_mmps_ = nonnegative_u16_param(*this, "travel_speed_mmps", 300);
    rotation_speed_ = nonnegative_u16_param(*this, "rotation_speed", 200);
    ultrasonic_work_distance_mm_ = nonnegative_u16_param(*this, "ultrasonic_work_distance_mm", 900);
    ultrasonic_adjust_distance_mm_ = nonnegative_u16_param(*this, "ultrasonic_adjust_distance_mm", 100);

    const auto points = load_pose_xy_file(poses_file);
    route_ = compile_route(points, route_config);
    if (route_.segments.empty()) {
      throw std::runtime_error("compiled route contains no segment");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_pose_ = Point2D{msg->pose.pose.position.x, msg->pose.pose.position.y};
        has_pose_ = true;
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        handle_scan(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu route points and %zu segments from %s; dry-run protocol is enabled",
      route_.points.size(), route_.segments.size(), poses_file.c_str());
  }

private:
  void handle_scan(const sensor_msgs::msg::LaserScan & scan)
  {
    if (!has_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for odometry");
      return;
    }

    const auto location = locate_on_route(route_, latest_pose_);
    if (!location.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cannot locate robot on route");
      return;
    }

    const auto segment_index = std::min<std::size_t>(
      location.segment_id - 1U,
      route_.segments.size() - 1U);
    const auto & segment = route_.segments[segment_index];
    const auto side_estimate = estimate_side_distance(
      scan_to_base_points(scan),
      follow_side_,
      side_config_);
    const auto frame = build_frame_from_segment(
      route_, segment, side_estimate,
      travel_speed_mmps_, rotation_speed_,
      ultrasonic_work_distance_mm_, ultrasonic_adjust_distance_mm_);
    const auto payload = encode_payload(frame);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "segment=%zu s=%.2f side_detected=%s dist=%.3f offset=%.3f confidence=%.2f payload=%s",
      location.segment_id,
      location.s_robot_m,
      side_estimate.detected ? "true" : "false",
      side_estimate.distance_m,
      side_estimate.active_offset_m,
      side_estimate.confidence,
      payload_to_hex(payload).c_str());
  }

  std::string odom_topic_;
  std::string scan_topic_;
  FollowSide follow_side_{FollowSide::Left};
  SideEstimatorConfig side_config_;
  CompiledRoute route_;
  Point2D latest_pose_;
  bool has_pose_{false};
  uint16_t travel_speed_mmps_{300U};
  uint16_t rotation_speed_{200U};
  uint16_t ultrasonic_work_distance_mm_{900U};
  uint16_t ultrasonic_adjust_distance_mm_{100U};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace pusher_nav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<pusher_nav_bridge::PusherNavBridgeNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("pusher_nav_bridge_node"), "fatal error: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
