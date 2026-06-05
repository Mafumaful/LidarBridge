#include "lio_scan_monitor/scan_analysis.hpp"
#include "lio_scan_monitor/terminal_display.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lio_scan_monitor
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

bool valid_scan_range(float range, double range_min, double range_max)
{
  return std::isfinite(range) && range >= range_min && range <= range_max;
}

std::vector<ScanPoint> scan_points_in_scan_frame(const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<ScanPoint> points;
  points.reserve(scan.ranges.size());

  if (scan.angle_increment == 0.0F) {
    return points;
  }

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!valid_scan_range(range, scan.range_min, scan.range_max)) {
      continue;
    }

    const auto angle = static_cast<double>(scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(scan.angle_increment));
    const auto range_m = static_cast<double>(range);
    points.push_back(ScanPoint{
      range_m * std::cos(angle),
      range_m * std::sin(angle),
      index});
  }

  return points;
}

std::vector<ScanPoint> transform_scan_points_to_base(
  const sensor_msgs::msg::LaserScan & scan,
  const geometry_msgs::msg::TransformStamped & scan_to_base)
{
  const auto scan_points = scan_points_in_scan_frame(scan);
  std::vector<ScanPoint> base_points;
  base_points.reserve(scan_points.size());

  for (const auto & scan_point : scan_points) {
    geometry_msgs::msg::PointStamped source;
    source.header = scan.header;
    source.point.x = scan_point.x_m;
    source.point.y = scan_point.y_m;
    source.point.z = 0.0;

    geometry_msgs::msg::PointStamped target;
    tf2::doTransform(source, target, scan_to_base);
    base_points.push_back(ScanPoint{
      target.point.x,
      target.point.y,
      scan_point.scan_index});
  }

  return base_points;
}

std_msgs::msg::ColorRGBA marker_color(float red, float green, float blue, float alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

std_msgs::msg::ColorRGBA marker_color(const RgbaColor & source)
{
  return marker_color(source.r, source.g, source.b, source.a);
}

geometry_msgs::msg::Point point_from_polar(double radius_m, double angle_rad, double z_m)
{
  geometry_msgs::msg::Point point;
  point.x = radius_m * std::cos(angle_rad);
  point.y = radius_m * std::sin(angle_rad);
  point.z = z_m;
  return point;
}

builtin_interfaces::msg::Time time_to_msg(const rclcpp::Time & time)
{
  auto nanoseconds = time.nanoseconds();
  auto seconds = nanoseconds / kNanosecondsPerSecond;
  auto remainder = nanoseconds % kNanosecondsPerSecond;

  if (remainder < 0) {
    --seconds;
    remainder += kNanosecondsPerSecond;
  }

  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(seconds);
  msg.nanosec = static_cast<uint32_t>(remainder);
  return msg;
}

builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
{
  builtin_interfaces::msg::Duration msg;
  if (seconds <= 0.0) {
    return msg;
  }

  const auto nanoseconds = static_cast<int64_t>(
    std::llround(seconds * static_cast<double>(kNanosecondsPerSecond)));
  msg.sec = static_cast<int32_t>(nanoseconds / kNanosecondsPerSecond);
  msg.nanosec = static_cast<uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return msg;
}

visualization_msgs::msg::Marker make_delete_marker(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time_to_msg(stamp);
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

}  // namespace

class LioScanMonitorNode : public rclcpp::Node
{
public:
  LioScanMonitorNode()
  : Node("lio_scan_monitor_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    enable_visualization_ = declare_parameter<bool>("enable_visualization", true);
    enable_sector_visualization_ = declare_parameter<bool>("enable_sector_visualization", true);
    terminal_options_.use_color = declare_parameter<bool>("terminal_use_color", true);
    terminal_options_.refresh_in_place =
      declare_parameter<bool>("terminal_refresh_in_place", true);
    visualization_topic_ = declare_parameter<std::string>(
      "visualization_topic",
      "/lio_scan_monitor/obstacles");
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 1.0);
    sector_marker_radius_m_ = declare_parameter<double>("sector_marker_radius_m", 3.0);
    sector_fill_alpha_ = declare_parameter<double>("sector_fill_alpha", 0.08);

    const auto sector_width_deg = declare_parameter<double>("scan_sector_width_deg", 45.0);
    sector_width_rad_ = sector_width_deg * kPi / 180.0;
    if (sector_width_rad_ <= 0.0) {
      throw std::runtime_error("scan_sector_width_deg must be greater than 0");
    }

    log_period_ms_ = declare_parameter<int>("log_period_ms", 500);
    if (log_period_ms_ < 50) {
      RCLCPP_WARN(
        get_logger(),
        "log_period_ms=%d is too small; clamping to 50 ms",
        log_period_ms_);
      log_period_ms_ = 50;
    }
    if (sector_marker_radius_m_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "sector_marker_radius_m=%.3f is invalid; clamping to 3.0 m",
        sector_marker_radius_m_);
      sector_marker_radius_m_ = 3.0;
    }
    if (sector_fill_alpha_ < 0.0) {
      sector_fill_alpha_ = 0.0;
    } else if (sector_fill_alpha_ > 1.0) {
      sector_fill_alpha_ = 1.0;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (enable_visualization_) {
      marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic_, 10);
    }

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(10),
      std::bind(&LioScanMonitorNode::odom_callback, this, std::placeholders::_1));

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LioScanMonitorNode::scan_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(log_period_ms_),
      std::bind(&LioScanMonitorNode::log_latest_state, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening for Odometry on '%s' and LaserScan on '%s'; obstacles are evaluated in '%s'",
      odom_topic_.c_str(),
      scan_topic_.c_str(),
      base_frame_id_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & position = msg->pose.pose.position;
    const auto & orientation = msg->pose.pose.orientation;

    latest_pose_ = TerminalPose{
      position.x,
      position.y,
      position.z,
      yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
    };
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<ScanPoint> base_points;

    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "LaserScan message has an empty frame_id; cannot evaluate obstacles in '%s'",
        base_frame_id_.c_str());
      return;
    }

    if (msg->header.frame_id == base_frame_id_) {
      base_points = scan_points_in_scan_frame(*msg);
    } else {
      try {
        const auto scan_to_base = tf_buffer_->lookupTransform(
          base_frame_id_,
          msg->header.frame_id,
          tf2::TimePointZero);
        base_points = transform_scan_points_to_base(*msg, scan_to_base);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Cannot transform LaserScan from '%s' to '%s': %s",
          msg->header.frame_id.c_str(),
          base_frame_id_.c_str(),
          ex.what());
        return;
      }
    }

    latest_obstacles_ = analyze_points_8_directions(base_points, sector_width_rad_);

    if (enable_visualization_ && marker_publisher_) {
      publish_obstacle_markers();
    }
  }

  void log_latest_state()
  {
    std::cout << render_terminal_panel(
      latest_pose_,
      latest_obstacles_,
      odom_topic_,
      scan_topic_,
      base_frame_id_,
      terminal_options_);
    std::cout.flush();
  }

  void publish_obstacle_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const auto stamp = get_clock()->now();

    if (enable_sector_visualization_) {
      append_sector_markers(marker_array, stamp);
    }

    for (std::size_t index = 0; index < latest_obstacles_.size(); ++index) {
      const auto & obstacle = latest_obstacles_[index];
      const auto id = static_cast<int>(index);

      if (!obstacle.detected) {
        marker_array.markers.push_back(
          make_delete_marker(base_frame_id_, stamp, "obstacle_points", id));
        marker_array.markers.push_back(
          make_delete_marker(base_frame_id_, stamp, "obstacle_vectors", id));
        marker_array.markers.push_back(
          make_delete_marker(base_frame_id_, stamp, "obstacle_labels", id));
        continue;
      }

      visualization_msgs::msg::Marker point_marker;
      point_marker.header.frame_id = base_frame_id_;
      point_marker.header.stamp = time_to_msg(stamp);
      point_marker.ns = "obstacle_points";
      point_marker.id = id;
      point_marker.type = visualization_msgs::msg::Marker::SPHERE;
      point_marker.action = visualization_msgs::msg::Marker::ADD;
      point_marker.pose.position.x = obstacle.x_m;
      point_marker.pose.position.y = obstacle.y_m;
      point_marker.pose.position.z = 0.05;
      point_marker.pose.orientation.w = 1.0;
      point_marker.scale.x = 0.18;
      point_marker.scale.y = 0.18;
      point_marker.scale.z = 0.18;
      point_marker.color = marker_color(1.0F, 0.12F, 0.08F, 0.9F);
      set_marker_lifetime(point_marker);
      marker_array.markers.push_back(point_marker);

      visualization_msgs::msg::Marker vector_marker;
      vector_marker.header.frame_id = base_frame_id_;
      vector_marker.header.stamp = time_to_msg(stamp);
      vector_marker.ns = "obstacle_vectors";
      vector_marker.id = id;
      vector_marker.type = visualization_msgs::msg::Marker::ARROW;
      vector_marker.action = visualization_msgs::msg::Marker::ADD;
      vector_marker.pose.orientation.w = 1.0;
      vector_marker.scale.x = 0.04;
      vector_marker.scale.y = 0.08;
      vector_marker.scale.z = 0.12;
      vector_marker.color = marker_color(0.05F, 0.75F, 1.0F, 0.85F);
      geometry_msgs::msg::Point origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = 0.05;
      geometry_msgs::msg::Point obstacle_point;
      obstacle_point.x = obstacle.x_m;
      obstacle_point.y = obstacle.y_m;
      obstacle_point.z = 0.05;
      vector_marker.points.push_back(origin);
      vector_marker.points.push_back(obstacle_point);
      set_marker_lifetime(vector_marker);
      marker_array.markers.push_back(vector_marker);

      visualization_msgs::msg::Marker label_marker;
      label_marker.header.frame_id = base_frame_id_;
      label_marker.header.stamp = time_to_msg(stamp);
      label_marker.ns = "obstacle_labels";
      label_marker.id = id;
      label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label_marker.action = visualization_msgs::msg::Marker::ADD;
      label_marker.pose.position.x = obstacle.x_m;
      label_marker.pose.position.y = obstacle.y_m;
      label_marker.pose.position.z = 0.35;
      label_marker.pose.orientation.w = 1.0;
      label_marker.scale.z = 0.18;
      label_marker.color = marker_color(1.0F, 1.0F, 1.0F, 0.95F);
      label_marker.text = marker_label_text(obstacle);
      set_marker_lifetime(label_marker);
      marker_array.markers.push_back(label_marker);
    }

    marker_publisher_->publish(marker_array);
  }

  void append_sector_markers(
    visualization_msgs::msg::MarkerArray & marker_array,
    const rclcpp::Time & stamp)
  {
    constexpr int kArcSegments = 8;
    const auto sectors = build_direction_sectors_8(sector_width_rad_, sector_marker_radius_m_);

    for (std::size_t index = 0; index < sectors.size(); ++index) {
      const auto & sector = sectors[index];
      const auto id = static_cast<int>(index);
      const auto fill_color = direction_sector_color(index, static_cast<float>(sector_fill_alpha_));
      const auto line_color = direction_sector_color(index, 0.9F);
      const auto label_color = direction_sector_color(index, 1.0F);

      visualization_msgs::msg::Marker fill_marker;
      fill_marker.header.frame_id = base_frame_id_;
      fill_marker.header.stamp = time_to_msg(stamp);
      fill_marker.ns = "direction_sector_fill";
      fill_marker.id = id;
      fill_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      fill_marker.action = visualization_msgs::msg::Marker::ADD;
      fill_marker.pose.orientation.w = 1.0;
      fill_marker.scale.x = 1.0;
      fill_marker.scale.y = 1.0;
      fill_marker.scale.z = 1.0;
      fill_marker.color = marker_color(fill_color);

      geometry_msgs::msg::Point origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = 0.0;

      for (int segment = 0; segment < kArcSegments; ++segment) {
        const auto start_fraction = static_cast<double>(segment) / kArcSegments;
        const auto end_fraction = static_cast<double>(segment + 1) / kArcSegments;
        const auto angle_a =
          sector.start_angle_rad + ((sector.end_angle_rad - sector.start_angle_rad) * start_fraction);
        const auto angle_b =
          sector.start_angle_rad + ((sector.end_angle_rad - sector.start_angle_rad) * end_fraction);

        fill_marker.points.push_back(origin);
        fill_marker.points.push_back(point_from_polar(sector.radius_m, angle_a, 0.0));
        fill_marker.points.push_back(point_from_polar(sector.radius_m, angle_b, 0.0));
      }
      set_marker_lifetime(fill_marker);
      marker_array.markers.push_back(fill_marker);

      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = base_frame_id_;
      line_marker.header.stamp = time_to_msg(stamp);
      line_marker.ns = "direction_sector_lines";
      line_marker.id = id;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = 0.035;
      line_marker.color = marker_color(line_color);
      line_marker.points.push_back(origin);
      for (int segment = 0; segment <= kArcSegments; ++segment) {
        const auto fraction = static_cast<double>(segment) / kArcSegments;
        const auto angle =
          sector.start_angle_rad + ((sector.end_angle_rad - sector.start_angle_rad) * fraction);
        line_marker.points.push_back(point_from_polar(sector.radius_m, angle, 0.02));
      }
      line_marker.points.push_back(origin);
      set_marker_lifetime(line_marker);
      marker_array.markers.push_back(line_marker);

      visualization_msgs::msg::Marker label_marker;
      label_marker.header.frame_id = base_frame_id_;
      label_marker.header.stamp = time_to_msg(stamp);
      label_marker.ns = "direction_sector_labels";
      label_marker.id = id;
      label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label_marker.action = visualization_msgs::msg::Marker::ADD;
      label_marker.pose.position = point_from_polar(sector.radius_m * 0.72, sector.center_angle_rad, 0.22);
      label_marker.pose.orientation.w = 1.0;
      label_marker.scale.z = 0.18;
      label_marker.color = marker_color(label_color);
      label_marker.text = sector.name;
      set_marker_lifetime(label_marker);
      marker_array.markers.push_back(label_marker);
    }
  }

  void set_marker_lifetime(visualization_msgs::msg::Marker & marker) const
  {
    if (marker_lifetime_sec_ > 0.0) {
      marker.lifetime = duration_from_seconds(marker_lifetime_sec_);
    }
  }

  std::string marker_label_text(const DirectionObstacle & obstacle) const
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << obstacle.name << " " << obstacle.distance_m << "m";
    return oss.str();
  }

  std::string odom_topic_;
  std::string scan_topic_;
  std::string base_frame_id_ = "base_link";
  std::string visualization_topic_;
  double sector_width_rad_ = kPi / 4.0;
  double marker_lifetime_sec_ = 1.0;
  double sector_marker_radius_m_ = 3.0;
  double sector_fill_alpha_ = 0.08;
  int log_period_ms_ = 500;
  bool enable_visualization_ = true;
  bool enable_sector_visualization_ = true;
  TerminalDisplayOptions terminal_options_;

  std::optional<TerminalPose> latest_pose_;
  std::vector<DirectionObstacle> latest_obstacles_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lio_scan_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lio_scan_monitor::LioScanMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
